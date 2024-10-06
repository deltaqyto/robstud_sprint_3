#include <unordered_map>
#include <queue>
#include <stdexcept>
#include <optional>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CensorMatic : public rclcpp::Node {
protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr double_map_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlaid_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_annotated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cylinder_positions_pub_;
    rclcpp::TimerBase::SharedPtr map_upload_timer;

    const double object_tolerance = 0.10; // 10 cm discontinuity tolerance
    const double min_cluster_size = 0.27; // Minimum cluster size
    const double max_cluster_size = 0.35; // Maximum cluster size

    const double image_origin_x = -7.0;
    const double image_origin_y = -7.0;
    const double meters_per_pixel = 0.01;

    geometry_msgs::msg::PoseArray cylinder_poses_; // Currently detected poses
    cv::Mat lidar_image_;

public:
    CensorMatic() : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&CensorMatic::odom_callback, this, std::placeholders::_1));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CensorMatic::laser_callback, this, std::placeholders::_1));

        scan_map_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_raw", 10);
        raw_map_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_map_raw", 10);
        raw_map_annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_map_annotated", 10);
        overlaid_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlaid_map", 10);
        double_map_pub = this->create_publisher<sensor_msgs::msg::Image>("map_comparison", 10);



        cylinder_positions_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cylinder_positions", 10);
        cylinder_poses_.header.frame_id = "/base_scan";

        map_upload_timer = this->create_wall_timer(
                                                   std::chrono::milliseconds(20),
                                                   std::bind(&CensorMatic::map_upload_callback, this));    
}

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        make_lidar_mat(msg);
        std::vector<std::pair<size_t, size_t>> clusters;
        std::vector<double> angle_db;
        
        const auto& ranges = msg->ranges;
        const size_t num_points = ranges.size();

        size_t cluster_start = 0;
        size_t cluster_length = 1;
        
        // Determine objects by maximum displacement
        for (size_t i = 1; i <= num_points; i++) {
            size_t current = i % num_points;
            size_t previous = (i - 1) % num_points;

            if (std::abs(ranges[current] - ranges[previous]) <= object_tolerance) {
                cluster_length++;
            } else {
                if (cluster_length > 1) {
                    clusters.emplace_back(cluster_start, cluster_length);
                }
                cluster_start = current;
                cluster_length = 1;
            }

            size_t next = (i + 1) % num_points;
            double angle = std::atan2(ranges[next] * std::sin(msg->angle_increment) - ranges[current] * std::sin(2 * msg->angle_increment),
                                      ranges[next] * std::cos(msg->angle_increment) - ranges[current] * std::cos(2 * msg->angle_increment));
            angle_db.push_back(angle);
        }

        if (cluster_length > 1) {
            clusters.emplace_back(cluster_start, cluster_length);
        }

        std::vector<std::pair<size_t, size_t>> thresholded_clusters;
        std::vector<double> cluster_angles;

        for (const auto& cluster : clusters) {
            if (cluster.second < 5) {  // Remove fragment items
                continue;
            }
            double size = 0;
            double center_x = 0;
            double center_y = 0;
            //RCLCPP_INFO(this->get_logger(), "Size: %zu", cluster.second);
            if (check_cluster_threshold(msg, cluster, size, center_x, center_y)) {
                thresholded_clusters.push_back(cluster);
                double avg_angle = calculate_cluster_angle(msg, angle_db, cluster);
                cluster_angles.push_back(avg_angle);
                
                // TODO reject based on curvature
                if (isnan(avg_angle) || isinf(avg_angle)) {
                    continue;
                }
                //RCLCPP_INFO(this->get_logger(), "Curvature: %f", avg_angle);

                //double distance = msg->ranges[cluster.first];
                //double angle = msg->angle_min + cluster.first * msg->angle_increment;
                double x = center_x; //distance * std::cos(angle);
                double y = center_y; //distance * std::sin(angle);

                double robot_x = current_pose_.position.x;
                double robot_y = current_pose_.position.y;
                tf2::Quaternion q(
                    current_pose_.orientation.x,
                    current_pose_.orientation.y,
                    current_pose_.orientation.z,
                    current_pose_.orientation.w);
                double robot_yaw = tf2::getYaw(q);

                double x_odom = robot_x + x * std::cos(robot_yaw) - y * std::sin(robot_yaw);
                double y_odom = robot_y + x * std::sin(robot_yaw) + y * std::cos(robot_yaw);


                append_cylinder_position(x_odom, y_odom);
                RCLCPP_INFO(this->get_logger(), "X:%f Y:%f -> %f", x, y, size);
    
            }
        }
        publish_cylinders();

        RCLCPP_INFO(this->get_logger(), "Matches: %zu", thresholded_clusters.size());
    }

    void make_lidar_mat(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int image_size = static_cast<int>(std::ceil(msg->range_max / meters_per_pixel)) * 2;
        lidar_image_ = cv::Mat(image_size, image_size, CV_8UC1, cv::Scalar(0));  // Black image

        int center = image_size / 2;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            
            if (std::isnan(range) || std::isinf(range) || range < msg->range_min || range > msg->range_max) {
                continue;
            }

            float angle = msg->angle_min + i * msg->angle_increment;
            int x = static_cast<int>(std::round(range * std::cos(angle) / meters_per_pixel));
            int y = static_cast<int>(std::round(range * std::sin(angle) / meters_per_pixel));

            cv::circle(lidar_image_, cv::Point(center + x, center + y), 2, cv::Scalar(255), -1);
        }

        cv::flip(lidar_image_, lidar_image_, 0); // Flip for visual alignment
    }

    void map_upload_callback(){
        cv::Mat map_image = get_map_image();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", map_image).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "map";
        raw_map_raw_pub_->publish(*msg);

        cv::Mat transformed_image = transform_image(map_image, current_pose_);

        sensor_msgs::msg::Image::SharedPtr msg2 = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", transformed_image).toImageMsg();
        msg2->header.stamp = this->now();
        msg2->header.frame_id = "map";
        raw_map_annotated_pub_->publish(*msg2);

        
        cv::Mat overlay_image = overlay_images(transformed_image, lidar_image_);
        sensor_msgs::msg::Image::SharedPtr msg3 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay_image).toImageMsg();
        msg3->header.stamp = this->now();
        msg3->header.frame_id = "map";
        overlaid_map_pub_->publish(*msg3);

        //cv::Mat overlay_maps = overlay_images(transformed_image, scanned_map);
        //sensor_msgs::msg::Image::SharedPtr msg4 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", overlay_maps).toImageMsg();
        //msg4->header.stamp = this->now();
        //msg4->header.frame_id = "map";
        //double_map_pub->publish(*msg4);
    }

    cv::Mat transform_image(cv::Mat map_image, const geometry_msgs::msg::Pose& pose) {
        double pose_x = pose.position.x;
        double pose_y = pose.position.y;
        tf2::Quaternion q(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w);
        double yaw = tf2::getYaw(q);

        int center_x = static_cast<int>((pose_x - image_origin_x) / meters_per_pixel);
        int center_y = map_image.rows - static_cast<int>((pose_y - image_origin_y) / meters_per_pixel);

        cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2f(center_x, center_y), -yaw * 180 / M_PI, 1.0);

        rot_mat.at<double>(0,2) += map_image.cols/2 - center_x;
        rot_mat.at<double>(1,2) += map_image.rows/2 - center_y;

        cv::Mat transformed_image;
        cv::warpAffine(map_image, transformed_image, rot_mat, map_image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(205));
        return transformed_image;
    }

    cv::Mat overlay_images(const cv::Mat& img1, const cv::Mat& img2) {
        int max_width = std::max(img1.cols, img2.cols);
        int max_height = std::max(img1.rows, img2.rows);

        // Pad uncovered regions in dim blue
        cv::Mat output(max_height, max_width, CV_8UC3, cv::Scalar(0, 0, 50));

        // Helper to overlay on a channel
        auto overlay_channel = [&](const cv::Mat& img, int channel) {
            if (img.empty()) return;

            cv::Mat resized;
            if (img.type() == CV_8UC1) {
                resized = img;
            } else if (img.type() == CV_8UC3) {
                cv::cvtColor(img, resized, cv::COLOR_BGR2GRAY);
            } else {
                RCLCPP_WARN(this->get_logger(), "Odd image type");
                return;
            }

            int x_offset = (max_width - img.cols) / 2;
            int y_offset = (max_height - img.rows) / 2;

            cv::Mat roi(output, cv::Rect(x_offset, y_offset, img.cols, img.rows));
            cv::Mat overlay_channel(roi.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            overlay_channel.forEach<cv::Vec3b>([&](cv::Vec3b& pixel, const int position[]) -> void {
                pixel[channel] = resized.at<uchar>(position[0], position[1]);
            });
            cv::add(roi, overlay_channel, roi);
        };

        
        overlay_channel(img1, 2);
        overlay_channel(img2, 1);

        return output;
    }

    bool check_cluster_threshold(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                                 const std::pair<size_t, size_t>& cluster, 
                                 double& size, double& center_x, double& center_y) {
        size_t start = cluster.first;
        size_t length = cluster.second;
        size_t end = (start + length - 1) % scan->ranges.size();

        double x1 = scan->ranges[start] * std::cos(scan->angle_min + start * scan->angle_increment);
        double y1 = scan->ranges[start] * std::sin(scan->angle_min + start * scan->angle_increment);
        double x2 = scan->ranges[end] * std::cos(scan->angle_min + end * scan->angle_increment);
        double y2 = scan->ranges[end] * std::sin(scan->angle_min + end * scan->angle_increment);
        double cluster_size = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
        size = cluster_size;

        center_x = (x1 + x2)/2.0;
        center_y = (y1 + y2)/2.0;

        return (cluster_size >= min_cluster_size && cluster_size <= max_cluster_size);
    }

    double calculate_cluster_angle(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                                   const std::vector<double>& angle_db,
                                   const std::pair<size_t, size_t>& cluster) {
        size_t start = cluster.first;
        size_t length = cluster.second;
        double sum_angles = 0.0;

        for (size_t i = 0; i < length; i++) {
            size_t index = (start + i) % scan->ranges.size();
            sum_angles += angle_db[index];
        }

        return sum_angles / length;
    }

    void append_cylinder_position(double x, double y) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        cylinder_poses_.poses.push_back(pose);
    }

    void publish_cylinders() {
        cylinder_poses_.header.stamp = this->now();
        cylinder_positions_pub_->publish(cylinder_poses_);
        cylinder_poses_.poses.clear();
        cylinder_poses_.header.frame_id = "/odom";
    }

     cv::Mat get_map_image() {
        std::string package_name = "robstud_sprint_3";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string map_path = package_share_directory + "/data/map.pgm";
        
        cv::Mat map_image = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        if (map_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image from %s", map_path.c_str());
        } else {
            //RCLCPP_INFO(this->get_logger(), "Got map from %s", map_path.c_str());
        }
        return map_image;
    }

    geometry_msgs::msg::Pose current_pose_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CensorMatic>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}