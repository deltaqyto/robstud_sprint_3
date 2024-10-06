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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_annotated_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_annotated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cylinder_positions_pub_;
    rclcpp::TimerBase::SharedPtr map_upload_timer;

    const double object_tolerance = 0.10; // 10 cm discontinuity tolerance
    const double min_cluster_size = 0.27; // Minimum cluster size
    const double max_cluster_size = 0.35; // Maximum cluster size

    geometry_msgs::msg::PoseArray cylinder_poses_; // Currently detected poses

public:
    CensorMatic() : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&CensorMatic::odom_callback, this, std::placeholders::_1));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CensorMatic::laser_callback, this, std::placeholders::_1));

        scan_map_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_raw", 10);
        scan_map_annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_annotated", 10);
        raw_map_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_map_raw", 10);
        raw_map_annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_map_annotated", 10);
        
        cylinder_positions_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cylinder_positions", 10);
        cylinder_poses_.header.frame_id = "/base_scan";

        map_upload_timer = this->create_wall_timer(
                                                   std::chrono::seconds(1),
                                                   std::bind(&CensorMatic::map_upload_callback, this));    
}

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

    void map_upload_callback(){
        cv::Mat map_image = get_map_image();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", map_image).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "map";
        raw_map_raw_pub_->publish(*msg);


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