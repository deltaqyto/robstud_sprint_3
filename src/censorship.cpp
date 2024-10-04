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

class CensorMatic : public rclcpp::Node {
protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_annotated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cylinder_positions_pub_;

    const double object_tolerance = 0.05; // 5 cm discontinuity tolerance
    const double min_cluster_size = 0.14; // Minimum cluster size
    const double max_cluster_size = 0.30; // Maximum cluster size

    geometry_msgs::msg::PoseArray cylinder_poses_; // Currently detected poses

public:
    CensorMatic() : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&CensorMatic::odom_callback, this, std::placeholders::_1));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CensorMatic::laser_callback, this, std::placeholders::_1));

        scan_map_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_raw", 10);
        scan_map_annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_annotated", 10);
        cylinder_positions_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cylinder_positions", 10);
        cylinder_poses_.header.frame_id = "/base_scan";
    
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
            //RCLCPP_INFO(this->get_logger(), "Size: %zu", cluster.second);
            if (check_cluster_threshold(msg, cluster)) {
                thresholded_clusters.push_back(cluster);
                double avg_angle = calculate_cluster_angle(msg, angle_db, cluster);
                cluster_angles.push_back(avg_angle);
                //RCLCPP_INFO(this->get_logger(), "Size: %f", avg_angle);

                // TODO reject based on curvature


                // TODO determine center of cluster
                double distance = msg->ranges[cluster.first];
                double angle = msg->angle_min + cluster.first * msg->angle_increment;
                double x = distance * std::cos(angle);
                double y = distance * std::sin(angle);
                append_cylinder_position(x, y);
                RCLCPP_INFO(this->get_logger(), "X:%f Y:%f", x, y);
    
            }
        }
        publish_cylinders();

        RCLCPP_INFO(this->get_logger(), "Matches: %zu", thresholded_clusters.size());
    }

    bool check_cluster_threshold(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                                 const std::pair<size_t, size_t>& cluster) {
        size_t start = cluster.first;
        size_t length = cluster.second;
        size_t end = (start + length - 1) % scan->ranges.size();

        double x1 = scan->ranges[start] * std::cos(scan->angle_min + start * scan->angle_increment);
        double y1 = scan->ranges[start] * std::sin(scan->angle_min + start * scan->angle_increment);
        double x2 = scan->ranges[end] * std::cos(scan->angle_min + end * scan->angle_increment);
        double y2 = scan->ranges[end] * std::sin(scan->angle_min + end * scan->angle_increment);
        double cluster_size = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
        //RCLCPP_INFO(this->get_logger(), "Size: %f", cluster_size);
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
        append_cylinder_position(0, 0);
        cylinder_poses_.header.stamp = this->now();
        cylinder_positions_pub_->publish(cylinder_poses_);
        cylinder_poses_.poses.clear();
        cylinder_poses_.header.frame_id = "/odom";
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