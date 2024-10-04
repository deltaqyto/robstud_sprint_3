#include <unordered_map>
#include <queue>
#include <stdexcept>
#include <optional>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"

class CensorMatic : public rclcpp::Node {
protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_annotated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cylinder_position_pub_;

public:
    CensorMatic() : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&CensorMatic::odom_callback, this, std::placeholders::_1));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CensorMatic::laser_callback, this, std::placeholders::_1));

        scan_map_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_raw", 10);
        scan_map_annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("scan_map_annotated", 10);
        cylinder_position_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("cylinder_position", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // TODO Process cylinders
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