#ifndef CEN_H
#define CEN_H

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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/utils.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @class CylinderIdentifier
 * @brief Identifies cylinders in the environment using laser scan data and mapping.
 */
class CylinderIdentifier : public rclcpp::Node {
protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr double_map_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlaid_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_annotated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cylinder_positions_pub_;
    rclcpp::TimerBase::SharedPtr map_upload_timer;

    const double object_tolerance = 0.10; // 10 cm discontinuity tolerance
    const double min_cluster_size = 0.1; // Minimum cluster size
    const double max_cluster_size = 0.4; // Maximum cluster size

    const double image_origin_x = -7.0;
    const double image_origin_y = -7.0;
    const double meters_per_pixel = 0.01;

    geometry_msgs::msg::PoseArray cylinder_poses_; // Currently detected poses
    cv::Mat lidar_image_;
    cv::Mat scanned_image_;

public:
    CylinderIdentifier();

private:
    /**
     * @brief Callback function for odometry messages
     * 
     * @param msg Received Odometry message
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback function for laser scan messages
     * Processes incoming laser scan data to detect cylinders.
     * 
     * @param msg Received LaserScan message
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Callback function for occupancy grid map messages
     * Updates the internal map representation based on received occupancy grid data.
     * 
     * @param msg Received OccupancyGrid message
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief Creates a image representation of the laser scan data
     * 
     * @param msg LaserScan message
     */
    void make_lidar_mat(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Callback function for the map upload timer
     * 
     * Periodically publishes updated map and overlay information.
     */
    void map_upload_callback();

    /**
     * @brief Transforms an image based on the current pose
     * 
     * @param map_image The input image to be transformed
     * @param pose The current pose used for the transformation
     * @return cv::Mat The transformed image
     */
    cv::Mat transform_image(cv::Mat map_image, const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Overlays two images
     * 
     * @param img1 The first input image
     * @param img2 The second input image
     * @return cv::Mat The resulting overlaid image
     */
    cv::Mat overlay_images(const cv::Mat& img1, const cv::Mat& img2);

    /**
     * @brief Checks if a cluster meets the threshold criteria for a cylinder
     * 
     * @param scan Shared pointer to the LaserScan message
     * @param cluster The cluster to be checked
     * @param size Output parameter for the calculated cluster size
     * @param center_x Output parameter for the cluster's center x-coordinate
     * @param center_y Output parameter for the cluster's center y-coordinate
     * @return true If the cluster meets the cylinder criteria
     * @return false If the cluster does not meet the cylinder criteria
     */
    bool check_cluster_threshold(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                                 const std::pair<size_t, size_t>& cluster, 
                                 double& size, double& center_x, double& center_y);

    /**
     * @brief Calculates the average angle of a cluster
     * 
     * @param scan Shared pointer to the LaserScan message
     * @param angle_db Vector of pre-calculated angles
     * @param cluster The cluster for which to calculate the average angle
     * @return double The calculated average angle of the cluster
     */
    double calculate_cluster_angle(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                                   const std::vector<double>& angle_db,
                                   const std::pair<size_t, size_t>& cluster);

    /**
     * @brief Adds a cylinder position to the list of detected cylinders
     * 
     * @param x The x-coordinate of the cylinder
     * @param y The y-coordinate of the cylinder
     */
    void append_cylinder_position(double x, double y);

    /**
     * @brief Publishes the detected cylinder positions
     */
    void publish_cylinders();

    /**
     * @brief Retrieves the map image from file
     * 
     * @return cv::Mat The loaded map image
     */
    cv::Mat get_map_image();

    geometry_msgs::msg::Pose current_pose_;

};


#endif