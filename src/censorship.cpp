#include <unordered_map>
#include <queue>
#include <stdexcept>
#include <optional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

class CensorMatic: public rclcpp::Node {
protected:
    // TODO subscribers for the laser data, odometry
    // TODO publishers for images -> scan map raw, scan map annotated
    // TODO publisher for a pose, called cylinder_position
public:
    CensorMatic(): Node("task_planner") {
        // TODO get subscribers set up
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // TODO Save just the pose to variable
    }

    void publish_cylinder_position(double x, double y){
        // Publish a pose, with z = 0, and default rotations
    }

    // TODO variables for just the pose 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CensorMatic>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}