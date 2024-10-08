#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapServer : public rclcpp::Node {
public:
    double map_x = -7;
    double map_y = -7;
    double res = 0.01;

    MapServer() : Node("map_server") {
        occupancy_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occ_map", 10);
        raw_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_map", 10);
        overlay_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay_map", 10);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&MapServer::map_callback, this, std::placeholders::_1));
        map_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MapServer::publish_map, this));
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        load_map();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    cv::Mat map_image_;
    cv::Mat subscribed_map_image_;
    nav_msgs::msg::OccupancyGrid occupancy_map_;

    void load_map() {
        std::string package_name = "robstud_sprint_3";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string map_path = package_share_directory + "/data/map.pgm";
        map_image_ = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image from %s", map_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded map from %s", map_path.c_str());
            create_occupancy_map();
        }
    }

    void create_occupancy_map() {
        occupancy_map_.header.frame_id = "map";
        occupancy_map_.info.resolution = res;
        occupancy_map_.info.width = map_image_.cols;
        occupancy_map_.info.height = map_image_.rows;
        occupancy_map_.info.origin.position.x = map_x;
        occupancy_map_.info.origin.position.y = map_y;
        occupancy_map_.info.origin.position.z = 0.0;
        occupancy_map_.info.origin.orientation.w = 1.0;
        occupancy_map_.data.resize(map_image_.total());
        for (int y = 0; y < map_image_.rows; ++y) {
            for (int x = 0; x < map_image_.cols; ++x) {
                int index = y * map_image_.cols + x;
                uchar pixel = map_image_.at<uchar>(map_image_.rows - 1 - y, x);
                if (pixel == 0) {
                    occupancy_map_.data[index] = 100;
                } else if (pixel == 255) {
                    occupancy_map_.data[index] = 0;
                } else {
                    occupancy_map_.data[index] = -1;
                }
            }
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        double scale_factor = res / msg->info.resolution ;
        int new_width = static_cast<int>(msg->info.width / scale_factor);
        int new_height = static_cast<int>(msg->info.height / scale_factor);
        
        cv::Mat temp(msg->info.height, msg->info.width, CV_8UC1);
        for (size_t i = 0; i < msg->data.size(); ++i) {
            int value = msg->data[i];
            if (value == -1) {
                temp.data[i] = 128;
            } else {
                temp.data[i] = static_cast<uchar>((100 - value) * 255 / 100);
            }
        }
        
        cv::Mat resized;
        cv::resize(temp, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
        
        int x_offset = static_cast<int>((msg->info.origin.position.x - map_x) / res);
        int y_offset = static_cast<int>((map_y + map_image_.rows * res - (msg->info.origin.position.y + msg->info.height * msg->info.resolution)) / res);
        
        subscribed_map_image_ = cv::Mat(map_image_.size(), CV_8UC1, cv::Scalar(128));
        cv::Rect roi(x_offset, y_offset, resized.cols, resized.rows);
        
        if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= subscribed_map_image_.cols && roi.y + roi.height <= subscribed_map_image_.rows) {
            resized.copyTo(subscribed_map_image_(roi));
        } else {
            cv::Rect valid_roi = roi & cv::Rect(0, 0, subscribed_map_image_.cols, subscribed_map_image_.rows);
            if (valid_roi.width > 0 && valid_roi.height > 0) {
                cv::Rect src_roi(valid_roi.x - roi.x, valid_roi.y - roi.y, valid_roi.width, valid_roi.height);
                resized(src_roi).copyTo(subscribed_map_image_(valid_roi));
            }
        }
        cv::flip(subscribed_map_image_, subscribed_map_image_, 0);
    }

    void publish_map() {
        //publish_static_transform();
        if (!map_image_.empty()) {
            occupancy_map_.header.stamp = this->now();
            occupancy_map_pub_->publish(occupancy_map_);

            sensor_msgs::msg::Image::SharedPtr raw_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", map_image_).toImageMsg();
            raw_msg->header.stamp = this->now();
            raw_msg->header.frame_id = "map";
            raw_map_pub_->publish(*raw_msg);

            if (!subscribed_map_image_.empty() && subscribed_map_image_.size() == map_image_.size()) {
                cv::Mat overlay;
                cv::addWeighted(map_image_, 0.5, subscribed_map_image_, 0.5, 0.0, overlay);
                sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", overlay).toImageMsg();
                overlay_msg->header.stamp = this->now();
                overlay_msg->header.frame_id = "map";
                overlay_map_pub_->publish(*overlay_msg);
            }
        }
    }

    void publish_static_transform() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}