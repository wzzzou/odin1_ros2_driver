#include <fstream>
#include <memory>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class StaticPcdMapPublisher final : public rclcpp::Node {
public:
    explicit StaticPcdMapPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("static_pcd_map_publisher", options)
    {
        pcd_map_file_ = this->declare_parameter<std::string>("pcd_map_file", "");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "odin1_map");
        topic_name_ = this->declare_parameter<std::string>("topic_name", "/odin1/map_cloud");

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

        initialized_ = loadAndPublish();
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    bool loadAndPublish()
    {
        if (pcd_map_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_map_file' is empty");
            return false;
        }

        std::ifstream input(pcd_map_file_);
        if (!input.good()) {
            RCLCPP_ERROR(this->get_logger(), "PCD file is missing or unreadable: %s", pcd_map_file_.c_str());
            return false;
        }

        pcl::PCLPointCloud2 cloud;
        const int load_result = pcl::io::loadPCDFile(pcd_map_file_, cloud);
        if (load_result != 0 || cloud.data.empty() || cloud.width == 0 || cloud.height == 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load a valid PCD file: %s", pcd_map_file_.c_str());
            return false;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl_conversions::moveFromPCL(cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.header.stamp = this->now();

        publisher_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published PCD map '%s' to topic '%s'", pcd_map_file_.c_str(), topic_name_.c_str());
        return true;
    }

    std::string pcd_map_file_;
    std::string frame_id_;
    std::string topic_name_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    bool initialized_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StaticPcdMapPublisher>();
    if (!node->initialized()) {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
