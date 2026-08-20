// 点云降采样中继节点。
//
// 目的：远程 RViz 场景下，NUC 通过 WiFi 把 0.55MB 的 /odin1/cloud_slam
// 发给本机时，单帧会被拆成多个 UDP 分片，WiFi 丢一片就整帧作废，导致本机
// 完全收不到点云。本节点在 NUC 本机订阅完整点云（本机通信链路稳定，
// 能收全量），做体素降采样 + 剔除无效点后，发布一个显著更小的
// /odin1/cloud_slam_lite，减少跨机 UDP/IP 分片数量和整帧丢失概率。
//
// 隔离原则：只新增 lite 话题，不改动原始 /odin1/cloud_slam，因此建图、里程计
// 等依赖完整点云的链路完全不受影响。

#include <cmath>
#include <memory>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CloudDownsampleRelay final : public rclcpp::Node {
public:
    explicit CloudDownsampleRelay(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("cloud_downsample_relay", options)
    {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/odin1/cloud_slam_lite");
        // 默认 0.30m：实测该值下单帧约 16KB，跨 WiFi 约 3.3Hz，兼顾流畅与点云密度。
        // voxel 越大越流畅但越稀疏（0.5m→7Hz 稀疏，0.2m→1.3Hz 较密）。
        voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.30);
        // 剔除被驱动置零的无效点（confidence 不足时 xyz 全置 0）。
        drop_zero_points_ = this->declare_parameter<bool>("drop_zero_points", true);
        // 提示阈值：抽稀后字节数超过它就告警，说明跨机分片数量仍偏多。
        warn_bytes_ = this->declare_parameter<int>("warn_bytes", 60000);

        // 订阅侧用 RELIABLE 匹配驱动的 cloud_slam 发布 QoS；本机内通信不丢包。
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
        // 发布侧是远程显示链路，必须允许丢帧，不能因为 WiFi/RViz 确认慢而阻塞
        // 本节点的上游 cloud_slam 回调。
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, sub_qos,
            std::bind(&CloudDownsampleRelay::onCloud, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "cloud_downsample_relay: %s -> %s | voxel=%.3fm drop_zero=%s",
            input_topic_.c_str(), output_topic_.c_str(), voxel_leaf_size_,
            drop_zero_points_ ? "true" : "false");
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 只取 x/y/z 几何：远程 RViz 看的是环境结构，intensity/confidence/offset_time
        // 对此非必需。源点云 intensity 是 UINT8，与 PCL PointXYZI 期望的 FLOAT32
        // 不匹配（会报 "Failed to find match for field 'intensity'"），直接用
        // PointXYZ 既规避字段类型问题，又把单点从 16B 降到 12B，进一步减小体积。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud_in);

        // 剔除驱动置零的无效点（confidence 不足时 xyz 被置 0），否则它们会在原点
        // 堆出一个伪密集区，既浪费带宽又污染体素中心。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_valid(new pcl::PointCloud<pcl::PointXYZ>());
        if (drop_zero_points_) {
            cloud_valid->points.reserve(cloud_in->points.size());
            for (const auto& p : cloud_in->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                    continue;
                }
                if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) {
                    continue;
                }
                cloud_valid->points.push_back(p);
            }
            cloud_valid->width = cloud_valid->points.size();
            cloud_valid->height = 1;
            cloud_valid->is_dense = true;
        } else {
            cloud_valid = cloud_in;
        }

        // 体素降采样：把稠密点云压到远小于原始点数的规模。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_valid);
        const float leaf = static_cast<float>(voxel_leaf_size_);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*cloud_out);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud_out, out_msg);

        // 保留原始帧和时间戳，RViz 里 TF 对齐与原点云一致。
        out_msg.header = msg->header;

        publisher_->publish(out_msg);

        const size_t out_bytes = out_msg.data.size();
        if (static_cast<int>(out_bytes) > warn_bytes_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "lite 单帧 %zu 字节仍超过 %d，跨机分片数量偏多、丢帧风险较高；可调大 voxel_leaf_size。",
                out_bytes, warn_bytes_);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "lite 点数 %u，单帧 %zu 字节（原始 %u 点）。",
                out_msg.width * out_msg.height, out_bytes, msg->width * msg->height);
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    double voxel_leaf_size_{0.30};
    bool drop_zero_points_{true};
    int warn_bytes_{60000};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudDownsampleRelay>());
    rclcpp::shutdown();
    return 0;
}
