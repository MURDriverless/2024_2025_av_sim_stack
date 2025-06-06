// src/total_reader_absolute.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ouster/os_pcap.h>
#include <ouster/types.h>
#include <ouster/lidar_scan.h>
#include <ouster/client.h>
#include <ouster/version.h>
#include <ouster/field.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cstring>
#include <chrono>
#include <memory>
#include <string>

using namespace ouster;
using namespace ouster::sensor;
using namespace ouster::sensor_utils;

class TotalReaderNode : public rclcpp::Node {
public:
  TotalReaderNode()
  : Node("total_reader_absolute"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 절대 경로로 JSON / PCAP 설정
    const std::string json_path = "/home/jaewonl/lidar_pipeline/src/lidar_integration/data/straight_json.json";
    const std::string pcap_path = "/home/jaewonl/lidar_pipeline/src/lidar_integration/data/straight_pcap.pcap";
    const double skip_time = 0.0;

    RCLCPP_INFO(get_logger(), "Loading metadata from %s", json_path.c_str());
    // metadata load
    sensor_info raw_info;
    try {
      raw_info = metadata_from_json(json_path);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Failed to load JSON metadata: %s", e.what());
      rclcpp::shutdown();
      return;
    }
    info_ = std::make_shared<sensor_info>(raw_info);

    RCLCPP_INFO(get_logger(), "Opening PCAP: %s", pcap_path.c_str());
    try {
      pcap_reader_ = std::make_shared<PcapReader>(pcap_path);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "Failed to open PCAP: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // 준비된 정보로 LUT, ScanBatcher, LidarScan 생성
    lut_     = make_xyz_lut(*info_, true);
    batcher_ = std::make_unique<ScanBatcher>(*info_);
    scan_    = LidarScan(*info_);

    imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("/os1/imu", 10);
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::QoS(10).reliable());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this, skip_time]() { process_lidar(skip_time); }
    );
  }

private:
  void process_lidar(double skip_time) {
    if (!pcap_reader_ || !pcap_reader_->next_packet()) {
      RCLCPP_WARN(get_logger(), "[1] End of PCAP reached, rewinding");
      pcap_reader_->reset();
      scan_ = LidarScan(*info_);
      return;
    }

    const auto& pkt_info = pcap_reader_->current_info();
    int64_t ts = pkt_info.timestamp.count();
    if (ts < static_cast<int64_t>(skip_time * 1e9)) {
      RCLCPP_DEBUG(get_logger(), "[2] Skipping before skip_time");
      return;
    }

    const uint8_t* data = pcap_reader_->current_data();
    // IMU on port 7503
    if (pkt_info.dst_port == 7503 && pkt_info.packet_size >= 48) {
      RCLCPP_DEBUG(get_logger(), "[3] IMU packet received");
      auto read_f = [](const uint8_t* ptr){
        float v; std::memcpy(&v, ptr, sizeof(v)); return v;
      };
      sensor_msgs::msg::Imu msg;
      msg.header.stamp    = now();
      msg.header.frame_id = "os1_imu";
      msg.linear_acceleration.x = read_f(data + 24);
      msg.linear_acceleration.y = read_f(data + 28);
      msg.linear_acceleration.z = read_f(data + 32);
      msg.angular_velocity.x    = read_f(data + 36);
      msg.angular_velocity.y    = read_f(data + 40);
      msg.angular_velocity.z    = read_f(data + 44);
      imu_pub_->publish(msg);
      return;
    }

    // LiDAR on port 7502
    if (pkt_info.dst_port == 7502 && pkt_info.packet_size > 0) {
      RCLCPP_DEBUG(get_logger(), "[4] LiDAR packet received");
      LidarPacket pkt(static_cast<int>(pkt_info.packet_size));
      std::memcpy(pkt.buf.data(), data, pkt_info.packet_size);

      if (!(*batcher_)(pkt, scan_)) {
        RCLCPP_DEBUG(get_logger(), "[5] Incomplete scan, buffering");
        return;
      }

      // 원거리(range) → XYZ
      auto ranges = scan_.field<uint32_t>(ChanField::RANGE);
      auto xyz    = cartesian(ranges, lut_);

      // reflectivity (uint8_t)
      auto refl8 = scan_.field<uint8_t>(ChanField::REFLECTIVITY);
      Eigen::Map<const Eigen::Array<uint8_t, Eigen::Dynamic, 1>> refl(
        refl8.data(), refl8.size());

      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.reserve(xyz.rows());
      for (int i = 0; i < xyz.rows(); ++i) {
        const auto& pt = xyz.row(i);
        if (std::isfinite(pt(0)) && std::isfinite(pt(1)) && std::isfinite(pt(2))) {
          pcl::PointXYZI p;
          p.x = pt(0); p.y = pt(1); p.z = pt(2);
          p.intensity = static_cast<float>(refl(i));
          cloud.push_back(p);
        }
      }
      cloud.width   = cloud.size();
      cloud.height  = 1;
      cloud.is_dense = false;

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.stamp    = now();
      msg.header.frame_id = "os1";

      // map ← os1 TF
      try {
        auto tf = tf_buffer_.lookupTransform("map", "os1", tf2::TimePointZero);
        tf2::doTransform(msg, msg, tf);
        msg.header.frame_id = "map";
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "[7] TF failed, publishing in os1: %s", e.what());
      }

      lidar_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "[8] Published %zu points", cloud.size());

      // 다음 프레임 준비
      scan_ = LidarScan(*info_);
      RCLCPP_DEBUG(get_logger(), "[9] Next frame initialized");
    }
  }

  std::shared_ptr<sensor_info> info_;
  std::shared_ptr<PcapReader>            pcap_reader_;
  std::unique_ptr<ScanBatcher>           batcher_;
  LidarScan                              scan_;
  XYZLut                                 lut_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::TimerBase::SharedPtr           timer_;
  tf2_ros::Buffer                        tf_buffer_;
  tf2_ros::TransformListener             tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TotalReaderNode>());
  rclcpp::shutdown();
  return 0;
}
