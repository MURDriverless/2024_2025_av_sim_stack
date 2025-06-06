#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ouster/os_pcap.h>
#include <ouster/types.h>

#include <cstring>
#include <chrono>

using namespace ouster::sensor;
using namespace ouster::sensor_utils;

class ImuReaderNode : public rclcpp::Node {
public:
    ImuReaderNode() : Node("imu_reader") {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("lidar_integration");
        std::string json_path = this->declare_parameter<std::string>("json_path", pkg_path + "/data/lidar_metadata.json");
        std::string pcap_path = this->declare_parameter<std::string>("pcap_path", pkg_path + "/data/lidar_data.pcap");
        skip_time_ = this->declare_parameter<double>("skip_time");

        RCLCPP_INFO(this->get_logger(), "Opening PCAP: %s", pcap_path.c_str());
        pcap_reader_ = std::make_shared<PcapReader>(pcap_path);
        if (!pcap_reader_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PCAP");
            rclcpp::shutdown();
        }

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/os1/imu", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&ImuReaderNode::process_packet, this)
        );
    }

private:
    void process_packet() {
        if (!pcap_reader_->next_packet()) {
            RCLCPP_WARN(this->get_logger(), "End of PCAP reached, rewinding...");
            pcap_reader_->reset();
            return;
        }

        const auto& pkt_info = pcap_reader_->current_info();
        constexpr int imu_port = 7503;
        constexpr int imu_packet_size = 48;

        // Skip non-IMU packets
        if (pkt_info.dst_port != imu_port) return;


        // Skip based on timestamp
        std::chrono::nanoseconds ts = pkt_info.timestamp;
        if (ts.count() < static_cast<int64_t>(skip_time_ * 1e9)) return;

        const uint8_t* data = pcap_reader_->current_data();
        if (pkt_info.packet_size < imu_packet_size) {
            RCLCPP_WARN(this->get_logger(), "Skipping IMU packet: too small (%lu bytes)", pkt_info.packet_size);
            return;
        }

        // Helper to read float32 from little-endian byte buffer
        auto read_float32 = [](const uint8_t* ptr) -> float {
            float value;
            std::memcpy(&value, ptr, sizeof(float));
            return value;
        };

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "os1_imu";

        // From official Ouster IMU format: word 6–11 = byte 24–47
        msg.linear_acceleration.x = read_float32(data + 24);
        msg.linear_acceleration.y = read_float32(data + 28);
        msg.linear_acceleration.z = read_float32(data + 32);

        msg.angular_velocity.x = read_float32(data + 36);
        msg.angular_velocity.y = read_float32(data + 40);
        msg.angular_velocity.z = read_float32(data + 44);

        imu_pub_->publish(msg);
    }

    double skip_time_;
    std::shared_ptr<PcapReader> pcap_reader_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuReaderNode>());
    rclcpp::shutdown();
    return 0;
}