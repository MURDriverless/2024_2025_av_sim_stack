// src/total_reader.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <memory>                     // std::shared_ptr, std::make_shared
#include <stdexcept>                 // std::runtime_error
#include <string>                    // std::string
#include <vector>                    // std::vector 
#include <chrono>                    // std::chrono::duration

#include <ouster/os_pcap.h>           // PcapReader, packet_info
#include <ouster/lidar_scan.h>        // LidarScan, ScanBatcher, cartesian, XYZLut
#include <ouster/field.h>             // ChanField
#include <ouster/types.h>             // sensor_info, packet_format
#include <ouster/metadata.h>          // metadata_from_json
#include <ouster/indexed_pcap_reader.h>// IndexedPcapReader, PcapIndex
#include <ouster/client.h>            // LidarPacket
#include <ouster/version.h>           // version info

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>



using namespace ouster;
using namespace ouster::sensor;
using namespace ouster::sensor_utils;


class TotalReaderNode : public rclcpp::Node {
public:
    TotalReaderNode()
    : Node("total_reader"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        
        auto pkg = ament_index_cpp::get_package_share_directory("lidar_integration");
        std::vector<std::string> default_json = {
            pkg + "/data/straight_json.json"
        };
        std::string default_pcap = pkg + "/data/straight_pcap.pcap";

        // refers to the indexed_pcap_reader_test.cpp
        // ============================================================
        json_paths_ = declare_parameter<std::vector<std::string>>("json_paths", default_json);
        // PCAP path
        pcap_path_  = declare_parameter<std::string>            ("pcap_path",  default_pcap);
        // skip_time in secs
        skip_time_  = declare_parameter<double>("skip_time", 0.0);

       //check whether the parameters are set correctly and files exist
        // ============================================================
        for (const auto &jp : json_paths_) {
            if (!std::filesystem::exists(jp)) {
                RCLCPP_ERROR(get_logger(), "JSON file not found: %s", jp.c_str());
                throw std::runtime_error("JSON file not found");
            }
        }
        if (!std::filesystem::exists(pcap_path_)) {
            RCLCPP_ERROR(get_logger(), "PCAP file not found: %s", pcap_path_.c_str());
            throw std::runtime_error("PCAP file not found");
        }

        // ============================================================
        // Load json metadata files
        // ============================================================
        try {
            for (const auto &jp : json_paths_) {
                auto info_tmp = metadata_from_json(jp);
                sensor_infos_.push_back(std::move(info_tmp));
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to load sensor_info: %s", e.what());
            throw;
        }

        // IndexedPcapReader 
        // ============================================================
        try {
            index_pcap_reader_ = std::make_shared<IndexedPcapReader>(pcap_path_, sensor_infos_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to open IndexedPcapReader: %s", e.what());
            throw;
        }
        if (!index_pcap_reader_) {
            RCLCPP_ERROR(get_logger(), "IndexedPcapReader is null");
            throw std::runtime_error("IndexedPcapReader is null");
        }

        //  ScanBatcher, XYZ LUT, LidarScan initialization
        // ============================================================
        try {
            info_    = std::make_shared<sensor_info>(sensor_infos_[0]);
            batcher_ = std::make_unique<ScanBatcher>(*info_);
            lut_     = make_xyz_lut(*info_, true);
            scan_    = LidarScan(*info_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize Ouster SDK objects: %s", e.what());
            throw;
        }


        // ROS2 publsiher
        // ============================================================
        imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("/os1/imu", 10);
        lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                        "/pointcloud", rclcpp::QoS(10).reliable());

 

        RCLCPP_INFO(get_logger(), "Building index for PCAP...");
        index_pcap_reader_->build_index();
        const PcapIndex &idx = index_pcap_reader_->get_index();
        RCLCPP_INFO(get_logger(),
                    "Index built: total frames for sensor[0] = %zu",
                    idx.frame_count(0));

        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { process_packet(); }
        );

        RCLCPP_INFO(get_logger(),
                    "TotalReaderNode initialized. JSON(s): %zu files, PCAP: %s",
                    json_paths_.size(), pcap_path_.c_str());
    }

private:

    std::optional<int64_t> prev_ns_;
    void process_packet() {

        if (!index_pcap_reader_ || !batcher_ || !info_) {
            RCLCPP_ERROR(get_logger(), "Null pointer detected in process_packet");
            return;
        }

        //next_packet() call: 
        // ----------------------------------------
        size_t payload_size = index_pcap_reader_->next_packet();
        if (payload_size == 0) {
            // → 파일 끝에 도달하거나 오류. 다시 처음으로 돌아가고 scan_ 재생성
            index_pcap_reader_->reset();
            scan_ = LidarScan(*info_);
            return;
        }



        // current_info = this (packet_info)
        const packet_info &pinfo = index_pcap_reader_->current_info();
        const int64_t curr_ns = pinfo.timestamp.count();

         if (curr_ns < static_cast<int64_t>(skip_time_ * 1e9)) {
            return;
        }
        
        if (prev_ns_) {
            int64_t dt_ns = curr_ns - prev_ns_.value();
            if (dt_ns > 0 && dt_ns < 1e9) { // if its highert han 1 sec, skip
                std::this_thread::sleep_for(std::chrono::nanoseconds(dt_ns));
            }
        }
        prev_ns_ = curr_ns;

        //    - IMU packet: dst_port == sensor_infos_[0].config.udp_port_imu(ex: 7503)
        //    - Lidar packet: dst_port == sensor_infos_[0].config.udp_port_lidar(ex: 7502)

        // IMU pakcet
        if (pinfo.dst_port == static_cast<int>(*sensor_infos_[0].config.udp_port_imu)) {
            const uint8_t *data = index_pcap_reader_->current_data();
            auto get_f = [&](const uint8_t *p) {
                float v;
                std::memcpy(&v, p, sizeof(v));
                return v;
            };

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(pinfo.timestamp.count()));

            imu_msg.header.frame_id = "os1_imu";

            // Ouster OS1 IMU payload offset in byte 24~44
            imu_msg.linear_acceleration.x = get_f(data + 24);
            imu_msg.linear_acceleration.y = get_f(data + 28);
            imu_msg.linear_acceleration.z = get_f(data + 32);
            imu_msg.angular_velocity.x    = get_f(data + 36);
            imu_msg.angular_velocity.y    = get_f(data + 40);
            imu_msg.angular_velocity.z    = get_f(data + 44);

            imu_pub_->publish(imu_msg);
            return;
        }

        // 5-2) LiDAR packet processing
        if (pinfo.dst_port == static_cast<int>(*sensor_infos_[0].config.udp_port_lidar)) {

            const uint8_t *data = index_pcap_reader_->current_data();
            LidarPacket pkt_lidar(static_cast<int>(payload_size));
            std::memcpy(pkt_lidar.buf.data(), data, payload_size);

            // after one frame = true
            bool frame_ready = (*batcher_)(pkt_lidar, scan_);
            if (!frame_ready) {
                return; 
            }

            // after one frame: scan_.field<>()
            auto ranges   = scan_.field<uint32_t>(ChanField::RANGE);
            auto xyz_mat  = cartesian(ranges, lut_);  // Eigen::Array<double, N, 3>
            auto refl_arr = scan_.field<uint8_t>(ChanField::REFLECTIVITY);
            Eigen::Map<const Eigen::Array<uint8_t, Eigen::Dynamic, 1>> refl(
                refl_arr.data(), static_cast<int>(refl_arr.size())
            );

            //PCL PointCloud (reflectivity → intensity)
            pcl::PointCloud<pcl::PointXYZI> cloud;
            cloud.reserve(xyz_mat.rows());
            for (int i = 0; i < xyz_mat.rows(); ++i) {
                const auto &pt = xyz_mat.row(i);
                if (std::isfinite(pt(0)) &&
                    std::isfinite(pt(1)) &&
                    std::isfinite(pt(2))) {
                    pcl::PointXYZI p;
                    p.x         = static_cast<float>(pt(0));
                    p.y         = static_cast<float>(pt(1));
                    p.z         = static_cast<float>(pt(2));
                    p.intensity = static_cast<float>(refl(i));
                    cloud.push_back(p);
                }
            }
            cloud.width    = static_cast<uint32_t>(cloud.size());
            cloud.height   = 1;
            cloud.is_dense = false;


            // 10) PointCloud2 publsihing
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(cloud, pc_msg);
            
            pc_msg.header.frame_id = "os1";
            

            //uint64_t nanosec = static_cast<uint64_t>(pinfo.timestamp.count());
            pc_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(pinfo.timestamp.count()));
            
            lidar_pub_->publish(pc_msg);

            // 11) for next frame
            scan_ = LidarScan(*info_);
        }
    }


    double                                             skip_time_;
    std::vector<std::string>                           json_paths_;
    std::string                                        pcap_path_;
    std::vector<ouster::sensor::sensor_info>           sensor_infos_;
    std::shared_ptr<IndexedPcapReader>                 index_pcap_reader_;
    std::shared_ptr<sensor_info>                       info_;            
    std::unique_ptr<ScanBatcher>                       batcher_;
    LidarScan                                          scan_;
    XYZLut                                             lut_;             
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::TimerBase::SharedPtr                       timer_;

    // TF2
    tf2_ros::Buffer       tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TotalReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
