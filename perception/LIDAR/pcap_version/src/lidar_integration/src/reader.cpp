#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Ouster SDK
#include <ouster/os_pcap.h>
#include <ouster/types.h>
#include <ouster/lidar_scan.h>
#include <ouster/client.h>
#include <ouster/version.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using namespace ouster;
using namespace ouster::sensor;
using namespace ouster::sensor_utils;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_reader");

    const auto pkg_share = ament_index_cpp::get_package_share_directory("lidar_integration");
    std::string json_path = node->declare_parameter<std::string>(
        "json_path", pkg_share + "/data/lidar_metadata.json");
    std::string pcap_path = node->declare_parameter<std::string>(
        "pcap_path", pkg_share + "/data/lidar_data.pcap");
    double skip_seconds = node->declare_parameter<double>("skip_time");
    int skip_frames = static_cast<int>(skip_seconds * 10);  // 10Hz

    RCLCPP_INFO(node->get_logger(), "Loading metadata from %s", json_path.c_str());

    sensor_info info = metadata_from_json(json_path);;
    try {
        RCLCPP_INFO(node->get_logger(), "✅ metadata_from_json() 성공");

        RCLCPP_INFO(node->get_logger(), "🟢 Parsing data_format...");
        const auto& format = info.format;

        RCLCPP_INFO(node->get_logger(), "Parsed format: width = %u, height = %u",
                    format.columns_per_frame, format.pixels_per_column);

        std::string udp_profile_str = to_string(info.format.udp_profile_lidar);
        RCLCPP_INFO(node->get_logger(), "UDP Profile: %s", udp_profile_str.c_str());
                

        RCLCPP_INFO(node->get_logger(), "✅ Format section completed");

        // sensor log
        if (!info.build_date.empty())
            RCLCPP_INFO(node->get_logger(), "build_date: %s", info.build_date.c_str());
        if (!info.prod_pn.empty())
            RCLCPP_INFO(node->get_logger(), "prod_pn: %s", info.prod_pn.c_str());
        if (!info.prod_line.empty())
            RCLCPP_INFO(node->get_logger(), "prod_line: %s", info.prod_line.c_str());
        if (!info.fw_rev.empty())
            RCLCPP_INFO(node->get_logger(), "build_rev: %s", info.fw_rev.c_str());
        if (!info.status.empty())
            RCLCPP_INFO(node->get_logger(), "status: %s", info.status.c_str());
        if (info.sn != 0)
            RCLCPP_INFO(node->get_logger(), "prod_sn: %lu", static_cast<uint64_t>(info.sn));
        if (!info.image_rev.empty())
            RCLCPP_INFO(node->get_logger(), "image_rev: %s", info.image_rev.c_str());

      } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "Failed to read metadata: %s", e.what());
          return 1;
      }

    RCLCPP_INFO(node->get_logger(), "Opening PCAP: %s", pcap_path.c_str());
    auto handle = replay_initialize(pcap_path);
    if (!handle) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open PCAP");
        return 1;
    }

    const size_t w = info.format.columns_per_frame;
    const size_t h = info.format.pixels_per_column;
    ScanBatcher batcher(info);
    LidarScan scan(w, h, info.format.udp_profile_lidar);
    auto lut = make_xyz_lut(info, true);

    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::QoS(10).reliable());
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    packet_info pkt_info;
    std::vector<uint8_t> buf;
    int frame_count = 0;

    while (rclcpp::ok()) {
        if (!next_packet_info(*handle, pkt_info)) {
            RCLCPP_WARN(node->get_logger(), "End of PCAP. Rewinding...");
            handle = replay_initialize(pcap_path);
            scan = LidarScan(w, h, info.format.udp_profile_lidar);
            frame_count = 0;
            continue;
        }

        buf.resize(pkt_info.packet_size);
        read_packet(*handle, buf.data(), pkt_info.packet_size);

        /*
        // Print measurement_id for each column
        const int col_per_pkt = info.format.columns_per_packet;
        const int h = info.format.pixels_per_column;
        const int col_stride = h * 12 + 24;  // 12 bytes per pixel, 24 bytes column header  
        for (int col = 0; col < col_per_pkt; ++col) {
            const uint8_t* col_ptr = buf.data() + col * col_stride;
            uint16_t meas_id = col_ptr[0] | (col_ptr[1] << 8);
            RCLCPP_INFO(node->get_logger(), "[Packet] Column %d Measurement ID: %u", col, meas_id);
        }
            */

        LidarPacket pkt(static_cast<int>(pkt_info.packet_size));
        std::copy(buf.begin(), buf.end(), pkt.buf.begin());   
        if (batcher(pkt, scan)) {
            frame_count++;
            //auto range = scan.field(ChanField::RANGE);
////////////////////////////////////////////////////////////////////

        Eigen::ArrayXX<uint32_t> range = scan.field<uint32_t>(ChanField::RANGE);
        RCLCPP_INFO(node->get_logger(), "Range[0,0]: %u", range(0, 0));


            RCLCPP_INFO(node->get_logger(),
            "LUT direction[0]: %.2f %.2f %.2f",
            lut.direction(0, 0), lut.direction(0, 1), lut.direction(0, 2));

            RCLCPP_INFO(node->get_logger(),
            "LUT offset[0]: %.2f %.2f %.2f",
            lut.offset(0, 0), lut.offset(0, 1), lut.offset(0, 2));
/////////////////////////////////////////////////////////////////
            
            if (frame_count < skip_frames) {
                scan = LidarScan(w, h, info.format.udp_profile_lidar);
                continue;
            }

            auto cloud_xyz = cartesian(scan.field(ChanField::RANGE), lut);
            for (int i = 0; i < 10 && i < cloud_xyz.rows(); ++i) {
                const auto& pt = cloud_xyz.row(i);
                RCLCPP_INFO(node->get_logger(), "XYZ[%d]: %.2f %.2f %.2f", i, pt(0), pt(1), pt(2));
            }

            pcl::PointCloud<pcl::PointXYZI> cloud;
            cloud.width = static_cast<uint32_t>(cloud_xyz.rows());
            cloud.height = 1;
            cloud.is_dense = false;
            cloud.points.reserve(cloud.width);

            Eigen::ArrayXXf reflectivity;
            if (scan.fields().count(ChanField::REFLECTIVITY)) {
                auto dtype = scan.field_type(ChanField::REFLECTIVITY).element_type;
                if (dtype == ChanFieldType::UINT8) {
                    reflectivity = scan.field<uint8_t>(ChanField::REFLECTIVITY).cast<float>();
                } else if (dtype == ChanFieldType::UINT16) {
                    reflectivity = scan.field<uint16_t>(ChanField::REFLECTIVITY).cast<float>();
                } else {
                    RCLCPP_WARN(node->get_logger(), "Unsupported REFLECTIVITY type");
                    reflectivity = Eigen::ArrayXXf::Zero(cloud_xyz.rows(), 1);
                }
            }

            for (int i = 0; i < cloud_xyz.rows(); ++i) {
                const auto& pt = cloud_xyz.row(i);
                if (std::isfinite(pt(0)) && std::isfinite(pt(1)) && std::isfinite(pt(2))) {
                    pcl::PointXYZI p;
                    p.x = pt(0);
                    p.y = pt(1);
                    p.z = pt(2);
                    p.intensity = (i < reflectivity.rows()) ? reflectivity(i, 0) : 0.0f;
                    cloud.points.push_back(p);
                }
            }

            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(cloud, msg);
            msg.header.stamp = node->now();
            msg.header.frame_id = "os1";

            try {
                auto transform = tf_buffer.lookupTransform("map", "os1", tf2::TimePointZero);
                tf2::doTransform(msg, msg, transform);
                msg.header.frame_id = "map";
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(node->get_logger(), "Transform failed: %s", ex.what());
                continue;
            }

            pub->publish(msg);
            RCLCPP_INFO(node->get_logger(), "Published point cloud: %zu points", cloud.size());

            scan = LidarScan(w, h, info.format.udp_profile_lidar);
            //rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    replay_uninitialize(*handle);
    rclcpp::shutdown();
    return 0;
}