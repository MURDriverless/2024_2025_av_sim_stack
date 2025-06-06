#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

using namespace std;
using namespace std::chrono_literals;

struct PointXYZIR {
    float x, y, z;
    float signal;
    float reflectivity;
};

class CsvToPointCloudNode : public rclcpp::Node {
public:
    CsvToPointCloudNode() : Node("csv_to_pointcloud_node"), current_index_(0), paused_(false) {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        load_csv();

        if (!points_.empty()) {
            timer_ = this->create_wall_timer(100ms, std::bind(&CsvToPointCloudNode::publish_frame, this));
        } else {
            RCLCPP_WARN(this->get_logger(), "No points loaded from CSV.");
        }
    }

private:
    vector<PointXYZIR> points_;
    size_t current_index_;
    bool paused_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void load_csv() {
        string pkg_path = ament_index_cpp::get_package_share_directory("csv_to_pointcloud_node");
        string csv_path = pkg_path + "/data/straight_origin.csv";

        ifstream file(csv_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
            return;
        }

        string line;
        getline(file, line);  // skip header

        while (getline(file, line)) {
            stringstream ss(line);
            string token;
            vector<string> tokens;

            while (getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() < 7) continue;

            try {
                float x = stof(tokens[4]) / 1000.0f;
                float y = stof(tokens[5]) / 1000.0f;
                float z = stof(tokens[6]) / 1000.0f;
                float signal = stof(tokens[2]);
                float reflectivity = stof(tokens[3]);

                points_.push_back({x, y, z, signal, reflectivity});
            } catch (...) {
                continue;
            }
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu points from CSV", points_.size());
    }

    void publish_frame() {
        if (kbhit()) {
            char c = getchar();
            if (c == 's') paused_ = !paused_;
        }

        if (paused_) return;

        const size_t batch_size = 1024 * 128;
        if (current_index_ >= points_.size()) {
            current_index_ = 0;
            RCLCPP_INFO(this->get_logger(), "Looping back to start of point cloud.");
            return;
        }

        // 1) batch_size 
        size_t end_index = min(current_index_ + batch_size, points_.size());
        vector<PointXYZIR> raw_frame(points_.begin() + current_index_, points_.begin() + end_index);
        current_index_ = end_index;

        // 2) x, y, z = 0 then skip literally means range is 0
        vector<PointXYZIR> frame;
        frame.reserve(raw_frame.size());
        for (const auto& pt : raw_frame) {
            if (pt.x == 0.0f || pt.y == 0.0f || pt.z == 0.0f) {
                continue;
            }
            frame.push_back(pt);
        }

        
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.frame_id = "lidar_frame";
        msg.header.stamp = this->get_clock()->now();
        msg.height = 1;
        msg.width = frame.size();
        msg.is_dense = true;
        msg.is_bigendian = false;

        msg.fields.resize(4);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[0].count = 1;

        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[1].count = 1;

        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[2].count = 1;

        msg.fields[3].name = "intensity";
        msg.fields[3].offset = 12;
        msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[3].count = 1;

        msg.point_step = 16;
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step);

        // 4) data buffer
        float* data_ptr = reinterpret_cast<float*>(msg.data.data());
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto& pt : frame) {
            *(data_ptr++) = pt.x;
            *(data_ptr++) = pt.y;
            *(data_ptr++) = pt.z;
            *(data_ptr++) = pt.signal;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "lidar_frame";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "points";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pt.x;
            marker.pose.position.y = pt.y;
            marker.pose.position.z = pt.z;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.g = 1.0;
            marker_array.markers.push_back(marker);
        }

        publisher_->publish(msg);
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %u valid points", msg.width);
    }

    bool kbhit() {
        termios term;
        tcgetattr(STDIN_FILENO, &term);

        termios term2 = term;
        term2.c_lflag &= ~ICANON;
        tcsetattr(STDIN_FILENO, TCSANOW, &term2);

        int bytesWaiting;
        ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);

        tcsetattr(STDIN_FILENO, TCSANOW, &term);
        return bytesWaiting > 0;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CsvToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
