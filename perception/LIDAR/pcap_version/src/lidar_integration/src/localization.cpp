// src/localization.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // tf2::toMsg

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <cmath>
#include <memory>
#include <string>

using std::placeholders::_1;

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode()
    : Node("localization_node"),
      cumulative_distance_(0.0)
    {
        // 1) IMU 구독
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/os1/imu", 100,
            std::bind(&LocalizationNode::imu_callback, this, _1));

        // 2) TF 브로드캐스터 생성
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 3) 거리 기록을 위한 파일 열기
        auto pkg = ament_index_cpp::get_package_share_directory("lidar_integration");
        std::string file_path = pkg + "/data/imu_distance.txt";
        distance_file_.open(file_path, std::ios::out | std::ios::trunc);
        if (!distance_file_.is_open()) {
            RCLCPP_WARN(get_logger(), "Could not open distance log file: %s", file_path.c_str());
        } else {
            // 헤더 한 줄 쓰기
            distance_file_ << "# stamp_sec stamp_nsec cumulative_distance_m\n";
        }

        RCLCPP_INFO(get_logger(), "LocalizationNode initialized. Distance log: %s", file_path.c_str());
    }

    ~LocalizationNode() {
        if (distance_file_.is_open()) {
            distance_file_.close();
        }
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        // ROS 시간 기준으로 stamp
        rclcpp::Time msg_stamp(imu_msg->header.stamp);

        static bool first = true;
        static rclcpp::Time last_stamp;
        static double velocity_x = 0.0, velocity_y = 0.0;
        static double position_x = 0.0, position_y = 0.0;
        static double yaw        = 0.0;
        static double prev_pos_x = 0.0, prev_pos_y = 0.0;

        if (first) {
            // 첫 번째 콜백 시에는 이전 타임스탬프만 저장하고 TF 브로드캐스트/거리 계산은 하지 않음
            last_stamp = msg_stamp;
            prev_pos_x = position_x;
            prev_pos_y = position_y;
            first = false;
            return;
        }

        // dt 계산 (초 단위, msg_stamp 기준)
        double dt = (msg_stamp - last_stamp).seconds();
        last_stamp = msg_stamp;

        // ====================================
        // 1) IMU 축 정의에 따라 가속도/각속도 적분
        //    (예제에서는: x=forward, y=leftward, z=upward 으로 가정)
        //    실제 OS1 IMU 매뉴얼을 보고 필요시 축 매핑을 수정해야 합니다.
        // ====================================
        double ax = imu_msg->linear_acceleration.x;  
        double ay = imu_msg->linear_acceleration.y;  
        double wz = imu_msg->angular_velocity.z;    

        yaw += wz * dt;                 // Yaw 회전 적분
        velocity_x += ax * dt;          // x 축 속도 적분
        velocity_y += ay * dt;          // y 축 속도 적분

        position_x += velocity_x * dt;  // x 축 위치 적분
        position_y += velocity_y * dt;  // y 축 위치 적분

        // ====================================
        // 2) TF 메시지 생성
        // ====================================
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = imu_msg->header.stamp;
        transform.header.frame_id = "map";
        transform.child_frame_id  = "os1";

        transform.transform.translation.x = position_x;
        transform.transform.translation.y = position_y;
        transform.transform.translation.z = 0.0;

        // Z축 회전만 고려 (평면 2D 가정)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        transform.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transform);

        // ====================================
        // 3) 누적 이동 거리 계산 & 로그에 쓰기
        //    - 단순히 "바로 전 위치(prev_pos) → 현재 위치(pos)" 간 직선 거리 차를 구해서 누적
        // ====================================
        double dx = position_x - prev_pos_x;
        double dy = position_y - prev_pos_y;
        double delta_dist = std::sqrt(dx*dx + dy*dy);
        cumulative_distance_ += delta_dist;

        // 다음 프레임에는 현재 위치를 "이전 위치"로
        prev_pos_x = position_x;
        prev_pos_y = position_y;

        // ====================================
        // 4) 로그 파일에 한 줄 쓰기: [ros_sec ros_nsec cumulative_distance]
        // ====================================
        if (distance_file_.is_open()) {
            distance_file_
                << msg_stamp.seconds() << " "
                << msg_stamp.nanoseconds() << " "
                << cumulative_distance_ << "\n";
            distance_file_.flush();
        }
    }

    // ----------------------------
    // 멤버 변수
    // ----------------------------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::ofstream distance_file_;

    double cumulative_distance_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
