#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PointCloudToMarkers : public rclcpp::Node {
public:
    PointCloudToMarkers() : Node("pointcloud_to_markers") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10,
            std::bind(&PointCloudToMarkers::pointcloud_callback, this, std::placeholders::_1)
        );
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray marker_array;

        int id = 0;
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                                                        iter_y(*msg, "y"),
                                                        iter_z(*msg, "z"),
                                                        iter_signal(*msg, "intensity"),         // CSV signal -> intensity (signals)
                                                        iter_reflectivity(*msg, "reflectivity");
             iter_x != iter_x.end(); 
             ++iter_x, ++iter_y, ++iter_z, ++iter_signal, ++iter_reflectivity)
        {
            float signal = *iter_signal;
            float reflectivity = *iter_reflectivity;

            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "points";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = *iter_x;
            marker.pose.position.y = *iter_y;
            marker.pose.position.z = *iter_z;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;

            // apply reflectivity
            marker.color.r = std::min(reflectivity / 100.0f, 1.0f);  // Reflectivity → Red
            marker.color.g = std::min(signal / 1000.0f, 1.0f);       // Signal → Green
            marker.color.b = 0.2f;

            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToMarkers>());
    rclcpp::shutdown();
    return 0;
}
