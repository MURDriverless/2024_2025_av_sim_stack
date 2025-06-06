#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker_array.hpp>

class CropBoxFilterNode : public rclcpp::Node {
public:
    CropBoxFilterNode() : Node("crop_box_filter_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10,
            std::bind(&CropBoxFilterNode::pointcloud_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cropped_pointcloud", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cropbox_markers", 10);

        RCLCPP_INFO(this->get_logger(), "CropBox Filter Node has been started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(4, -6.0, -1.54, 1.0)); //change z to -1.6 for testing
        crop.setMax(Eigen::Vector4f(12.0, 6.0, 0.0, 1.0));

        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);
        crop.filter(*cropped);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cropped, output);
        output.header = msg->header;
        pub_->publish(output);

        // Optional marker visualization
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto& pt : cropped->points) {
            visualization_msgs::msg::Marker marker;
            marker.header = output.header;
            marker.ns = "cropbox";
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
            marker.color.r = 0.8;
            marker.color.g = 0.5;
            marker.color.b = 0.2;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu cropped points", cropped->size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CropBoxFilterNode>());
    rclcpp::shutdown();
    return 0;
}
