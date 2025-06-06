#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <limits>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>

struct PointXYZR {
    float x, y, z;
    float reflectivity;
};

class ConeDetection : public rclcpp::Node {
public:
    ConeDetection() : Node("cone_detection") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cropped_pointcloud", 10,
            std::bind(&ConeDetection::pointCloudCallback, this, std::placeholders::_1)
        );

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cone_markers", 10);
        filtered_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "[STATE] Received point cloud with %u points", msg->width * msg->height);

        // 1. Filter by reflectivity and height
        std::vector<PointXYZR> intensity_filtered;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_ref(*msg, "intensity");
        
        while (iter_x != iter_x.end()) {
            if (*iter_ref > 15.0f && *iter_z > -1.59f) {
                intensity_filtered.push_back({*iter_x, *iter_y, *iter_z, *iter_ref});
            }
            ++iter_x; ++iter_y; ++iter_z; ++iter_ref;
        }
        RCLCPP_INFO(get_logger(), "[STATE] After intensity and height filtering: %u points", intensity_filtered.size());

        // 2. Convert to PCL XYZ cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (auto &pt : intensity_filtered) {
            xyz_cloud->points.emplace_back(pt.x, pt.y, pt.z);
        }

        // 3. Apply VoxelGrid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(xyz_cloud);
        voxel.setLeafSize(0.03f, 0.03f, 0.03f);
        voxel.filter(*voxel_cloud);
        RCLCPP_INFO(get_logger(), "[STATE] After voxel grid filter: %u points", voxel_cloud->size());

        // Publish filtered cloud
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*voxel_cloud, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_publisher_->publish(filtered_msg);
        RCLCPP_INFO(get_logger(), "[STATE] Published filtered point cloud");

        // 4. Prepare for clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(voxel_cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1f);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(voxel_cloud);
        std::vector<pcl::PointIndices> clusters;
        ec.extract(clusters);
        RCLCPP_INFO(get_logger(), "[STATE] Number of clusters found: %u", clusters.size());

        // 5. Analyze clusters and apply cylinder and size constraints
        visualization_msgs::msg::MarkerArray marker_array;
        std::vector<std::pair<float,float>> cone_centers;
        int id = 0;

        for (auto &indices : clusters) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (auto idx : indices.indices) {
                cluster_cloud->points.push_back(voxel_cloud->points[idx]);
            }

            // Compute centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster_cloud, centroid);
            float cx = centroid[0], cy = centroid[1], cz = centroid[2];

            // Cylinder condition - optional
            Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
            Eigen::Vector3f b(-2*cx, -2*cy, 0.0f);
            float radius = 0.5f;
            float d = cx*cx + cy*cy - radius*radius;
            pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr comp(
                new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>(pcl::ComparisonOps::LE, A, b, d)
            );
            pcl::ConditionAnd<pcl::PointXYZ>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZ>());
            cond->addComparison(comp);
            pcl::ConditionalRemoval<pcl::PointXYZ> remover(true);
            remover.setCondition(cond);
            remover.setInputCloud(cluster_cloud);
            pcl::PointCloud<pcl::PointXYZ> cyl_filtered;
            remover.filter(cyl_filtered);

            // Compute size
            float min_x=std::numeric_limits<float>::max(), max_x=-min_x;
            float min_y=min_x, max_y=-min_x;
            float min_z=min_x, max_z=-min_x;
            for (auto &p : cyl_filtered.points) {
                min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
                min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
            }
            float width = max_x - min_x;
            float depth = max_y - min_y;
            float height = max_z - min_z;
           // RCLCPP_INFO(get_logger(), "[STATE] Cluster %d dimensions (WxDxH): %.3f x %.3f x %.3f", id, width, depth, height);

            // Size constraints
            if (height < 0.1f || height > 0.45f) continue;
            if (width < 0.1f || width > 0.25f) continue;
            if (depth < 0.1f || depth > 0.35f) continue;

            // Record cone center
            cone_centers.emplace_back(cx, cy);

            // Create marker
            visualization_msgs::msg::Marker m;
            m.header = msg->header;
            m.ns = "cone_clusters";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = cx;
            m.pose.position.y = cy;
            m.pose.position.z = cz;
            m.scale.x = 0.2f; m.scale.y = 0.2f; m.scale.z = 0.2f;
            m.color.a = 1.0f;
            if (cy > 0) { m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 1.0f; }
            else       { m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; }
            m.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker_array.markers.push_back(m);
        }

        marker_publisher_->publish(marker_array);
        RCLCPP_INFO(get_logger(), "[STATE] Published %u markers", marker_array.markers.size());

        // Save cone positions to CSV
        std::string path = ament_index_cpp::get_package_share_directory("csv_to_pointcloud_node") + "/data/cone_paths.csv";
        std::ofstream ofs(path);
        ofs << "x,y,label\n";
        for (auto &c : cone_centers) {
            ofs << c.first << "," << c.second << "," << (c.second>0?"right":"left") << "\n";
        }
        ofs.close();
        RCLCPP_INFO(get_logger(), "[STATE] Saved %u cone positions to CSV", cone_centers.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetection>());
    rclcpp::shutdown();
    return 0;
}