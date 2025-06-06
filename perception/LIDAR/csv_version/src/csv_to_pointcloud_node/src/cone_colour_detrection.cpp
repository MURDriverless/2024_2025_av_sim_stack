#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>

struct PointXYZR {
    float x, y, z;
    float reflectivity;
};

struct ClusterData {
    Eigen::Vector4f centroid;
    float distance;
    std::vector<float> reflectivities;
};

class ConeDetection : public rclcpp::Node {
public:
    ConeDetection() : Node("cone_detection") {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cropped_pointcloud", 10,
            std::bind(&ConeDetection::callback, this, std::placeholders::_1)
        );
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("cone_markers", 10);
        rclcpp::on_shutdown([this]() { saveReflectivityPatterns(); });
    }

    ~ConeDetection() {
        // Ensure save even if abrupt
        saveReflectivityPatterns();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::array<ClusterData,2> selected_cones_;

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. Read points and reflectivities
        std::vector<PointXYZR> points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        sensor_msgs::PointCloud2ConstIterator<float> itx(*msg,"x"), ity(*msg,"y"), itz(*msg,"z"), itr(*msg,"intensity");
        for (; itx!=itx.end(); ++itx, ++ity, ++itz, ++itr) {
            points.push_back({*itx, *ity, *itz, *itr});
            cloud->points.emplace_back(*itx,*ity,*itz);
        }

        // 2. VoxelGrid downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.03f,0.03f,0.03f);
        vg.filter(*voxel);

        // 3. Clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(voxel);
        std::vector<pcl::PointIndices> clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1f);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(voxel);
        ec.extract(clusters);
        if (clusters.size() < 2) {
            RCLCPP_WARN(get_logger(), "Not enough clusters detected");
            return;
        }

        // 4. Compute cluster data
        std::vector<ClusterData> list;
        for (auto &ci : clusters) {
            ClusterData cd;
            pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>());
            for (int idx : ci.indices) c->points.push_back(voxel->points[idx]);
            pcl::compute3DCentroid(*c, cd.centroid);
            cd.distance = std::hypot(cd.centroid[0], cd.centroid[1]);
            // Collect reflectivities for points in cluster
            for (auto &p : points) {
                float dx = p.x - cd.centroid[0];
                float dy = p.y - cd.centroid[1];
                if (std::hypot(dx,dy) < 0.3f) cd.reflectivities.push_back(p.reflectivity);
            }
            list.push_back(cd);
        }

        // 5. Select two closest clusters
        std::sort(list.begin(), list.end(), [](auto &a, auto &b){ return a.distance < b.distance; });
        selected_cones_[0] = list[0];
        selected_cones_[1] = list[1];

        // 6. Publish marker spheres colored for visibility only
        visualization_msgs::msg::MarkerArray ma;
        for (int i = 0; i < 2; ++i) {
            auto &cd = selected_cones_[i];
            visualization_msgs::msg::Marker m;
            m.header = msg->header;
            m.ns = "cones";
            m.id = i;
            m.type = m.SPHERE;
            m.action = m.ADD;
            m.pose.position.x = cd.centroid[0];
            m.pose.position.y = cd.centroid[1];
            m.pose.position.z = cd.centroid[2];
            m.scale.x = 0.2f; m.scale.y = 0.2f; m.scale.z = 0.2f;
            m.color.a = 1.0f;
            // Visual differentiate
            if (i == 0) { m.color.r = 1.0f; } else { m.color.b = 1.0f; }
            ma.markers.push_back(m);
        }
        marker_pub_->publish(ma);
    }

    void saveReflectivityPatterns() {
        // Save to local folder
        std::string path = "./cone_reflectivity.csv";
        std::ofstream ofs(path);
        ofs << "cone,reflectivity,color\n";
        // Compute global mean and sigma
        std::vector<float> all;
        for (auto &cd : selected_cones_) all.insert(all.end(), cd.reflectivities.begin(), cd.reflectivities.end());
        if (all.empty()) return;
        float mean = std::accumulate(all.begin(), all.end(), 0.0f) / all.size();
        float var = 0;
        for (auto v : all) var += (v - mean)*(v - mean);
        float sigma = std::sqrt(var / all.size());
        float low_th = mean - sigma;
        float high_th = mean + sigma;

        for (int i = 0; i < 2; ++i) {
            auto &r = selected_cones_[i].reflectivities;
            for (auto v : r) {
                std::string color;
                if (v < low_th)        color = (i==0?"Yellow":"Blue");
                else if (v > high_th)  color = (i==0?"Yellow":"Blue");
                else                   color = (i==0?"Black":"White");
                ofs << i << "," << v << "," << color << "\n";
            }
        }
        ofs.close();
        RCLCPP_INFO(get_logger(), "Saved reflectivity patterns to %s", path.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetection>());
    rclcpp::shutdown();
    return 0;
}
