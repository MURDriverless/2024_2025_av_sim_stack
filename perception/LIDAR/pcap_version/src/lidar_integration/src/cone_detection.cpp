// src/cone_detection.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <limits>
#include <vector>
#include <tuple>
#include <algorithm>
#include <numeric>
#include <pcl/common/centroid.h>

struct PointXYZR {
    float x, y, z;
    float reflectivity;
};

class ConeDetection : public rclcpp::Node {
public:
    ConeDetection()
        : Node("cone_detection_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cropped_pointcloud", 10,
            std::bind(&ConeDetection::pointcloud_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "cone_markers", 10);
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "filtered_pointcloud", 10);

        RCLCPP_INFO(this->get_logger(), "ConeDetection Node has been started.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 반드시 "os1" 프레임만 처리!
        if (msg->header.frame_id != "map") {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 frame_id must be 'os1', but got: %s", msg->header.frame_id.c_str());
            return;
        }
        const sensor_msgs::msg::PointCloud2& input_msg = *msg;

        // 2) Reflectivity 기준으로 필터링: reflectivity > 25
        std::vector<PointXYZR> filtered_points;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(input_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(input_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(input_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_reflectivity(input_msg, "intensity");

        while (iter_x != iter_x.end()) {
            float refl = *iter_reflectivity;
            if (refl > 25.0f) {
                filtered_points.push_back({*iter_x, *iter_y, *iter_z, refl});
            }
            ++iter_x; ++iter_y; ++iter_z; ++iter_reflectivity;
        }

        // 3) PointXYZI 클라우드 생성
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->points.reserve(filtered_points.size());
        for (const auto &pt : filtered_points) {
            pcl::PointXYZI p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.reflectivity;
            cloud->points.push_back(p);
        }

        // 4) VoxelGrid 다운샘플링
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        vg.filter(*filtered_cloud);

        // 퍼블리시: 필터링된 클라우드 (frame_id = "os1" 유지!)
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_msg.header.frame_id = "os1";
        filtered_pub_->publish(filtered_msg);

        // 5) Euclidean 클러스터링
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(filtered_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.1);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered_cloud);
        ec.extract(cluster_indices);

        // 6) 결과 저장용 벡터
        std::vector<std::tuple<float, float, std::string>> centre_info;      // x, y, left/right
        std::vector<std::tuple<float, std::string>> intensity_info;          // avg_refl, color_label

        visualization_msgs::msg::MarkerArray marker_array;
        int cluster_id = 0;

        for (const auto &indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            cloud_cluster->points.reserve(indices.indices.size());
            std::vector<std::pair<float,float>> zr_pairs; // (z, reflectivity)

            // (1) 클러스터 포인트 추출 및 (z, reflectivity) 저장
            for (int idx : indices.indices) {
                const auto &pt = filtered_cloud->points[idx];
                cloud_cluster->points.push_back(pt);
                zr_pairs.emplace_back(pt.z, pt.intensity);
            }
            cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = false;

            // (2) (z, reflectivity) 정보를 z 오름차순으로 정렬
            std::sort(zr_pairs.begin(), zr_pairs.end(),
                      [](auto &a, auto &b) { return a.first < b.first; });

            // (3) 정렬된 벡터에서 반사도만 추출
            std::vector<float> refl_sorted;
            refl_sorted.reserve(zr_pairs.size());
            for (auto &p : zr_pairs) {
                refl_sorted.push_back(p.second);
            }

            // (4) "밝기-어둠-밝기" vs "어둠-밝기-어둠" 패턴 탐지
            auto min_it = std::min_element(refl_sorted.begin(), refl_sorted.end());
            size_t min_idx = std::distance(refl_sorted.begin(), min_it);
            float min_refl = *min_it;
            size_t window = std::max<size_t>(1, refl_sorted.size() / 10);
            size_t start_before = (min_idx >= window) ? (min_idx - window) : 0;
            size_t end_after = std::min(min_idx + window, refl_sorted.size() - 1);

            float sum_before = 0.0f; size_t count_before = 0;
            for (size_t i = start_before; i < min_idx; ++i) {
                sum_before += refl_sorted[i]; ++count_before;
            }
            float avg_before = (count_before > 0 ? sum_before / count_before : refl_sorted.front());

            float sum_after = 0.0f; size_t count_after = 0;
            for (size_t i = min_idx + 1; i <= end_after; ++i) {
                sum_after += refl_sorted[i]; ++count_after;
            }
            float avg_after = (count_after > 0 ? sum_after / count_after : refl_sorted.back());

            std::string color_label;
            if (avg_before > min_refl && avg_after > min_refl) {
                color_label = "yellow";
            } else if (avg_before < min_refl && avg_after < min_refl) {
                color_label = "blue";
            } else {
                color_label = "unknown";
            }

            // (5) 클러스터 중심 계산
            pcl::PointXYZI centroid_i;
            pcl::computeCentroid(*cloud_cluster, centroid_i);
            pcl::PointXYZ centre;
            centre.x = centroid_i.x;
            centre.y = centroid_i.y;
            centre.z = centroid_i.z;

            std::string lr_label = (centre.x < 0.0f) ? "left" : "right";
            centre_info.emplace_back(centre.x, centre.y, lr_label + "_" + color_label);

            float sum_all = std::accumulate(refl_sorted.begin(), refl_sorted.end(), 0.0f);
            float avg_all = sum_all / static_cast<float>(refl_sorted.size());
            intensity_info.emplace_back(avg_all, color_label);

            // (8) 마커 생성 (회색조)
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header; // 반드시 os1 frame!
            marker.ns = "cone_clusters";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = centre.x;
            marker.pose.position.y = centre.y;
            marker.pose.position.z = centre.z;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0f;

            float normalized = std::min(std::max(avg_all / 255.0f, 0.0f), 1.0f);
            marker.color.r = normalized;
            marker.color.g = normalized;
            marker.color.b = normalized;

            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);

        // 7-1) CSV #1: cone_centres.csv → x, y, left/right_color
        {
            std::string path1 = ament_index_cpp::get_package_share_directory("lidar_integration")
                              + "/data/cone_centres.csv";
            std::ofstream out1(path1);
            out1 << "x,y,lr_color\n";
            for (const auto &t : centre_info) {
                float cx, cy; std::string lrcol;
                std::tie(cx, cy, lrcol) = t;
                out1 << cx << "," << cy << "," << lrcol << "\n";
            }
            out1.close();
        }

        // 7-2) CSV #2: cone_intensity.csv → avg_refl, color_label
        {
            std::string path2 = ament_index_cpp::get_package_share_directory("lidar_integration")
                              + "/data/cone_intensity.csv";
            std::ofstream out2(path2);
            out2 << "avg_refl,color_label\n";
            for (const auto &t : intensity_info) {
                float ar; std::string lbl;
                std::tie(ar, lbl) = t;
                out2 << ar << "," << lbl << "\n";
            }
            out2.close();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetection>());
    rclcpp::shutdown();
    return 0;
}
