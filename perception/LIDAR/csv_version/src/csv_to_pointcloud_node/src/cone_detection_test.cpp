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
#include <Eigen/Dense>
#include <pcl/common/centroid.h>

// this is for a test purpose, you can replace it to cone_detection_real.cpp to cone_detection.cpp to see all the results at once.
// # show only stage 1
// ros2 param set /cone_detection stage 1
// # show stages 1+2
// ros2 param set /cone_detection stage 2
// # show stages 1+2+3
// ros2 param set /cone_detection stage 3
// # show all 4
// ros2 param set /cone_detection stage 4




struct PointXYZR {
  float x, y, z;
  float reflectivity;
};

class ConeDetection : public rclcpp::Node {
public:
  ConeDetection()
  : Node("cone_detection")
  {
    // Declare a parameter "stage" (1–4) to control cumulative display
    this->declare_parameter<int>("stage", 4);

    // Subscribe to cropped point cloud
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cropped_pointcloud", 10,
      std::bind(&ConeDetection::pointcloud_callback, this, std::placeholders::_1));

    // Single publisher for cumulative markers
    active_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "active_markers", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr active_pub_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Read current stage parameter
    int stage = this->get_parameter("stage").as_int();

    // STAGE 1: reflectivity & Z filtering
    std::vector<PointXYZR> pts1;
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"),
                                          it_y(*msg, "y"),
                                          it_z(*msg, "z"),
                                          it_r(*msg, "intensity");
    while (it_x != it_x.end()) {
      if (*it_r > 25.0f && *it_z > -1.59f) {
        pts1.push_back({*it_x, *it_y, *it_z, *it_r});
      }
      ++it_x; ++it_y; ++it_z; ++it_r;
    }
    visualization_msgs::msg::MarkerArray m1;
    for (size_t i = 0; i < pts1.size(); ++i) {
      auto &p = pts1[i];
      visualization_msgs::msg::Marker mk;
      mk.header = msg->header;
      mk.ns     = "stage1";
      mk.id     = i;
      mk.type   = mk.SPHERE;
      mk.action = mk.ADD;
      mk.pose.position.x = p.x;
      mk.pose.position.y = p.y;
      mk.pose.position.z = p.z;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.05;
      mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0; mk.color.a = 0.5;
      m1.markers.push_back(mk);
    }

    // STAGE 2: voxel downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &p : pts1) {
      cloud2->points.emplace_back(p.x, p.y, p.z);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr vox(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud2);
    vg.setLeafSize(0.03f, 0.03f, 0.03f);
    vg.filter(*vox);
    visualization_msgs::msg::MarkerArray m2;
    for (size_t i = 0; i < vox->points.size(); ++i) {
      auto &p = vox->points[i];
      visualization_msgs::msg::Marker mk;
      mk.header = msg->header;
      mk.ns     = "stage2";
      mk.id     = i;
      mk.type   = mk.SPHERE;
      mk.action = mk.ADD;
      mk.pose.position.x = p.x;
      mk.pose.position.y = p.y;
      mk.pose.position.z = p.z;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.05;
      mk.color.r = 0.0; mk.color.g = 1.0; mk.color.b = 0.0; mk.color.a = 0.5;
      m2.markers.push_back(mk);
    }

    // STAGE 3: raw Euclidean clustering centroids
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(vox);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(50);
    ec.setSearchMethod(tree);
    ec.setInputCloud(vox);
    ec.extract(clusters);
    visualization_msgs::msg::MarkerArray m3;
    for (size_t cid = 0; cid < clusters.size(); ++cid) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
      for (int idx : clusters[cid].indices) {
        tmp->points.push_back(vox->points[idx]);
      }
      pcl::PointXYZ cen;
      pcl::computeCentroid(*tmp, cen);
      visualization_msgs::msg::Marker mk;
      mk.header = msg->header;
      mk.ns     = "stage3";
      mk.id     = cid;
      mk.type   = mk.SPHERE;
      mk.action = mk.ADD;
      mk.pose.position.x = cen.x;
      mk.pose.position.y = cen.y;
      mk.pose.position.z = cen.z;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.1;
      mk.color.r = 0.0; mk.color.g = 0.0; mk.color.b = 1.0; mk.color.a = 0.5;
      m3.markers.push_back(mk);
    }

    // STAGE 4: cylinder + size filtered centroids
    Eigen::Matrix3f M = Eigen::Matrix3f::Zero(); M(0,0)=1; M(1,1)=1;
    visualization_msgs::msg::MarkerArray m4;
    int id4 = 0;
    for (size_t cid = 0; cid < clusters.size(); ++cid) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<float> refls;
      for (int idx : clusters[cid].indices) {
        tmp->points.push_back(vox->points[idx]);
        refls.push_back(pts1[idx].reflectivity);
      }
      pcl::PointXYZ cen;
      pcl::computeCentroid(*tmp, cen);

      Eigen::Vector3f V; V << -2*cen.x, -2*cen.y, 0;
      float R = 0.5f, C = cen.x*cen.x + cen.y*cen.y - R*R;
      auto comp4 = std::make_shared<pcl::TfQuadraticXYZComparison<pcl::PointXYZ>>(
        pcl::ComparisonOps::LE, M, V, C);
      auto cond4 = std::make_shared<pcl::ConditionAnd<pcl::PointXYZ>>();
      cond4->addComparison(comp4);
      pcl::PointCloud<pcl::PointXYZ> rec;
      pcl::ConditionalRemoval<pcl::PointXYZ> cr(true);
      cr.setCondition(cond4);
      cr.setInputCloud(tmp);
      cr.setKeepOrganized(false);
      cr.filter(rec);
      if (rec.empty()) continue;

      float minx=std::numeric_limits<float>::max(), maxx=-minx;
      float miny=minx, maxy=-minx;
      float minz=minx, maxz=-minx;
      float sx=0, sy=0, sz=0;
      for (auto &p : rec) {
        sx += p.x; sy += p.y; sz += p.z;
        minx = std::min(minx,p.x); maxx = std::max(maxx,p.x);
        miny = std::min(miny,p.y); maxy = std::max(maxy,p.y);
        minz = std::min(minz,p.z); maxz = std::max(maxz,p.z);
      }
      float w = maxx-minx, d = maxy-miny, h = maxz-minz;
      if (!(h>=0.1f && h<=0.45f && w>=0.1f && w<=0.25f && d>=0.1f && d<=0.35f))
        continue;

      pcl::PointXYZ c4; c4.x = sx/rec.size(); c4.y = sy/rec.size(); c4.z = sz/rec.size();
      visualization_msgs::msg::Marker mk;
      mk.header = msg->header;
      mk.ns     = "stage4";
      mk.id     = id4++;
      mk.type   = mk.SPHERE;
      mk.action = mk.ADD;
      mk.pose.position.x = c4.x;
      mk.pose.position.y = c4.y;
      mk.pose.position.z = c4.z;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.2;
      mk.color.r = 1.0; mk.color.g = 1.0; mk.color.b = 0.0; mk.color.a = 0.8;
      m4.markers.push_back(mk);
    }

    // Build and publish cumulative markers
    visualization_msgs::msg::MarkerArray out;
    if (stage >= 1) { out.markers.insert(out.markers.end(), m1.markers.begin(), m1.markers.end()); }
    if (stage >= 2) { out.markers.insert(out.markers.end(), m2.markers.begin(), m2.markers.end()); }
    if (stage >= 3) { out.markers.insert(out.markers.end(), m3.markers.begin(), m3.markers.end()); }
    if (stage >= 4) { out.markers.insert(out.markers.end(), m4.markers.begin(), m4.markers.end()); }
    active_pub_->publish(out);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetection>());
  rclcpp::shutdown();
  return 0;
}
