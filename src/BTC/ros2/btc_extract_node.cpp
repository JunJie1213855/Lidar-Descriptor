#include "btc.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <functional>

class BtcExtractNode : public rclcpp::Node {
public:
  BtcExtractNode() : Node("btc_extract_node") {
    const auto topic = declare_parameter<std::string>("input_topic", "/points_raw");
    output_dir_ = declare_parameter<std::string>("output_dir", "/tmp/btc_descriptors");
    max_visualized_ = std::max<int>(0, declare_parameter<int>("max_visualized_descriptors", 100));
    auto config_file = declare_parameter<std::string>("config_file", "");
    if (config_file.empty()) throw std::runtime_error("config_file is required");
    ConfigSetting config;
    load_config_setting(config_file, config);
    manager_ = std::make_unique<BtcDescManager>(config);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/btc/descriptors", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&BtcExtractNode::onCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "BTC extraction: %s -> %s", topic.c_str(), output_dir_.c_str());
  }
private:
  void publishVisualization(const std_msgs::msg::Header &header,
                            const std::vector<BTC> &descriptors) {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    Marker edges;
    edges.header = header;
    edges.ns = "btc_triangles";
    edges.id = 0;
    edges.type = Marker::LINE_LIST;
    edges.action = Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.08;
    edges.color.r = 1.0f;
    edges.color.g = 0.35f;
    edges.color.b = 0.1f;
    edges.color.a = 0.85f;

    Marker vertices = edges;
    vertices.ns = "btc_vertices";
    vertices.type = Marker::SPHERE_LIST;
    vertices.scale.x = vertices.scale.y = vertices.scale.z = 0.18;
    vertices.color.r = 1.0f;
    vertices.color.g = 0.9f;
    vertices.color.b = 0.2f;

    if (max_visualized_ > 0) {
      const size_t step = std::max<size_t>(
          1, (descriptors.size() + max_visualized_ - 1) / max_visualized_);
      for (size_t i = 0; i < descriptors.size(); i += step) {
        const auto &d = descriptors[i];
        geometry_msgs::msg::Point points[3];
        const BinaryDescriptor *binaries[3] = {&d.binary_A_, &d.binary_B_, &d.binary_C_};
        for (int j = 0; j < 3; ++j) {
          points[j].x = binaries[j]->location_.x();
          points[j].y = binaries[j]->location_.y();
          points[j].z = binaries[j]->location_.z();
          vertices.points.push_back(points[j]);
        }
        for (int j = 0; j < 3; ++j) {
          edges.points.push_back(points[j]);
          edges.points.push_back(points[(j + 1) % 3]);
        }
      }
    }
    array.markers.push_back(edges);
    array.markers.push_back(vertices);
    marker_publisher_->publish(array);
  }
  static std::string bits(const BinaryDescriptor &d) {
    std::string s;
    s.reserve(d.occupy_array_.size());
    for (bool value : d.occupy_array_) s.push_back(value ? '1' : '0');
    return s;
  }
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    try {
      pcl::fromROSMsg(*msg, *cloud);
      if (cloud->empty()) return;
      const int frame_id = static_cast<int>(sequence_++);
      std::vector<BTC> descriptors;
      manager_->GenerateBtcDescs(cloud, frame_id, descriptors);
      publishVisualization(msg->header, descriptors);
      std::filesystem::create_directories(output_dir_);
      const auto path = std::filesystem::path(output_dir_) /
          ("btc_" + std::to_string(frame_id) + ".csv");
      std::ofstream file(path);
      if (!file) throw std::runtime_error("cannot open output file");
      file << "frame_id,triangle_a,triangle_b,triangle_c,angle_a,angle_b,angle_c,center_x,center_y,center_z,ax,ay,az,a_summary,a_bits,bx,by,bz,b_summary,b_bits,cx,cy,cz,c_summary,c_bits\n";
      file << std::setprecision(17);
      for (const auto &d : descriptors) {
        file << d.frame_number_;
        for (const auto *v : {&d.triangle_, &d.angle_, &d.center_})
          for (int i = 0; i < 3; ++i) file << ',' << (*v)[i];
        for (const auto *b : {&d.binary_A_, &d.binary_B_, &d.binary_C_}) {
          for (int i = 0; i < 3; ++i) file << ',' << b->location_[i];
          file << ',' << static_cast<int>(b->summary_) << ',' << bits(*b);
        }
        file << '\n';
      }
      RCLCPP_INFO(get_logger(), "Wrote %zu BTC descriptors: %s", descriptors.size(), path.c_str());
      manager_->plane_cloud_vec_.clear();
      manager_->history_binary_list_.clear();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "BTC extraction failed: %s", e.what());
    }
  }
  std::string output_dir_;
  int max_visualized_ = 100;
  size_t sequence_ = 0;
  std::unique_ptr<BtcDescManager> manager_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BtcExtractNode>());
  rclcpp::shutdown();
  return 0;
}
