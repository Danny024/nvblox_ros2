#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nvblox/nvblox.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NvbloxRos2Node : public rclcpp::Node {
public:
  NvbloxRos2Node() : Node("nvblox_ros2_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    // Declare parameters with mitigation defaults
    declare_parameter("voxel_size", 0.1);  // Adjustable for performance
    declare_parameter("depth_topic", "/camera/depth/image_rect_raw");
    declare_parameter("color_topic", "/camera/color/image_raw");
    declare_parameter("pose_topic", "/camera/pose");
    declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    declare_parameter("lidar_topic", "/lidar/points");
    declare_parameter("map_frame", "map");
    declare_parameter("max_triangles", 10000);  // Limit mesh size for RViz2

    // Retrieve parameters
    voxel_size_ = get_parameter("voxel_size").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    max_triangles_ = get_parameter("max_triangles").as_int();

    // Initialize nvblox mapper
    mapper_ = std::make_unique<nvblox::Mapper>(voxel_size_, nvblox::MemoryType::kUnified);

    // Set up synchronized subscribers
    depth_sub_.subscribe(this, get_parameter("depth_topic").as_string());
    color_sub_.subscribe(this, get_parameter("color_topic").as_string());
    pose_sub_.subscribe(this, get_parameter("pose_topic").as_string());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), depth_sub_, color_sub_, pose_sub_);
    sync_->registerCallback(std::bind(&NvbloxRos2Node::cameraSyncCallback, this,
                                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Independent subscribers
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        get_parameter("lidar_topic").as_string(), 10,
        std::bind(&NvbloxRos2Node::lidarCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("camera_info_topic").as_string(), 10,
        std::bind(&NvbloxRos2Node::cameraInfoCallback, this, std::placeholders::_1));

    // Publishers
    mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>("nvblox/mesh", 10);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("nvblox/costmap", 10);

    // Timer for outputs
    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&NvbloxRos2Node::publishOutputs, this));

    RCLCPP_INFO(this->get_logger(), "Initialized nvblox ROS 2 node with voxel size: %.2f, max triangles: %d",
                voxel_size_, max_triangles_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>;

  void cameraSyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                          const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!camera_info_received_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for camera info, skipping camera data.");
      return;
    }

    try {
      nvblox::DepthImage depth = convertDepthImage(depth_msg);
      nvblox::Transform T_L_C = convertPose(pose_msg);
      mapper_->integrateDepth(depth, T_L_C);

      if (color_msg) {
        nvblox::ColorImage color = convertColorImage(color_msg);
        mapper_->integrateColor(color, T_L_C);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "No color data provided, using default gray.");
      }

      latest_timestamp_ = depth_msg->header.stamp;
      latest_frame_ = depth_msg->header.frame_id;  // For TF validation
      RCLCPP_DEBUG(this->get_logger(), "Processed camera data at timestamp: %f",
                   latest_timestamp_.seconds());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process camera data: %s", e.what());
    }
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      nvblox::Pointcloud pointcloud = convertPointCloud(msg);
      nvblox::Transform T_L_S = lookupTransform(msg->header.frame_id, map_frame_, msg->header.stamp);
      mapper_->integratePointcloud(pointcloud, T_L_S);

      latest_timestamp_ = msg->header.stamp;
      latest_frame_ = msg->header.frame_id;
      RCLCPP_DEBUG(this->get_logger(), "Processed LiDAR data at timestamp: %f",
                   latest_timestamp_.seconds());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process LiDAR data: %s", e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!camera_info_received_) {
      try {
        nvblox::Camera camera(msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->width, msg->height);
        mapper_->setCamera(camera);
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Set camera intrinsics: fx=%.2f, fy=%.2f, width=%d, height=%d",
                    msg->k[0], msg->k[4], msg->width, msg->height);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set camera info: %s", e.what());
      }
    }
  }

  void publishOutputs() {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      // Validate TF before publishing
      if (!tf_buffer_.canTransform(map_frame_, latest_frame_, latest_timestamp_, rclcpp::Duration(0.1))) {
        RCLCPP_WARN(this->get_logger(), "No valid transform from %s to %s, skipping publication.",
                    latest_frame_.c_str(), map_frame_.c_str());
        return;
      }

      nvblox::Mesh mesh;
      mapper_->generateMesh(&mesh);
      auto mesh_msg = convertMeshToMarker(mesh);
      mesh_msg->header.frame_id = map_frame_;
      mesh_msg->header.stamp = latest_timestamp_;
      mesh_pub_->publish(mesh_msg);

      nvblox::Esdf esdf;
      mapper_->generateEsdf(&esdf);
      auto costmap_msg = convertEsdfToCostmap(esdf, 0.0);
      costmap_msg->header.frame_id = map_frame_;
      costmap_msg->header.stamp = latest_timestamp_;
      costmap_pub_->publish(costmap_msg);

      RCLCPP_DEBUG(this->get_logger(), "Published mesh and costmap at timestamp: %f",
                   latest_timestamp_.seconds());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish outputs: %s", e.what());
    }
  }

  nvblox::DepthImage convertDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (msg->encoding != "32FC1") {
      throw std::runtime_error("Expected 32FC1 encoding for depth image, got: " + msg->encoding);
    }
    nvblox::DepthImage depth(msg->height, msg->width, nvblox::MemoryType::kUnified);
    std::memcpy(depth.data(), msg->data.data(), msg->height * msg->width * sizeof(float));
    return depth;
  }

  nvblox::ColorImage convertColorImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (msg->encoding != "rgb8") {
      throw std::runtime_error("Expected rgb8 encoding for color image, got: " + msg->encoding);
    }
    nvblox::ColorImage color(msg->height, msg->width, nvblox::MemoryType::kUnified);
    std::memcpy(color.data(), msg->data.data(), msg->height * msg->width * 3 * sizeof(uint8_t));
    return color;
  }

  nvblox::Pointcloud convertPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    nvblox::Pointcloud pc(nvblox::MemoryType::kUnified);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
    if (!iter_x.is_valid() || !iter_y.is_valid() || !iter_z.is_valid()) {
      throw std::runtime_error("PointCloud2 must contain valid x, y, z fields");
    }
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      pc.points().push_back(nvblox::Vector3f(*iter_x, *iter_y, *iter_z));
    }
    return pc;
  }

  nvblox::Transform convertPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    nvblox::Transform T;
    T.translation() = nvblox::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    T.rotation() = nvblox::Quaternion(msg->pose.orientation.w, msg->pose.orientation.x,
                                      msg->pose.orientation.y, msg->pose.orientation.z);
    return T;
  }

  nvblox::Transform lookupTransform(const std::string& from_frame, const std::string& to_frame,
                                    const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(to_frame, from_frame, stamp, rclcpp::Duration(0.1));
    } catch (const tf2::TransformException& ex) {
      throw std::runtime_error("TF lookup failed: " + std::string(ex.what()));
    }
    nvblox::Transform T;
    T.translation() = nvblox::Vector3f(transform.transform.translation.x,
                                       transform.transform.translation.y,
                                       transform.transform.translation.z);
    T.rotation() = nvblox::Quaternion(transform.transform.rotation.w,
                                      transform.transform.rotation.x,
                                      transform.transform.rotation.y,
                                      transform.transform.rotation.z);
    return T;
  }

  visualization_msgs::msg::Marker::UniquePtr convertMeshToMarker(const nvblox::Mesh& mesh) {
    auto marker = std::make_unique<visualization_msgs::msg::Marker>();
    marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->scale.x = marker->scale.y = marker->scale.z = 1.0;
    marker->color.a = 1.0;

    if (mesh.triangles.empty() || mesh.vertices.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty mesh data, publishing empty marker.");
      return marker;
    }

    size_t triangle_count = std::min(static_cast<size_t>(max_triangles_), mesh.triangles.size());
    marker->points.reserve(triangle_count * 3);
    if (!mesh.colors.empty()) {
      marker->colors.reserve(triangle_count * 3);
    }

    for (size_t i = 0; i < triangle_count; ++i) {
      const auto& triangle = mesh.triangles[i];
      if (triangle[0] >= mesh.vertices.size() || triangle[1] >= mesh.vertices.size() ||
          triangle[2] >= mesh.vertices.size()) {
        RCLCPP_WARN(this->get_logger(), "Invalid triangle indices at %zu, skipping.", i);
        continue;
      }

      for (int j = 0; j < 3; ++j) {
        const auto& vertex = mesh.vertices[triangle[j]];
        geometry_msgs::msg::Point p;
        p.x = vertex.x();
        p.y = vertex.y();
        p.z = vertex.z();
        marker->points.push_back(p);

        if (!mesh.colors.empty() && triangle[j] < mesh.colors.size()) {
          const auto& color = mesh.colors[triangle[j]];
          std_msgs::msg::ColorRGBA c;
          c.r = color.r / 255.0f;
          c.g = color.g / 255.0f;
          c.b = color.b / 255.0f;
          c.a = 1.0;
          marker->colors.push_back(c);
        }
      }
    }

    if (marker->colors.empty()) {
      marker->color.r = 0.5;
      marker->color.g = 0.5;
      marker->color.b = 0.5;
      RCLCPP_DEBUG(this->get_logger(), "No color data, using default gray.");
    }

    RCLCPP_DEBUG(this->get_logger(), "Generated mesh marker with %zu triangles (limited to %d)",
                 triangle_count, max_triangles_);
    return marker;
  }

  nav_msgs::msg::OccupancyGrid::UniquePtr convertEsdfToCostmap(const nvblox::Esdf& esdf, float z_slice) {
    auto costmap = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    costmap->info.resolution = voxel_size_;
    costmap->info.width = esdf.width();
    costmap->info.height = esdf.height();
    costmap->info.origin.position.x = esdf.origin().x();
    costmap->info.origin.position.y = esdf.origin().y();
    costmap->info.origin.position.z = z_slice;
    costmap->data.resize(esdf.width() * esdf.height());
    for (int y = 0; y < esdf.height(); ++y) {
      for (int x = 0; x < esdf.width(); ++x) {
        float dist = esdf.getDistanceAt(x, y, z_slice / voxel_size_);
        costmap->data[y * esdf.width() + x] = (dist < 0.0) ? 100 : (dist < voxel_size_ ? 50 : 0);
      }
    }
    return costmap;
  }

  std::unique_ptr<nvblox::Mapper> mapper_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_, color_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex mutex_;
  bool camera_info_received_ = false;
  double voxel_size_;
  std::string map_frame_;
  std::string latest_frame_;
  rclcpp::Time latest_timestamp_{0, 0, RCL_ROS_TIME};
  int max_triangles_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NvbloxRos2Node>());
  rclcpp::shutdown();
  return 0;
}
