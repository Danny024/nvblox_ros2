#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nvblox/nvblox.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <stdexcept>

class NvbloxRos2Node : public rclcpp::Node {
public:
  NvbloxRos2Node() : Node("nvblox_ros2_node"), 
                     tf_buffer_(get_clock()), 
                     tf_listener_(tf_buffer_),
                     lidar_(360, 16, 0.1f, 100.0f, 0.0f) {
    declare_parameter("voxel_size", 0.1);
    declare_parameter("depth_topic", "/camera/depth/image_rect_raw");
    declare_parameter("color_topic", "/camera/color/image_raw");
    declare_parameter("pose_topic", "/camera/pose");
    declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    declare_parameter("lidar_topic", "/lidar/points");
    declare_parameter("map_frame", "map");
    declare_parameter("max_triangles", 10000);
    declare_parameter("lidar_width", 360);
    declare_parameter("lidar_height", 16);
    declare_parameter("lidar_min_angle", -M_PI);
    declare_parameter("lidar_max_angle", M_PI);
    declare_parameter("lidar_min_range", 0.1);
    declare_parameter("lidar_max_range", 100.0);

    voxel_size_ = get_parameter("voxel_size").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    max_triangles_ = get_parameter("max_triangles").as_int();
    lidar_width_ = get_parameter("lidar_width").as_int();
    lidar_height_ = get_parameter("lidar_height").as_int();
    lidar_min_angle_ = get_parameter("lidar_min_angle").as_double();
    lidar_max_angle_ = get_parameter("lidar_max_angle").as_double();
    lidar_min_range_ = get_parameter("lidar_min_range").as_double();
    lidar_max_range_ = get_parameter("lidar_max_range").as_double();

    lidar_ = nvblox::Lidar(lidar_width_, lidar_height_, lidar_min_range_, lidar_max_range_, 0.0f);
    mapper_ = std::make_unique<nvblox::Mapper>(voxel_size_);

    depth_sub_.subscribe(this, get_parameter("depth_topic").as_string());
    color_sub_.subscribe(this, get_parameter("color_topic").as_string());
    pose_sub_.subscribe(this, get_parameter("pose_topic").as_string());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), depth_sub_, color_sub_, pose_sub_);
    sync_->registerCallback(std::bind(&NvbloxRos2Node::cameraSyncCallback, this,
                                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        get_parameter("lidar_topic").as_string(), 10,
        std::bind(&NvbloxRos2Node::lidarCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("camera_info_topic").as_string(), 10,
        std::bind(&NvbloxRos2Node::cameraInfoCallback, this, std::placeholders::_1));

    mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>("nvblox/mesh", 10);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("nvblox/costmap", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&NvbloxRos2Node::publishOutputs, this));

    RCLCPP_INFO(this->get_logger(), "Initialized nvblox ROS 2 node with voxel size: %.2f", voxel_size_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>;

  void cameraSyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                          const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                          const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!camera_info_received_) {
      static int warn_count = 0;
      if (warn_count++ < 5) { // Limit warnings to avoid flooding logs
        RCLCPP_WARN(this->get_logger(), "Waiting for camera info, skipping camera data. Ensure %s is publishing.",
                    get_parameter("camera_info_topic").as_string().c_str());
      }
      // Fallback: Use default intrinsics after 5 seconds if no camera info
      if (this->now().seconds() - this->get_clock()->now().seconds() > 5.0 && !camera_info_received_) {
        RCLCPP_WARN(this->get_logger(), "No camera info received after 5s, using default intrinsics (640x480, fx=fy=525, cx=320, cy=240).");
        camera_ = nvblox::Camera(525.0, 525.0, 320.0, 240.0, 640, 480); // Default for common cameras
        camera_info_received_ = true;
      }
      return;
    }

    try {
      // Validate depth format
      if (depth_msg->encoding != "32FC1") {
        throw std::runtime_error("Depth image encoding must be 32FC1, got: " + depth_msg->encoding);
      }
      RCLCPP_DEBUG(this->get_logger(), "Received depth image: %dx%d", depth_msg->width, depth_msg->height);
      nvblox::DepthImage depth_image(depth_msg->height, depth_msg->width, nvblox::MemoryType::kUnified);
      std::memcpy(depth_image.dataPtr(), depth_msg->data.data(), depth_msg->height * depth_msg->width * sizeof(float));

      nvblox::Transform T_L_C = convertPose(pose_msg);
      mapper_->integrateDepth(depth_image, T_L_C, camera_);

      if (color_msg) {
        if (color_msg->encoding != "rgb8") {
          throw std::runtime_error("Color image encoding must be rgb8, got: " + color_msg->encoding);
        }
        RCLCPP_DEBUG(this->get_logger(), "Received color image: %dx%d", color_msg->width, color_msg->height);
        nvblox::ColorImage color_image(color_msg->height, color_msg->width, nvblox::MemoryType::kUnified);
        std::memcpy(color_image.dataPtr(), color_msg->data.data(), color_msg->height * color_msg->width * 3 * sizeof(uint8_t));
        mapper_->integrateColor(color_image, T_L_C, camera_);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "No color data provided, skipping color integration.");
      }

      latest_timestamp_ = depth_msg->header.stamp;
      latest_frame_ = depth_msg->header.frame_id;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process camera data: %s", e.what());
    }
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      nvblox::DepthImage depth_image(lidar_height_, lidar_width_, nvblox::MemoryType::kUnified);
      std::memset(depth_image.dataPtr(), 0, lidar_height_ * lidar_width_ * sizeof(float));

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x, y = *iter_y, z = *iter_z;
        float range = std::sqrt(x * x + y * y + z * z);
        float theta = std::atan2(y, x);  // Azimuth angle
        float phi = std::acos(z / range);  // Elevation angle

        int u = static_cast<int>((theta - lidar_min_angle_) / (lidar_max_angle_ - lidar_min_angle_) * lidar_width_);
        int v = static_cast<int>((phi / M_PI) * lidar_height_);

        if (u >= 0 && u < lidar_width_ && v >= 0 && v < lidar_height_) {
          depth_image(v, u) = range;
        }
      }

      nvblox::Transform T_L_S = lookupTransform(msg->header.frame_id, map_frame_, msg->header.stamp);
      mapper_->integrateLidarDepth(depth_image, T_L_S, lidar_);

      latest_timestamp_ = msg->header.stamp;
      latest_frame_ = msg->header.frame_id;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process LiDAR data: %s", e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!camera_info_received_) {
      try {
        camera_ = nvblox::Camera(msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->width, msg->height);
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info set: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, %dx%d",
                    msg->k[0], msg->k[4], msg->k[2], msg->k[5], msg->width, msg->height);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set camera info: %s", e.what());
      }
    }
  }

  void publishOutputs() {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (!tf_buffer_.canTransform(map_frame_, latest_frame_, latest_timestamp_, rclcpp::Duration::from_seconds(0.1))) {
        RCLCPP_WARN(this->get_logger(), "No valid transform from %s to %s, skipping.", latest_frame_.c_str(), map_frame_.c_str());
        return;
      }

      mapper_->updateMesh();
      const nvblox::MeshLayer& mesh_layer = mapper_->mesh_layer();
      auto mesh_msg = convertMeshToMarker(mesh_layer);
      if (mesh_msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Mesh generation produced no triangles; check depth integration.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Generated mesh with %zu triangles.", mesh_msg->points.size() / 3);
      }
      mesh_msg->header.frame_id = map_frame_;
      mesh_msg->header.stamp = latest_timestamp_;
      mesh_pub_->publish(*mesh_msg);

      mapper_->updateEsdf();
      const nvblox::EsdfLayer& esdf_layer = mapper_->esdf_layer();
      auto costmap_msg = convertEsdfToCostmap(esdf_layer, 0.0);
      costmap_msg->header.frame_id = map_frame_;
      costmap_msg->header.stamp = latest_timestamp_;
      costmap_pub_->publish(*costmap_msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish outputs: %s", e.what());
    }
  }

  nvblox::DepthImage convertDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (msg->encoding != "32FC1") throw std::runtime_error("Depth image must be 32FC1");
    nvblox::DepthImage depth(msg->height, msg->width, nvblox::MemoryType::kUnified);
    std::memcpy(depth.dataPtr(), msg->data.data(), msg->height * msg->width * sizeof(float));
    return depth;
  }

  nvblox::ColorImage convertColorImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (msg->encoding != "rgb8") throw std::runtime_error("Color image must be rgb8");
    nvblox::ColorImage color(msg->height, msg->width, nvblox::MemoryType::kUnified);
    std::memcpy(color.dataPtr(), msg->data.data(), color.height() * color.width() * 3 * sizeof(uint8_t));
    return color;
  }

  nvblox::Transform convertPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
    Eigen::Quaternionf q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Affine3f affine = Eigen::Translation3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) *
                             q.toRotationMatrix();
    return nvblox::Transform(affine.matrix());
  }

  nvblox::Transform lookupTransform(const std::string& from_frame, const std::string& to_frame,
                                    const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(to_frame, from_frame, stamp, rclcpp::Duration::from_seconds(0.1));
    Eigen::Quaternionf q(transform.transform.rotation.w, transform.transform.rotation.x,
                         transform.transform.rotation.y, transform.transform.rotation.z);
    Eigen::Affine3f affine = Eigen::Translation3f(transform.transform.translation.x,
                                                  transform.transform.translation.y,
                                                  transform.transform.translation.z) *
                             q.toRotationMatrix();
    return nvblox::Transform(affine.matrix());
  }

  visualization_msgs::msg::Marker::UniquePtr convertMeshToMarker(const nvblox::MeshLayer& mesh_layer) {
    auto marker = std::make_unique<visualization_msgs::msg::Marker>();
    marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->scale.x = marker->scale.y = marker->scale.z = 1.0;
    marker->color.a = 1.0;

    std::vector<nvblox::Index3D> block_indices = mesh_layer.getAllBlockIndices();
    size_t triangle_count = 0;
    for (const auto& idx : block_indices) {
      const nvblox::MeshBlock::ConstPtr block = mesh_layer.getBlockAtIndex(idx);
      if (block) triangle_count += block->triangles.size() / 3; // Triplets
    }
    triangle_count = std::min(static_cast<size_t>(max_triangles_), triangle_count);

    marker->points.reserve(triangle_count * 3);
    marker->colors.reserve(triangle_count * 3);

    size_t triangles_added = 0;
    for (const auto& idx : block_indices) {
      const nvblox::MeshBlock::ConstPtr block = mesh_layer.getBlockAtIndex(idx);
      if (!block) continue;

      // Check triangles size is multiple of 3
      if (block->triangles.size() % 3 != 0) {
        RCLCPP_WARN(this->get_logger(), "Block at index %d has %zu triangle indices, not a multiple of 3; mesh may be malformed.",
                    idx.x(), block->triangles.size());
      }

      for (size_t i = 0; i + 2 < block->triangles.size(); i += 3) {
        if (triangles_added >= max_triangles_) break;
        for (int j = 0; j < 3; ++j) {
          const int vertex_idx = block->triangles[i + j];
          if (vertex_idx < 0 || vertex_idx >= static_cast<int>(block->vertices.size())) {
            RCLCPP_WARN(this->get_logger(), "Invalid vertex index %d in block at %d; skipping triangle.", vertex_idx, idx.x());
            continue;
          }
          const auto& vertex = block->vertices[vertex_idx];
          geometry_msgs::msg::Point p;
          p.x = vertex.x();
          p.y = vertex.y();
          p.z = vertex.z();
          marker->points.push_back(p);

          if (!block->colors.empty() && vertex_idx < static_cast<int>(block->colors.size())) {
            const auto& color = block->colors[vertex_idx];
            std_msgs::msg::ColorRGBA c;
            c.r = color.r / 255.0f;
            c.g = color.g / 255.0f;
            c.b = color.b / 255.0f;
            c.a = 1.0;
            marker->colors.push_back(c);
          }
        }
        triangles_added++;
      }
      if (triangles_added >= max_triangles_) break;
    }

    if (marker->colors.empty()) {
      marker->color.r = 0.5;
      marker->color.g = 0.5;
      marker->color.b = 0.5;
    }
    return marker;
  }

  nav_msgs::msg::OccupancyGrid::UniquePtr convertEsdfToCostmap(const nvblox::EsdfLayer& esdf_layer, float z_slice) {
    auto costmap = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    costmap->info.resolution = voxel_size_;
    costmap->info.width = static_cast<int>(esdf_layer.block_size() / voxel_size_);
    costmap->info.height = costmap->info.width;
    costmap->info.origin.position.z = z_slice;
    costmap->data.resize(costmap->info.width * costmap->info.height, 0); // Default to unknown (0)

    RCLCPP_WARN(this->get_logger(), "ESDF costmap generation limited due to missing esdf_layer.h; using placeholder.");
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
  nvblox::Camera camera_;
  nvblox::Lidar lidar_;
  int lidar_width_;
  int lidar_height_;
  double lidar_min_angle_;
  double lidar_max_angle_;
  double lidar_min_range_;
  double lidar_max_range_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NvbloxRos2Node>());
  rclcpp::shutdown();
  return 0;
}
