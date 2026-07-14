#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/VLASearchBBox.h>
#include <quadrotor_msgs/VLASearchTarget.h>
#include <ros/ros.h>
#include <scene_graph/VLASearchObservation.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace {

template <typename MessagePtr>
MessagePtr nearestMessage(
    const std::deque<MessagePtr>& history, const ros::Time& stamp,
    double max_delta) {
  MessagePtr best;
  double best_delta = std::numeric_limits<double>::max();
  for (const auto& message : history) {
    const double delta = std::abs((message->header.stamp - stamp).toSec());
    if (delta < best_delta) {
      best_delta = delta;
      best = message;
    }
  }
  return best_delta <= max_delta ? best : MessagePtr();
}

Eigen::Matrix3d quaternionToMatrix(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(
             quaternion.w, quaternion.x, quaternion.y, quaternion.z)
      .normalized()
      .toRotationMatrix();
}

Eigen::Vector3d vectorParameter(
    ros::NodeHandle& node, const std::string& name,
    const std::vector<double>& defaults) {
  std::vector<double> values = defaults;
  node.param(name, values, defaults);
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (size_t index = 0; index < std::min<size_t>(3, values.size()); ++index) {
    result[static_cast<int>(index)] = values[index];
  }
  return result;
}

}  // namespace

class BBoxLidarTargetEstimator {
 public:
  BBoxLidarTargetEstimator() : node_(), private_node_("~") {
    private_node_.param(
        "bbox_topic", bbox_topic_, std::string("/vla_search/bbox"));
    private_node_.param(
        "fallback_topic", fallback_topic_,
        std::string("/vla_search/bbox/fallback"));
    private_node_.param(
        "target_topic", target_topic_, std::string("/vla_search/target"));
    private_node_.param(
        "camera_info_topic", camera_info_topic_,
        std::string("/camera/color/camera_info"));
    private_node_.param(
        "cloud_topic", cloud_topic_, std::string("/livox/lidar_sync"));
    private_node_.param(
        "odom_topic", odom_topic_, std::string("/odom_world"));
    private_node_.param(
        "observation_topic", observation_topic_,
        std::string("/vla_search/observation"));
    private_node_.param("cloud_in_world_frame", cloud_in_world_frame_, true);
    private_node_.param("history_duration", history_duration_, 8.0);
    private_node_.param("max_sync_delta", max_sync_delta_, 0.25);
    private_node_.param("min_points_in_bbox", min_points_in_bbox_, 6);
    private_node_.param("depth_gate", depth_gate_, 0.6);
    private_node_.param("min_depth", min_depth_, 0.2);
    private_node_.param("max_depth", max_depth_, 20.0);

    lidar_rotation_ = rpyDegreesToMatrix(
        vectorParameter(private_node_, "lidar_in_base_rpy_deg", {0.0, 0.0, 0.0}));
    lidar_translation_ = vectorParameter(
        private_node_, "lidar_in_base_t_xyz", {0.0, 0.0, 0.0});

    // Observation 0-3 来自同一个前视相机的不同时刻，机体 yaw 已包含在历史 odometry 中。
    const Eigen::Matrix3d camera_rotation = rpyDegreesToMatrix(
        vectorParameter(
            private_node_, "camera_in_base_rpy_deg", {0.0, 0.0, 0.0}));
    const Eigen::Vector3d camera_translation = vectorParameter(
        private_node_, "camera_in_base_t_xyz", {0.17, 0.10, 0.0});
    for (size_t index = 0; index < camera_rotations_.size(); ++index) {
      camera_rotations_[index] = camera_rotation;
      camera_translations_[index] = camera_translation;
    }

    camera_info_subscriber_ = node_.subscribe(
        camera_info_topic_, 1,
        &BBoxLidarTargetEstimator::cameraInfoCallback, this);
    cloud_subscriber_ = node_.subscribe(
        cloud_topic_, 10, &BBoxLidarTargetEstimator::cloudCallback, this);
    odom_subscriber_ = node_.subscribe(
        odom_topic_, 50, &BBoxLidarTargetEstimator::odomCallback, this);
    bbox_subscriber_ = node_.subscribe(
        bbox_topic_, 10, &BBoxLidarTargetEstimator::bboxCallback, this);
    observation_subscriber_ = node_.subscribe(
        observation_topic_, 10,
        &BBoxLidarTargetEstimator::observationCallback, this);
    fallback_publisher_ =
        node_.advertise<quadrotor_msgs::VLASearchBBox>(fallback_topic_, 10);
    target_publisher_ =
        node_.advertise<quadrotor_msgs::VLASearchTarget>(target_topic_, 10);
  }

 private:
  struct Intrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int width{0};
    int height{0};
    bool valid{false};
  };

  struct ObservationSnapshot {
    uint32_t task_session_id{0};
    uint32_t observation_batch_id{0};
    uint8_t observation_index{0};
    ros::Time stamp;
    geometry_msgs::Pose odom_pose;
    sensor_msgs::PointCloud2ConstPtr cloud;
  };

  static Eigen::Matrix3d rpyDegreesToMatrix(const Eigen::Vector3d& rpy) {
    return (
        Eigen::AngleAxisd(rpy.x() * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(rpy.y() * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy.z() * M_PI / 180.0, Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
  }

  template <typename MessagePtr>
  void trimHistory(std::deque<MessagePtr>& history) {
    const ros::Time oldest = ros::Time::now() - ros::Duration(history_duration_);
    while (!history.empty() && history.front()->header.stamp < oldest) {
      history.pop_front();
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    intrinsics_.fx = message->K[0];
    intrinsics_.cx = message->K[2];
    intrinsics_.fy = message->K[4];
    intrinsics_.cy = message->K[5];
    intrinsics_.width = static_cast<int>(message->width);
    intrinsics_.height = static_cast<int>(message->height);
    intrinsics_.valid =
        intrinsics_.fx > 0.0 && intrinsics_.fy > 0.0 &&
        intrinsics_.width > 0 && intrinsics_.height > 0;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_history_.push_back(message);
    trimHistory(cloud_history_);
    for (auto& snapshot : observation_snapshots_) {
      if (!snapshot.cloud &&
          std::abs((message->header.stamp - snapshot.stamp).toSec()) <=
              max_sync_delta_) {
        snapshot.cloud = message;
      }
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_history_.push_back(message);
    trimHistory(odom_history_);
  }

  void observationCallback(
      const scene_graph::VLASearchObservationConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const sensor_msgs::PointCloud2ConstPtr cloud =
        nearestMessage(cloud_history_, message->header.stamp, max_sync_delta_);
    ObservationSnapshot snapshot;
    snapshot.task_session_id = message->task_session_id;
    snapshot.observation_batch_id = message->observation_batch_id;
    snapshot.observation_index = message->observation_index;
    snapshot.stamp = message->header.stamp;
    const nav_msgs::OdometryConstPtr odom =
        nearestMessage(odom_history_, message->header.stamp, max_sync_delta_);
    snapshot.odom_pose = odom ? odom->pose.pose : message->odom_pose;
    snapshot.cloud = cloud;
    observation_snapshots_.push_back(snapshot);
    while (observation_snapshots_.size() > 16) {
      observation_snapshots_.pop_front();
    }
  }

  void publishFailure(
      const quadrotor_msgs::VLASearchBBox& request,
      const std::string& reason) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[VLA_SEARCH][LiDAR] " << reason << ", use MoGe fallback.");
    fallback_publisher_.publish(request);
  }

  void bboxCallback(const quadrotor_msgs::VLASearchBBoxConstPtr& message) {
    if (message->observation_index >= camera_rotations_.size()) {
      publishFailure(*message, "observation_index is outside [0,3]");
      return;
    }

    const ros::Time capture_stamp =
        message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    sensor_msgs::PointCloud2ConstPtr cloud;
    nav_msgs::OdometryConstPtr odom;
    geometry_msgs::Pose capture_pose;
    bool has_observation_snapshot = false;
    Intrinsics intrinsics;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto snapshot = observation_snapshots_.rbegin();
           snapshot != observation_snapshots_.rend(); ++snapshot) {
        if (snapshot->task_session_id == message->task_session_id &&
            snapshot->observation_batch_id ==
                message->observation_batch_id &&
            snapshot->observation_index == message->observation_index) {
          if (snapshot->cloud) {
            cloud = snapshot->cloud;
            capture_pose = snapshot->odom_pose;
            has_observation_snapshot = true;
            break;
          }
        }
      }
      if (!has_observation_snapshot) {
        cloud = nearestMessage(cloud_history_, capture_stamp, max_sync_delta_);
        odom = nearestMessage(odom_history_, capture_stamp, max_sync_delta_);
        if (odom) {
          capture_pose = odom->pose.pose;
        }
      }
      intrinsics = intrinsics_;
    }
    if (!cloud || (!has_observation_snapshot && !odom) || !intrinsics.valid) {
      publishFailure(*message, "historical cloud, odometry or intrinsics is unavailable");
      return;
    }

    int x0 = std::min(message->bbox_xyxy[0], message->bbox_xyxy[2]);
    int y0 = std::min(message->bbox_xyxy[1], message->bbox_xyxy[3]);
    int x1 = std::max(message->bbox_xyxy[0], message->bbox_xyxy[2]);
    int y1 = std::max(message->bbox_xyxy[1], message->bbox_xyxy[3]);
    x0 = std::max(0, std::min(intrinsics.width - 1, x0));
    x1 = std::max(0, std::min(intrinsics.width - 1, x1));
    y0 = std::max(0, std::min(intrinsics.height - 1, y0));
    y1 = std::max(0, std::min(intrinsics.height - 1, y1));
    if (x1 <= x0 || y1 <= y0) {
      publishFailure(*message, "bbox is invalid after image-bound clamping");
      return;
    }

    const Eigen::Matrix3d world_from_base =
        quaternionToMatrix(capture_pose.orientation);
    const Eigen::Vector3d world_translation(
        capture_pose.position.x, capture_pose.position.y,
        capture_pose.position.z);
    const Eigen::Matrix3d base_from_camera =
        camera_rotations_[message->observation_index];
    const Eigen::Matrix3d camera_from_base = base_from_camera.transpose();
    const Eigen::Vector3d camera_translation =
        camera_translations_[message->observation_index];

    std::vector<std::pair<double, Eigen::Vector3d>> candidates;
    sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*cloud, "z");
    for (; x_iterator != x_iterator.end();
         ++x_iterator, ++y_iterator, ++z_iterator) {
      Eigen::Vector3d point(
          static_cast<double>(*x_iterator), static_cast<double>(*y_iterator),
          static_cast<double>(*z_iterator));
      if (!point.allFinite()) {
        continue;
      }
      Eigen::Vector3d point_base;
      if (cloud_in_world_frame_) {
        // 世界系点云先变换到无人机机体系，便于后续统一投影到相机坐标系。
        point_base =
            world_from_base.transpose() * (point - world_translation);
      } else {
        // 雷达系点云通过外参直接变换到无人机机体系。
        point_base = lidar_rotation_ * point + lidar_translation_;
      }
      const Eigen::Vector3d point_camera =
          camera_from_base * (point_base - camera_translation);
      const double depth = point_camera.x();
      if (depth < min_depth_ || depth > max_depth_) {
        continue;
      }
      const double image_x =
          intrinsics.fx * (-point_camera.y()) / depth + intrinsics.cx;
      const double image_y =
          intrinsics.fy * (-point_camera.z()) / depth + intrinsics.cy;
      if (image_x >= x0 && image_x <= x1 &&
          image_y >= y0 && image_y <= y1) {
        candidates.emplace_back(depth, point_base);
      }
    }
    if (static_cast<int>(candidates.size()) < min_points_in_bbox_) {
      publishFailure(*message, "not enough LiDAR points in bbox");
      return;
    }

    std::vector<double> depths;
    depths.reserve(candidates.size());
    for (const auto& candidate : candidates) {
      depths.push_back(candidate.first);
    }
    std::nth_element(
        depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    const double median_depth = depths[depths.size() / 2];
    Eigen::Vector3d point_sum = Eigen::Vector3d::Zero();
    int point_count = 0;
    for (const auto& candidate : candidates) {
      if (std::abs(candidate.first - median_depth) <= depth_gate_) {
        point_sum += candidate.second;
        ++point_count;
      }
    }
    if (point_count < 3) {
      publishFailure(*message, "LiDAR depth cluster is unstable");
      return;
    }

    const Eigen::Vector3d target_world =
        world_from_base * (point_sum / point_count) + world_translation;
    quadrotor_msgs::VLASearchTarget target;
    target.header = message->header;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "world";
    target.task_session_id = message->task_session_id;
    target.observation_batch_id = message->observation_batch_id;
    target.request_id = message->request_id;
    target.observation_index = message->observation_index;
    target.success = true;
    target.source = quadrotor_msgs::VLASearchTarget::SOURCE_LIDAR;
    target.pose.position.x = target_world.x();
    target.pose.position.y = target_world.y();
    target.pose.position.z = target_world.z();
    target.pose.orientation.w = 1.0;
    target_publisher_.publish(target);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber bbox_subscriber_;
  ros::Subscriber observation_subscriber_;
  ros::Publisher fallback_publisher_;
  ros::Publisher target_publisher_;

  std::string bbox_topic_;
  std::string fallback_topic_;
  std::string target_topic_;
  std::string camera_info_topic_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string observation_topic_;
  bool cloud_in_world_frame_{true};
  double history_duration_{8.0};
  double max_sync_delta_{0.25};
  int min_points_in_bbox_{6};
  double depth_gate_{0.6};
  double min_depth_{0.2};
  double max_depth_{20.0};

  std::mutex mutex_;
  Intrinsics intrinsics_;
  std::deque<sensor_msgs::PointCloud2ConstPtr> cloud_history_;
  std::deque<nav_msgs::OdometryConstPtr> odom_history_;
  std::deque<ObservationSnapshot> observation_snapshots_;
  Eigen::Matrix3d lidar_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d lidar_translation_{Eigen::Vector3d::Zero()};
  std::array<Eigen::Matrix3d, 4> camera_rotations_;
  std::array<Eigen::Vector3d, 4> camera_translations_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bbox_lidar_target_estimator");
  BBoxLidarTargetEstimator estimator;
  ros::spin();
  return 0;
}
