#ifndef VLA_SWARM_MAP_H
#define VLA_SWARM_MAP_H

#include <Eigen/Eigen>
#include <map_interface/map_interface.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class SceneGraph;

namespace ego_planner {

struct VLASwarmRoom {
  int id{-1};
  cv::Rect pixel_box;
  Eigen::Vector2d world_min{Eigen::Vector2d::Zero()};
  Eigen::Vector2d world_max{Eigen::Vector2d::Zero()};
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double area{0.0};
  std::string description;
};

struct VLASwarmDoor {
  int id{-1};
  cv::Point pixel;
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  bool frontier{false};
  std::vector<int> room_ids;
};

/**
 * VLA_Swarm 单机二维地图语义模块。
 *
 * 该模块只读取当前 MapInterface，不订阅或回灌多机地图消息。SmallMap 图像、房间、
 * 门和 Prompt JSON 均共享同一套世界/像素转换，后续路径阶段可直接复用世界坐标。
 */
class VLASwarmMap {
 public:
  using Ptr = std::shared_ptr<VLASwarmMap>;

  VLASwarmMap(ros::NodeHandle& nh, const MapInterface::Ptr& map);

  bool update(const Eigen::Vector3d& robot_position);
  void reset();

  bool ready() const;
  uint64_t mapSequence() const;
  cv::Mat image() const;
  std::vector<VLASwarmRoom> rooms() const;
  std::vector<VLASwarmDoor> doors() const;
  bool findDoor(int door_id, VLASwarmDoor& door) const;
  bool planDoorPath(const Eigen::Vector3d& start_position, int door_id,
                    double flight_height, double waypoint_distance,
                    std::vector<Eigen::Vector3d>& path) const;
  nlohmann::json promptContext(const SceneGraph& scene_graph,
                               const Eigen::Vector3d& robot_position) const;

  bool worldToPixel(const Eigen::Vector2d& world, cv::Point& pixel) const;
  bool pixelToWorld(const cv::Point& pixel, Eigen::Vector2d& world) const;

 private:
  struct DoorCandidate {
    cv::Point pixel;
    bool frontier{false};
    std::vector<int> room_ids;
  };

  void buildOccupancyImage(cv::Mat& image, cv::Mat& free_mask,
                           cv::Mat& unknown_mask) const;
  void segmentRooms(const cv::Mat& free_mask, cv::Mat& room_labels,
                    std::vector<VLASwarmRoom>& rooms);
  void detectDoors(const cv::Mat& free_mask, const cv::Mat& unknown_mask,
                   const cv::Mat& room_labels,
                   std::vector<VLASwarmDoor>& doors);
  void assignStableRoomIds(std::vector<VLASwarmRoom>& rooms);
  void assignStableDoorIds(std::vector<VLASwarmDoor>& doors);
  void drawSemanticOverlay(cv::Mat& image, const cv::Mat& room_labels,
                           const Eigen::Vector3d& robot_position) const;
  void publishImage(const cv::Mat& image) const;

  static double rectangleOverlapRatio(const cv::Rect& lhs, const cv::Rect& rhs);
  static std::string roomLabel(int id);

  MapInterface::Ptr map_;
  ros::Publisher image_pub_;
  std::string image_topic_{"/vla_swarm/small_map"};
  std::string frame_id_{"world"};

  int image_width_{320};
  int image_height_{240};
  double sample_height_min_{0.6};
  double sample_height_max_{1.6};
  double sample_height_step_{0.4};
  int min_room_area_px_{300};
  int room_erode_iterations_{4};
  int door_min_width_px_{4};
  int door_max_width_px_{30};
  int door_merge_distance_px_{12};
  int astar_clearance_px_{2};

  Eigen::Vector3d map_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d map_max_{Eigen::Vector3d::Zero()};
  double pixels_per_meter_x_{1.0};
  double pixels_per_meter_y_{1.0};

  mutable std::mutex mutex_;
  bool ready_{false};
  uint64_t map_sequence_{0};
  int next_room_id_{0};
  int next_door_id_{0};
  cv::Mat image_;
  cv::Mat free_mask_;
  cv::Mat room_labels_;
  std::vector<VLASwarmRoom> rooms_;
  std::vector<VLASwarmDoor> doors_;
};

}  // namespace ego_planner

#endif
