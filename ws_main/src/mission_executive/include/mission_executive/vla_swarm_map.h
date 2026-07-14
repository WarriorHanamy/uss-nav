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
 * Single-robot 2D semantic mapping module for VLA Swarm.
 *
 * Reads the local MapInterface to produce a small-map image, room/door
 * segmentation, and a prompt JSON for LLM-based task planning. All
 * world-to-pixel conversions are shared so downstream path planning
 * can reuse world coordinates directly.
 */
class VLASwarmMap {
 public:
  using Ptr = std::shared_ptr<VLASwarmMap>;

  /**
   * Construct the VLA swarm map with ROS node handle and map interface.
   *
   * @param[in] nh   ROS node handle
   * @param[in] map  Map interface pointer
   */
  VLASwarmMap(ros::NodeHandle& nh, const MapInterface::Ptr& map);
  /**
   * Update the 2D semantic map from the current occupancy grid.
   *
   * @param[in] robot_position  Current robot position [m]
   * @return True if the map was updated
   */
  bool update(const Eigen::Vector3d& robot_position);
  /**
   * Reset the semantic map state.
   */
  void reset();
  /**
   * Check whether a valid semantic map is ready.
   *
   * @return True if the map is initialized
   */
  bool ready() const;
  /**
   * Get the current map sequence number.
   *
   * @return Sequence counter
   */
  uint64_t mapSequence() const;
  /**
   * Get the small-map image with semantic overlay.
   *
   * @return CV image
   */
  cv::Mat image() const;
  /**
   * Get the list of detected rooms.
   *
   * @return Room list
   */
  std::vector<VLASwarmRoom> rooms() const;
  /**
   * Get the list of detected doors.
   *
   * @return Door list
   */
  std::vector<VLASwarmDoor> doors() const;
  /**
   * Find a door by its stable ID.
   *
   * @param[in]  door_id  Door ID
   * @param[out] door     Found door (output)
   * @return True if the door was found
   */
  bool findDoor(int door_id, VLASwarmDoor& door) const;
  /**
   * Plan a multi-waypoint path through a specified door.
   *
   * @param[in]  start_position    Start position [m]
   * @param[in]  door_id           Target door ID
   * @param[in]  flight_height     Flight height [m]
   * @param[in]  waypoint_distance Step distance between waypoints [m]
   * @param[out] path              Planned waypoints [m]
   * @return True if a valid path was planned
   */
  bool planDoorPath(const Eigen::Vector3d& start_position, int door_id,
                    double flight_height, double waypoint_distance,
                    std::vector<Eigen::Vector3d>& path) const;
  /**
   * Build the prompt JSON context for LLM-based scene reasoning.
   *
   * @param[in] scene_graph     Scene graph for object-level context
   * @param[in] robot_position  Current robot position [m]
   * @return JSON prompt payload
   */
  nlohmann::json promptContext(const SceneGraph& scene_graph,
                               const Eigen::Vector3d& robot_position) const;
  /**
   * Convert world coordinates to pixel coordinates.
   *
   * @param[in]  world  World position [m]
   * @param[out] pixel  Pixel position [px]
   * @return True if the point is within the map bounds
   */
  bool worldToPixel(const Eigen::Vector2d& world, cv::Point& pixel) const;
  /**
   * Convert pixel coordinates to world coordinates.
   *
   * @param[in]  pixel  Pixel position [px]
   * @param[out] world  World position [m]
   * @return True if conversion succeeded
   */
  bool pixelToWorld(const cv::Point& pixel, Eigen::Vector2d& world) const;

 private:
  struct DoorCandidate {
    cv::Point pixel;
    bool frontier{false};
    std::vector<int> room_ids;
  };

  /**
   * Build occupancy images (free mask, unknown mask) from the map.
   *
   * @param[out] image         Colored output image
   * @param[out] free_mask     Binary mask of free voxels
   * @param[out] unknown_mask  Binary mask of unknown voxels
   */
  void buildOccupancyImage(cv::Mat& image, cv::Mat& free_mask,
                           cv::Mat& unknown_mask) const;
  /**
   * Segment rooms from the free-space mask via connected components.
   *
   * @param[in]  free_mask   Binary free-space mask
   * @param[out] room_labels Labeled room image
   * @param[out] rooms       Detected room list
   */
  void segmentRooms(const cv::Mat& free_mask, cv::Mat& room_labels,
                    std::vector<VLASwarmRoom>& rooms);
  /**
   * Detect door positions at the boundary between free and unknown space.
   *
   * @param[in]  free_mask    Binary free-space mask
   * @param[in]  unknown_mask Binary unknown mask
   * @param[in]  room_labels  Labeled room image
   * @param[out] doors        Detected door list
   */
  void detectDoors(const cv::Mat& free_mask, const cv::Mat& unknown_mask,
                   const cv::Mat& room_labels,
                   std::vector<VLASwarmDoor>& doors);
  /**
   * Assign temporally stable IDs to rooms by matching with previous frame.
   *
   * @param[in,out] rooms  Room list with IDs updated in-place
   */
  void assignStableRoomIds(std::vector<VLASwarmRoom>& rooms);
  /**
   * Assign temporally stable IDs to doors by matching with previous frame.
   *
   * @param[in,out] doors  Door list with IDs updated in-place
   */
  void assignStableDoorIds(std::vector<VLASwarmDoor>& doors);
  /**
   * Draw room labels, door markers, and robot position on the image.
   *
   * @param[in,out] image           Image to draw on
   * @param[in]     room_labels     Labeled room image
   * @param[in]     robot_position  Robot position for marker [m]
   */
  void drawSemanticOverlay(cv::Mat& image, const cv::Mat& room_labels,
                           const Eigen::Vector3d& robot_position) const;
  /**
   * Publish the semantic map image on the ROS topic.
   *
   * @param[in] image  Image to publish
   */
  void publishImage(const cv::Mat& image) const;
  /**
   * Compute the overlap ratio (IoU-like) between two rectangles.
   *
   * @param[in] lhs  First rectangle [px]
   * @param[in] rhs  Second rectangle [px]
   * @return Overlap ratio [--]
   */
  static double rectangleOverlapRatio(const cv::Rect& lhs, const cv::Rect& rhs);
  /**
   * Generate a human-readable label for a room given its ID.
   *
   * @param[in] id  Room ID
   * @return Label string (e.g. "Room A")
   */
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
