#include <exploration_manager/vla_swarm_map.h>

#include <cv_bridge/cv_bridge.h>
#include <scene_graph/scene_graph.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <utility>

namespace ego_planner {
namespace {

cv::Rect roomRectFromWorld(const VLASwarmRoom& room) {
  return room.pixel_box;
}

double pointDistance(const cv::Point& lhs, const cv::Point& rhs) {
  const double dx = static_cast<double>(lhs.x - rhs.x);
  const double dy = static_cast<double>(lhs.y - rhs.y);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

VLASwarmMap::VLASwarmMap(ros::NodeHandle& nh, const MapInterface::Ptr& map)
    : map_(map) {
  nh.param("vla_swarm/small_map_topic", image_topic_, image_topic_);
  nh.param("vla_swarm/small_map_frame", frame_id_, frame_id_);
  nh.param("vla_swarm/small_map_width", image_width_, image_width_);
  nh.param("vla_swarm/small_map_height", image_height_, image_height_);
  nh.param("vla_swarm/map_height_min", sample_height_min_, sample_height_min_);
  nh.param("vla_swarm/map_height_max", sample_height_max_, sample_height_max_);
  nh.param("vla_swarm/map_height_step", sample_height_step_, sample_height_step_);
  nh.param("vla_swarm/min_room_area_px", min_room_area_px_, min_room_area_px_);
  nh.param("vla_swarm/room_erode_iterations", room_erode_iterations_, room_erode_iterations_);
  nh.param("vla_swarm/door_min_width_px", door_min_width_px_, door_min_width_px_);
  nh.param("vla_swarm/door_max_width_px", door_max_width_px_, door_max_width_px_);
  nh.param("vla_swarm/door_merge_distance_px", door_merge_distance_px_, door_merge_distance_px_);
  nh.param("vla_swarm/astar_clearance_px", astar_clearance_px_, astar_clearance_px_);

  image_width_ = std::max(64, image_width_);
  image_height_ = std::max(64, image_height_);
  sample_height_step_ = std::max(0.1, sample_height_step_);
  room_erode_iterations_ = std::max(1, room_erode_iterations_);
  astar_clearance_px_ = std::max(0, astar_clearance_px_);
  image_pub_ = nh.advertise<sensor_msgs::Image>(image_topic_, 2, true);
}

bool VLASwarmMap::update(const Eigen::Vector3d& robot_position) {
  if (!map_ || !map_->isInited()) {
    return false;
  }

  Eigen::Vector3d map_min;
  Eigen::Vector3d map_max;
  map_->getGlobalBox(map_min, map_max);
  if (map_max.x() <= map_min.x() || map_max.y() <= map_min.y()) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_min_ = map_min;
    map_max_ = map_max;
    pixels_per_meter_x_ = static_cast<double>(image_width_ - 1) / (map_max_.x() - map_min_.x());
    pixels_per_meter_y_ = static_cast<double>(image_height_ - 1) / (map_max_.y() - map_min_.y());
  }

  cv::Mat map_image;
  cv::Mat free_mask;
  cv::Mat unknown_mask;
  buildOccupancyImage(map_image, free_mask, unknown_mask);

  cv::Mat room_labels;
  std::vector<VLASwarmRoom> new_rooms;
  segmentRooms(free_mask, room_labels, new_rooms);

  std::vector<VLASwarmDoor> new_doors;
  detectDoors(free_mask, unknown_mask, room_labels, new_doors);

  // 只保留与机器人处于同一自由空间连通域的候选。这里做二维地图级粗过滤，
  // 后续 EGO 阶段仍需使用三维地图完成真实路径可达性验证。
  cv::Point robot_pixel;
  if (worldToPixel(robot_position.head<2>(), robot_pixel)) {
    cv::Mat free_components;
    cv::connectedComponents(free_mask, free_components, 8, CV_32S);
    int robot_component = free_components.at<int>(robot_pixel.y, robot_pixel.x);
    if (robot_component == 0) {
      for (int radius = 1; radius <= 8 && robot_component == 0; ++radius) {
        for (int row = std::max(0, robot_pixel.y - radius);
             row <= std::min(free_components.rows - 1, robot_pixel.y + radius);
             ++row) {
          for (int col = std::max(0, robot_pixel.x - radius);
               col <= std::min(free_components.cols - 1, robot_pixel.x + radius);
               ++col) {
            const int component = free_components.at<int>(row, col);
            if (component > 0) {
              robot_component = component;
              break;
            }
          }
          if (robot_component > 0) {
            break;
          }
        }
      }
    }
    if (robot_component > 0) {
      new_doors.erase(
          std::remove_if(
              new_doors.begin(), new_doors.end(),
              [&](const VLASwarmDoor& door) {
                return free_components.at<int>(
                           door.pixel.y, door.pixel.x) != robot_component;
              }),
          new_doors.end());
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    assignStableRoomIds(new_rooms);

    // 房间稳定 ID 确定后，把门关联的临时标签转换成稳定房间 ID。
    for (auto& door : new_doors) {
      for (int& room_index : door.room_ids) {
        if (room_index >= 0 && room_index < static_cast<int>(new_rooms.size())) {
          room_index = new_rooms[room_index].id;
        }
      }
      std::sort(door.room_ids.begin(), door.room_ids.end());
      door.room_ids.erase(std::unique(door.room_ids.begin(), door.room_ids.end()),
                          door.room_ids.end());
    }
    assignStableDoorIds(new_doors);

    rooms_ = new_rooms;
    doors_ = new_doors;
    free_mask_ = free_mask;
    room_labels_ = room_labels;
    image_ = map_image;
    drawSemanticOverlay(image_, room_labels_, robot_position);
    ready_ = true;
    map_sequence_ = map_->getOccupancyUpdateSeq();
    map_image = image_.clone();
  }

  publishImage(map_image);
  return true;
}

void VLASwarmMap::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = false;
  map_sequence_ = 0;
  image_.release();
  free_mask_.release();
  room_labels_.release();
  rooms_.clear();
  doors_.clear();
}

bool VLASwarmMap::ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

uint64_t VLASwarmMap::mapSequence() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_sequence_;
}

cv::Mat VLASwarmMap::image() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return image_.clone();
}

std::vector<VLASwarmRoom> VLASwarmMap::rooms() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rooms_;
}

std::vector<VLASwarmDoor> VLASwarmMap::doors() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return doors_;
}

bool VLASwarmMap::findDoor(int door_id, VLASwarmDoor& door) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iterator = std::find_if(
      doors_.begin(), doors_.end(),
      [door_id](const VLASwarmDoor& candidate) {
        return candidate.id == door_id;
      });
  if (iterator == doors_.end()) {
    return false;
  }
  door = *iterator;
  return true;
}

bool VLASwarmMap::planDoorPath(
    const Eigen::Vector3d& start_position, int door_id, double flight_height,
    double waypoint_distance, std::vector<Eigen::Vector3d>& path) const {
  std::lock_guard<std::mutex> lock(mutex_);
  path.clear();
  if (!ready_ || free_mask_.empty() ||
      pixels_per_meter_x_ <= 0.0 || pixels_per_meter_y_ <= 0.0) {
    return false;
  }

  const auto door_iterator = std::find_if(
      doors_.begin(), doors_.end(),
      [door_id](const VLASwarmDoor& door) {
        return door.id == door_id;
      });
  if (door_iterator == doors_.end()) {
    return false;
  }

  cv::Mat traversable = free_mask_.clone();
  if (astar_clearance_px_ > 0) {
    const int kernel_size = astar_clearance_px_ * 2 + 1;
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::erode(traversable, traversable, kernel);
  }

  auto world_to_pixel = [&](const Eigen::Vector2d& world, cv::Point& pixel) {
    pixel.x = static_cast<int>(
        std::lround((world.x() - map_min_.x()) * pixels_per_meter_x_));
    pixel.y = static_cast<int>(
        std::lround((map_max_.y() - world.y()) * pixels_per_meter_y_));
    return pixel.x >= 0 && pixel.x < image_width_ &&
           pixel.y >= 0 && pixel.y < image_height_;
  };
  auto nearest_free = [&](cv::Point& pixel) {
    if (pixel.x >= 0 && pixel.x < traversable.cols &&
        pixel.y >= 0 && pixel.y < traversable.rows &&
        traversable.at<uchar>(pixel.y, pixel.x) != 0) {
      return true;
    }
    for (int radius = 1; radius <= 12; ++radius) {
      for (int row = std::max(0, pixel.y - radius);
           row <= std::min(traversable.rows - 1, pixel.y + radius); ++row) {
        for (int col = std::max(0, pixel.x - radius);
             col <= std::min(traversable.cols - 1, pixel.x + radius); ++col) {
          if (traversable.at<uchar>(row, col) != 0) {
            pixel = cv::Point(col, row);
            return true;
          }
        }
      }
    }
    return false;
  };

  cv::Point start_pixel;
  if (!world_to_pixel(start_position.head<2>(), start_pixel)) {
    return false;
  }
  cv::Point goal_pixel = door_iterator->pixel;
  if (!nearest_free(start_pixel) || !nearest_free(goal_pixel)) {
    return false;
  }

  struct SearchNode {
    double score;
    int index;
    bool operator>(const SearchNode& other) const {
      return score > other.score;
    }
  };
  const int width = traversable.cols;
  const int height = traversable.rows;
  const int cell_count = width * height;
  const int start_index = start_pixel.y * width + start_pixel.x;
  const int goal_index = goal_pixel.y * width + goal_pixel.x;
  std::vector<double> distance(
      cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<uint8_t> closed(cell_count, 0);
  std::priority_queue<SearchNode, std::vector<SearchNode>,
                      std::greater<SearchNode>> open;
  distance[start_index] = 0.0;
  open.push({pointDistance(start_pixel, goal_pixel), start_index});

  static const int kNeighborX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int kNeighborY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  while (!open.empty()) {
    const int current_index = open.top().index;
    open.pop();
    if (closed[current_index] != 0) {
      continue;
    }
    closed[current_index] = 1;
    if (current_index == goal_index) {
      break;
    }
    const cv::Point current(current_index % width, current_index / width);
    for (int direction = 0; direction < 8; ++direction) {
      const cv::Point next(
          current.x + kNeighborX[direction],
          current.y + kNeighborY[direction]);
      if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height ||
          traversable.at<uchar>(next.y, next.x) == 0) {
        continue;
      }
      if (kNeighborX[direction] != 0 && kNeighborY[direction] != 0 &&
          (traversable.at<uchar>(
               current.y, current.x + kNeighborX[direction]) == 0 ||
           traversable.at<uchar>(
               current.y + kNeighborY[direction], current.x) == 0)) {
        continue;
      }
      const int next_index = next.y * width + next.x;
      const bool cardinal =
          direction == 1 || direction == 3 ||
          direction == 4 || direction == 6;
      const double candidate_distance =
          distance[current_index] + (cardinal ? 1.0 : std::sqrt(2.0));
      if (candidate_distance >= distance[next_index]) {
        continue;
      }
      distance[next_index] = candidate_distance;
      parent[next_index] = current_index;
      open.push({
          candidate_distance + pointDistance(next, goal_pixel),
          next_index});
    }
  }
  if (start_index != goal_index && parent[goal_index] < 0) {
    return false;
  }

  std::vector<cv::Point> pixel_path;
  for (int index = goal_index; index >= 0; index = parent[index]) {
    pixel_path.emplace_back(index % width, index / width);
    if (index == start_index) {
      break;
    }
  }
  std::reverse(pixel_path.begin(), pixel_path.end());
  if (pixel_path.size() == 1) {
    Eigen::Vector3d goal(
        door_iterator->position.x(), door_iterator->position.y(),
        flight_height);
    path = {start_position, goal};
    return true;
  }

  // 通过栅格视线检测删除无意义折点，降低下游 EGO 的目标切换频率。
  auto line_is_free = [&](const cv::Point& begin, const cv::Point& end) {
    cv::LineIterator iterator(traversable, begin, end, 8);
    for (int index = 0; index < iterator.count; ++index, ++iterator) {
      const cv::Point point = iterator.pos();
      if (traversable.at<uchar>(point.y, point.x) == 0) {
        return false;
      }
    }
    return true;
  };
  std::vector<cv::Point> simplified_path;
  simplified_path.push_back(pixel_path.front());
  size_t anchor = 0;
  while (anchor + 1 < pixel_path.size()) {
    size_t next = pixel_path.size() - 1;
    while (next > anchor + 1 &&
           !line_is_free(pixel_path[anchor], pixel_path[next])) {
      --next;
    }
    simplified_path.push_back(pixel_path[next]);
    anchor = next;
  }

  std::vector<Eigen::Vector3d> world_path;
  world_path.reserve(simplified_path.size());
  for (const cv::Point& pixel : simplified_path) {
    const double world_x =
        map_min_.x() + static_cast<double>(pixel.x) / pixels_per_meter_x_;
    const double world_y =
        map_max_.y() - static_cast<double>(pixel.y) / pixels_per_meter_y_;
    world_path.emplace_back(world_x, world_y, flight_height);
  }
  world_path.front() = start_position;
  world_path.back().x() = door_iterator->position.x();
  world_path.back().y() = door_iterator->position.y();

  // 按世界距离重新采样，保证连续局部目标之间的间距稳定。
  const double sample_distance = std::max(0.2, waypoint_distance);
  path.push_back(world_path.front());
  for (size_t index = 1; index < world_path.size(); ++index) {
    const Eigen::Vector3d segment = world_path[index] - world_path[index - 1];
    const double segment_length = segment.norm();
    if (segment_length <= 1e-6) {
      continue;
    }
    for (double distance_along = sample_distance;
         distance_along < segment_length; distance_along += sample_distance) {
      path.push_back(
          world_path[index - 1] +
          segment * (distance_along / segment_length));
    }
    if ((path.back() - world_path[index]).norm() > 1e-3) {
      path.push_back(world_path[index]);
    }
  }
  return path.size() >= 2;
}

nlohmann::json VLASwarmMap::promptContext(
    const SceneGraph& scene_graph, const Eigen::Vector3d& robot_position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  nlohmann::json context;
  context["map_ready"] = ready_;
  context["small_map_topic"] = image_topic_;
  context["map_sequence"] = map_sequence_;
  context["robot_position"] = {robot_position.x(), robot_position.y(), robot_position.z()};
  context["explored_rooms"] = nlohmann::json::array();
  context["detected_objects"] = nlohmann::json::array();
  context["doors"] = nlohmann::json::array();
  context["candidate_ids"] = nlohmann::json::array();
  context["room_descriptions"] = nlohmann::json::array();

  for (const auto& room : rooms_) {
    std::string semantic_description = room.description;
    nlohmann::json scene_area_ids = nlohmann::json::array();
    if (scene_graph.skeleton_gen_ &&
        scene_graph.skeleton_gen_->area_handler_) {
      for (const auto& area_pair :
           scene_graph.skeleton_gen_->area_handler_->area_map_) {
        const auto& area = area_pair.second;
        if (!area ||
            area->center_.x() < room.world_min.x() ||
            area->center_.x() > room.world_max.x() ||
            area->center_.y() < room.world_min.y() ||
            area->center_.y() > room.world_max.y()) {
          continue;
        }
        scene_area_ids.push_back(area_pair.first);
        const std::string area_semantic =
            !area->room_description_.empty()
                ? area->room_description_
                : area->room_label_;
        if (!area_semantic.empty() && area_semantic != "Unknown") {
          if (!semantic_description.empty()) {
            semantic_description += "; ";
          }
          semantic_description += area_semantic;
        }
      }
    }

    nlohmann::json room_json;
    room_json["room_id"] = room.id;
    room_json["room_description"] =
        semantic_description.empty() ? "unknown room" : semantic_description;
    room_json["center"] = {room.center.x(), room.center.y()};
    room_json["size"] = {
        room.world_max.x() - room.world_min.x(),
        room.world_max.y() - room.world_min.y()};
    room_json["scene_area_ids"] = scene_area_ids;
    context["explored_rooms"].push_back(room_json);
    context["room_descriptions"].push_back(room_json);
  }

  for (const auto& door : doors_) {
    nlohmann::json door_json;
    door_json["door_id"] = door.id;
    door_json["position"] = {door.position.x(), door.position.y()};
    door_json["frontier"] = door.frontier;
    door_json["room_ids"] = door.room_ids;
    context["doors"].push_back(door_json);
    context["candidate_ids"].push_back(door.id);
  }

  // 复用现有 SceneGraph 语义对象，按世界坐标直接写入 PLACE Prompt。
  if (scene_graph.object_factory_) {
    for (const auto& object_pair : scene_graph.object_factory_->object_map_) {
      const auto& object = object_pair.second;
      if (!object) {
        continue;
      }
      int room_id = -1;
      for (const auto& room : rooms_) {
        if (object->pos.x() >= room.world_min.x() &&
            object->pos.x() <= room.world_max.x() &&
            object->pos.y() >= room.world_min.y() &&
            object->pos.y() <= room.world_max.y()) {
          room_id = room.id;
          break;
        }
      }
      context["detected_objects"].push_back({
          {"object_id", object->id},
          {"object_description", object->label},
          {"room_id", room_id},
          {"position", {object->pos.x(), object->pos.y(), object->pos.z()}}});
    }
  }

  return context;
}

bool VLASwarmMap::worldToPixel(const Eigen::Vector2d& world, cv::Point& pixel) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pixels_per_meter_x_ <= 0.0 || pixels_per_meter_y_ <= 0.0) {
    return false;
  }
  pixel.x = static_cast<int>(std::lround((world.x() - map_min_.x()) * pixels_per_meter_x_));
  pixel.y = static_cast<int>(std::lround((map_max_.y() - world.y()) * pixels_per_meter_y_));
  return pixel.x >= 0 && pixel.x < image_width_ &&
         pixel.y >= 0 && pixel.y < image_height_;
}

bool VLASwarmMap::pixelToWorld(const cv::Point& pixel, Eigen::Vector2d& world) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pixel.x < 0 || pixel.x >= image_width_ ||
      pixel.y < 0 || pixel.y >= image_height_ ||
      pixels_per_meter_x_ <= 0.0 || pixels_per_meter_y_ <= 0.0) {
    return false;
  }
  world.x() = map_min_.x() + static_cast<double>(pixel.x) / pixels_per_meter_x_;
  world.y() = map_max_.y() - static_cast<double>(pixel.y) / pixels_per_meter_y_;
  return true;
}

void VLASwarmMap::buildOccupancyImage(
    cv::Mat& image, cv::Mat& free_mask, cv::Mat& unknown_mask) const {
  image = cv::Mat(image_height_, image_width_, CV_8UC3, cv::Scalar(255, 255, 255));
  free_mask = cv::Mat(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
  unknown_mask = cv::Mat(image_height_, image_width_, CV_8UC1, cv::Scalar(0));

  map_->Lock();
  for (int row = 0; row < image_height_; ++row) {
    const double world_y =
        map_max_.y() - static_cast<double>(row) / pixels_per_meter_y_;
    for (int col = 0; col < image_width_; ++col) {
      const double world_x =
          map_min_.x() + static_cast<double>(col) / pixels_per_meter_x_;
      bool occupied = false;
      bool free = false;
      for (double z = sample_height_min_; z <= sample_height_max_ + 1e-6;
           z += sample_height_step_) {
        const int occupancy = map_->getOccupancy(Eigen::Vector3d(world_x, world_y, z));
        occupied = occupied || occupancy == MapInterface::OCCUPIED;
        free = free || occupancy == MapInterface::FREE;
      }
      if (occupied) {
        image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
      } else if (free) {
        image.at<cv::Vec3b>(row, col) = cv::Vec3b(230, 128, 128);
        free_mask.at<uchar>(row, col) = 255;
      } else {
        unknown_mask.at<uchar>(row, col) = 255;
      }
    }
  }
  map_->Unlock();

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(free_mask, free_mask, cv::MORPH_CLOSE, kernel);
}

void VLASwarmMap::segmentRooms(
    const cv::Mat& free_mask, cv::Mat& room_labels,
    std::vector<VLASwarmRoom>& rooms) {
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat seeds;
  cv::erode(free_mask, seeds, kernel, cv::Point(-1, -1), room_erode_iterations_);

  cv::Mat seed_labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int label_count =
      cv::connectedComponentsWithStats(seeds, seed_labels, stats, centroids, 8, CV_32S);

  room_labels = cv::Mat(free_mask.size(), CV_32S, cv::Scalar(0));
  std::queue<cv::Point> queue;
  std::vector<int> seed_to_room(label_count, -1);
  for (int label = 1; label < label_count; ++label) {
    if (stats.at<int>(label, cv::CC_STAT_AREA) < min_room_area_px_) {
      continue;
    }
    seed_to_room[label] = static_cast<int>(rooms.size());
    rooms.emplace_back();
  }

  if (rooms.empty()) {
    // 窄走廊或小型环境可能在侵蚀后没有种子，此时退化为自由空间连通域，
    // 保证 SmallMap 仍然能够提供至少一个可描述区域。
    cv::Mat fallback_labels;
    const int fallback_count = cv::connectedComponentsWithStats(
        free_mask, fallback_labels, stats, centroids, 8, CV_32S);
    for (int label = 1; label < fallback_count; ++label) {
      if (stats.at<int>(label, cv::CC_STAT_AREA) < min_room_area_px_) {
        continue;
      }
      rooms.emplace_back();
      room_labels.setTo(
          static_cast<int>(rooms.size()), fallback_labels == label);
    }
  } else {
    for (int row = 0; row < seed_labels.rows; ++row) {
      for (int col = 0; col < seed_labels.cols; ++col) {
        const int label = seed_labels.at<int>(row, col);
        if (label > 0 && seed_to_room[label] >= 0) {
          room_labels.at<int>(row, col) = seed_to_room[label] + 1;
          queue.emplace(col, row);
        }
      }
    }

    const cv::Point directions[] = {
        cv::Point(1, 0), cv::Point(-1, 0), cv::Point(0, 1), cv::Point(0, -1),
        cv::Point(1, 1), cv::Point(-1, 1), cv::Point(1, -1), cv::Point(-1, -1)};
    while (!queue.empty()) {
      const cv::Point current = queue.front();
      queue.pop();
      const int label = room_labels.at<int>(current.y, current.x);
      for (const auto& direction : directions) {
        const cv::Point next = current + direction;
        if (next.x < 0 || next.x >= free_mask.cols ||
            next.y < 0 || next.y >= free_mask.rows ||
            free_mask.at<uchar>(next.y, next.x) == 0 ||
            room_labels.at<int>(next.y, next.x) != 0) {
          continue;
        }
        room_labels.at<int>(next.y, next.x) = label;
        queue.push(next);
      }
    }
  }

  for (int room_index = 0; room_index < static_cast<int>(rooms.size()); ++room_index) {
    std::vector<cv::Point> pixels;
    cv::findNonZero(room_labels == room_index + 1, pixels);
    if (pixels.empty()) {
      continue;
    }
    VLASwarmRoom& room = rooms[room_index];
    room.pixel_box = cv::boundingRect(pixels);
    room.area = static_cast<double>(pixels.size()) /
                (pixels_per_meter_x_ * pixels_per_meter_y_);
    pixelToWorld(
        cv::Point(room.pixel_box.x, room.pixel_box.y + room.pixel_box.height - 1),
        room.world_min);
    pixelToWorld(
        cv::Point(room.pixel_box.x + room.pixel_box.width - 1, room.pixel_box.y),
        room.world_max);
    room.center = 0.5 * (room.world_min + room.world_max);
  }
}

void VLASwarmMap::detectDoors(
    const cv::Mat& free_mask, const cv::Mat& unknown_mask,
    const cv::Mat& room_labels, std::vector<VLASwarmDoor>& doors) {
  std::vector<DoorCandidate> candidates;

  // 不同房间标签在自由空间中的接触带表示房间间开口。
  cv::Mat room_boundary(free_mask.size(), CV_8UC1, cv::Scalar(0));
  for (int row = 1; row < room_labels.rows - 1; ++row) {
    for (int col = 1; col < room_labels.cols - 1; ++col) {
      const int label = room_labels.at<int>(row, col);
      if (label <= 0) {
        continue;
      }
      std::set<int> neighboring_labels;
      neighboring_labels.insert(label);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int other = room_labels.at<int>(row + dy, col + dx);
          if (other > 0) {
            neighboring_labels.insert(other);
          }
        }
      }
      if (neighboring_labels.size() > 1) {
        room_boundary.at<uchar>(row, col) = 255;
      }
    }
  }

  auto collectComponents = [&](const cv::Mat& mask, bool frontier) {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count =
        cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    for (int label = 1; label < count; ++label) {
      const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      const int span = std::max(width, height);
      if (span < door_min_width_px_ ||
          (!frontier && span > door_max_width_px_)) {
        continue;
      }
      DoorCandidate candidate;
      candidate.pixel = cv::Point(
          static_cast<int>(std::lround(centroids.at<double>(label, 0))),
          static_cast<int>(std::lround(centroids.at<double>(label, 1))));
      if (frontier && span > door_max_width_px_) {
        // 大片未知边界通常是一个连通分量，质心可能落在边界内部。
        // 选择离质心最近的真实 frontier 像素，保留一个稳定探索候选。
        std::vector<cv::Point> component_pixels;
        cv::findNonZero(labels == label, component_pixels);
        double nearest_distance = std::numeric_limits<double>::max();
        for (const auto& pixel : component_pixels) {
          const double distance = pointDistance(pixel, candidate.pixel);
          if (distance < nearest_distance) {
            nearest_distance = distance;
            candidate.pixel = pixel;
          }
        }
      }
      candidate.frontier = frontier;
      std::set<int> room_indices;
      for (int row = std::max(0, candidate.pixel.y - 2);
           row <= std::min(room_labels.rows - 1, candidate.pixel.y + 2); ++row) {
        for (int col = std::max(0, candidate.pixel.x - 2);
             col <= std::min(room_labels.cols - 1, candidate.pixel.x + 2); ++col) {
          const int room_label = room_labels.at<int>(row, col);
          if (room_label > 0) {
            room_indices.insert(room_label - 1);
          }
        }
      }
      candidate.room_ids.assign(room_indices.begin(), room_indices.end());
      candidates.push_back(candidate);
    }
  };
  collectComponents(room_boundary, false);

  // 自由空间与未知空间的窄接触段作为待探索 frontier 门。
  cv::Mat dilated_unknown;
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(unknown_mask, dilated_unknown, kernel);
  cv::Mat frontier_mask;
  cv::bitwise_and(free_mask, dilated_unknown, frontier_mask);
  collectComponents(frontier_mask, true);

  for (const auto& candidate : candidates) {
    bool merged = false;
    for (auto& door : doors) {
      if (pointDistance(door.pixel, candidate.pixel) <= door_merge_distance_px_) {
        door.pixel.x = (door.pixel.x + candidate.pixel.x) / 2;
        door.pixel.y = (door.pixel.y + candidate.pixel.y) / 2;
        door.frontier = door.frontier || candidate.frontier;
        door.room_ids.insert(
            door.room_ids.end(), candidate.room_ids.begin(), candidate.room_ids.end());
        merged = true;
        break;
      }
    }
    if (!merged) {
      VLASwarmDoor door;
      door.pixel = candidate.pixel;
      door.frontier = candidate.frontier;
      door.room_ids = candidate.room_ids;
      pixelToWorld(door.pixel, door.position);
      doors.push_back(door);
    }
  }

  // 合并会改变像素中心，最终统一刷新世界坐标及关联房间集合。
  for (auto& door : doors) {
    std::sort(door.room_ids.begin(), door.room_ids.end());
    door.room_ids.erase(
        std::unique(door.room_ids.begin(), door.room_ids.end()),
        door.room_ids.end());
    pixelToWorld(door.pixel, door.position);
  }
}

void VLASwarmMap::assignStableRoomIds(std::vector<VLASwarmRoom>& rooms) {
  std::set<int> used_ids;
  for (auto& room : rooms) {
    double best_overlap = 0.0;
    int best_id = -1;
    for (const auto& old_room : rooms_) {
      if (used_ids.count(old_room.id) > 0) {
        continue;
      }
      const double overlap =
          rectangleOverlapRatio(roomRectFromWorld(room), roomRectFromWorld(old_room));
      if (overlap > 0.55 && overlap > best_overlap) {
        best_overlap = overlap;
        best_id = old_room.id;
        room.description = old_room.description;
      }
    }
    room.id = best_id >= 0 ? best_id : next_room_id_++;
    used_ids.insert(room.id);
  }
}

void VLASwarmMap::assignStableDoorIds(std::vector<VLASwarmDoor>& doors) {
  std::set<int> used_ids;
  for (auto& door : doors) {
    double best_distance = std::numeric_limits<double>::max();
    int best_id = -1;
    for (const auto& old_door : doors_) {
      if (used_ids.count(old_door.id) > 0 || door.frontier != old_door.frontier) {
        continue;
      }
      const double distance = (door.position - old_door.position).norm();
      if (distance < 0.8 && distance < best_distance) {
        best_distance = distance;
        best_id = old_door.id;
      }
    }
    door.id = best_id >= 0 ? best_id : next_door_id_++;
    used_ids.insert(door.id);
  }
}

void VLASwarmMap::drawSemanticOverlay(
    cv::Mat& image, const cv::Mat&, const Eigen::Vector3d& robot_position) const {
  for (const auto& room : rooms_) {
    cv::rectangle(image, room.pixel_box, cv::Scalar(0, 180, 0), 1);
    cv::putText(
        image, roomLabel(room.id),
        cv::Point(room.pixel_box.x + room.pixel_box.width / 2,
                  room.pixel_box.y + room.pixel_box.height / 2),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
  }
  for (const auto& door : doors_) {
    const cv::Scalar color = cv::Scalar(0, 255, 0);
    cv::rectangle(
        image, door.pixel - cv::Point(5, 5),
        door.pixel + cv::Point(5, 5), color, -1);
    cv::putText(
        image, std::to_string(door.id), door.pixel + cv::Point(5, -5),
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
  }
  cv::Point robot_pixel;
  robot_pixel.x = static_cast<int>(std::lround(
      (robot_position.x() - map_min_.x()) * pixels_per_meter_x_));
  robot_pixel.y = static_cast<int>(std::lround(
      (map_max_.y() - robot_position.y()) * pixels_per_meter_y_));
  if (robot_pixel.x >= 0 && robot_pixel.x < image_width_ &&
      robot_pixel.y >= 0 && robot_pixel.y < image_height_) {
    cv::drawMarker(
        image, robot_pixel, cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS,
        14, 2, cv::LINE_AA);
  }
}

void VLASwarmMap::publishImage(const cv::Mat& image) const {
  if (image.empty()) {
    return;
  }
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;
  image_pub_.publish(cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
}

double VLASwarmMap::rectangleOverlapRatio(
    const cv::Rect& lhs, const cv::Rect& rhs) {
  const cv::Rect intersection = lhs & rhs;
  const double smaller_area = std::min(lhs.area(), rhs.area());
  return smaller_area > 0.0
             ? static_cast<double>(intersection.area()) / smaller_area
             : 0.0;
}

std::string VLASwarmMap::roomLabel(int id) {
  int value = id + 1;
  std::string label;
  while (value > 0) {
    label.push_back(static_cast<char>('A' + (value - 1) % 26));
    value = (value - 1) / 26;
  }
  std::reverse(label.begin(), label.end());
  return label;
}

}  // namespace ego_planner
