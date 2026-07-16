//
// Created by gwq on 25-2-27.
//
#include <memory>
#include "../include/scene_graph/skeleton_generation.h"

void SkeletonGenerator::getROSParams(){
  readParam("skeleton/local_x_max",                            _local_x_max, 4.0);
  readParam("skeleton/local_x_min",                            _local_x_min, -4.0);
  readParam("skeleton/local_y_max",                            _local_y_max, 4.0);
  readParam("skeleton/local_y_min",                            _local_y_min, -4.0);
  readParam("skeleton/local_z_max",                            _local_z_max, 1.5);
  readParam("skeleton/local_z_min",                            _local_z_min, -0.8);

  if (_local_x_max <= _local_x_min || _local_y_max <= _local_y_min || _local_z_max <= _local_z_min){
    ROS_FATAL("[SkeletonGen] | Local range is not valid, please check the parameters !!!");
  }

  readParam("skeleton/map_type",                               _map_type, 1);
  readParam("skeleton/is_simulation",                          _is_simulation, true);
  readParam("skeleton/frontier_creation_threshold",            _frontier_creation_threshold, 2.0);
  readParam("skeleton/frontier_jump_threshold",                _frontier_jump_threshold, 2.0);
  readParam("skeleton/frontier_split_threshold",               _frontier_split_threshold, 2.0);
  readParam("skeleton/min_loop_creation_threshold",            _min_flowback_creation_threshold, 5);
  readParam("skeleton/min_flowback_creation_radius_threshold", _min_flowback_creation_radius_threshold, 1.0);
  readParam("skeleton/min_node_radius",                        _min_node_radius, 2.0);
  readParam("skeleton/min_node_dense_radius",                  _min_node_dense_radius, 0.5);
  readParam("skeleton/search_margin",                          _search_margin, 0.2);
  readParam("skeleton/max_ray_length",                         _max_ray_length, 8.0);
  readParam("skeleton/max_expansion_ray_length",               _max_expansion_ray_length, 5.0);
  readParam("skeleton/max_height_diff",                        _max_height_diff, 5.0);
  readParam("skeleton/sampling_density",                       _sampling_density, 16);
  readParam("skeleton/sampling_level",                         _sampling_level, 4);
  readParam("skeleton/max_facets_grouped",                     _max_facets_grouped, 6);
  readParam("skeleton/resolution",                             _resolution, 0.2);
  readParam("skeleton/truncated_z_high",                       _truncated_z_high, 2.5);
  readParam("skeleton/truncated_z_low",                        _truncated_z_low, 0.0);
  readParam("skeleton/debug_mode",                             _debug_mode, true);
  readParam("skeleton/bad_loop",                               _bad_loop, true);
  readParam("skeleton/expand_time_limit",                      _expand_time_limit, 50.0);

  readParam("skeleton/visualize_final_result_only",            _visualize_final_result_only, true);
  readParam("skeleton/visualize_all",                          _visualize_all, true);
  readParam("skeleton/visualize_outwards_normal",              _visualize_outwards_normal, false);
  readParam("skeleton/visualize_nbhd_frontiers",               _visualize_nbhd_facets, false);
  readParam("skeleton/visualize_black_polygon",                _visualize_black_polygon, false);
}

SkeletonGenerator::SkeletonGenerator(ros::NodeHandle& nh, global_belief::MapInterface::Ptr& map_interface){
  nh_ = nh;
  map_interface_    = map_interface;
  skeleton_astar_   = std::make_shared<skeleton_astar::SkeletonAstar>(nh);
  spectral_cluster_ = std::make_shared<SpectralCluster>(nh, 1.0);
  area_handler_  = std::make_shared<AreaHandler>(nh);
  skeleton_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("skeleton_vis", 100, true);
  cmd_sub_          = nh_.subscribe("/skeleton_cmd", 1, &SkeletonGenerator::cmdCallback, this);

  getROSParams();
  _local_range_max  = Eigen::Vector3d(_local_x_max, _local_y_max, _local_z_max);
  _local_range_min  = Eigen::Vector3d(_local_x_min, _local_y_min, _local_z_min);

  polyhedron_kd_tree_              = std::make_shared<KD_TREE<skeleton_gen::ikdTree_PolyhedronType>>();
  polyhedron_kd_tree_fixed_center_ = std::make_shared<KD_TREE<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>>();
  has_init_polyhedron_kdtree_      = false;
  mount_polyhedron_                = nullptr;
  last_mount_polyhedron_           = nullptr;
}

SkeletonGenerator::~SkeletonGenerator() = default;

// ----------------- Main Functions ----------------- //

void SkeletonGenerator::cmdCallback(const std_msgs::Empty::ConstPtr& msg){
  INFO_MSG("\n[SkeletonGen] | Received Skeleton Command\n");
  std::vector<Eigen::Vector3d> path;
  double distance = 0;
  distance = astarSearch(cur_pos_, Eigen::Vector3d(4.0, 0.0, 1.0), path, false);
  if (distance > 0.0)
    skeleton_astar_->visualizePath();
  else
    INFO_MSG_RED("[SkeletonGen] | A* search failed!");
}

bool SkeletonGenerator::ready() const{
  if (!has_init_polyhedron_kdtree_) return false;
  return polyhedron_kd_tree_->validnum() > 0;
}

int SkeletonGenerator::getNodeNum() const {
  return polyhedron_map_.size();
}

void SkeletonGenerator::resetForMapLoad() {
  polyhedron_map_.clear();
  cur_iter_polys_.clear();
  cur_iter_first_poly_ = nullptr;
  mount_polyhedron_ = nullptr;
  last_mount_polyhedron_ = nullptr;
  expand_pending_frontiers_.clear();
  remain_candidate_facet_expand_pts_.clear();
  polyhedron_kd_tree_ = std::make_shared<KD_TREE<skeleton_gen::ikdTree_PolyhedronType>>();
  polyhedron_kd_tree_fixed_center_ = std::make_shared<KD_TREE<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>>();
  remain_candidate_facet_expand_kd_tree_ = std::make_shared<KD_TREE<skeleton_gen::ikdTree_Vectoe3dType>>();
  has_init_polyhedron_kdtree_ = false;
  INFO_MSG_BLUE("[SkeletonGen] | Reset runtime state for map load.");
}

bool SkeletonGenerator::registerLoadedPolyhedron(const PolyHedronPtr& polyhedron) {
  if (polyhedron == nullptr) return false;
  if (polyhedron_map_.find(polyhedron->origin_center_) != polyhedron_map_.end()) {
    if (polyhedron->is_gate_) {
      INFO_MSG_YELLOW("[SkeletonGen] | Skip duplicate gate during map load, origin center: " << polyhedron->origin_center_.transpose());
      return true;
    }
    INFO_MSG_RED("[SkeletonGen] | Duplicate polyhedron during map load, origin center: " << polyhedron->origin_center_.transpose());
    return false;
  }

  polyhedron_map_[polyhedron->origin_center_] = polyhedron;
  PolyHedronKDTreeVector add_temp;
  add_temp.emplace_back(polyhedron);
  PolyHedronKDTree_FixedCenterVector add_temp_fixed_center;
  add_temp_fixed_center.emplace_back(polyhedron);

  if (has_init_polyhedron_kdtree_) {
    polyhedron_kd_tree_->Add_Points(add_temp, false);
    polyhedron_kd_tree_fixed_center_->Add_Points(add_temp_fixed_center, false);
  } else {
    polyhedron_kd_tree_->Build(add_temp);
    polyhedron_kd_tree_fixed_center_->Build(add_temp_fixed_center);
    has_init_polyhedron_kdtree_ = true;
  }
  return true;
}

PolyHedronPtr SkeletonGenerator::insertPolyhedronAt(const Eigen::Vector3d &center) {
    auto new_poly = std::make_shared<Polyhedron>(center, nullptr, false);
    new_poly->origin_center_ = center;

    polyhedron_map_[center] = new_poly;
    PolyHedronKDTreeVector add_temp;
    add_temp.emplace_back(new_poly);
    PolyHedronKDTree_FixedCenterVector add_temp_fixed;
    add_temp_fixed.emplace_back(new_poly);

    if (has_init_polyhedron_kdtree_) {
        polyhedron_kd_tree_->Add_Points(add_temp, false);
        polyhedron_kd_tree_fixed_center_->Add_Points(add_temp_fixed, false);
    } else {
        polyhedron_kd_tree_->Build(add_temp);
        polyhedron_kd_tree_fixed_center_->Build(add_temp_fixed);
        has_init_polyhedron_kdtree_ = true;
    }
    return new_poly;
}

void SkeletonGenerator::connectToVisibleNeighbors(const PolyHedronPtr &poly, double radius) {
    if (map_interface_ == nullptr || poly == nullptr) return;
    skeleton_gen::ikdTree_PolyhedronType_FixedCenter search_pt(poly);
    PolyHedronKDTree_FixedCenterVector nearby;
    polyhedron_kd_tree_fixed_center_->Radius_Search(search_pt, static_cast<double>(radius), nearby);

    for (auto &pt : nearby) {
        auto &neighbor = pt.polyhedron_;
        if (neighbor == poly || neighbor == nullptr) continue;
        if (neighbor->nav_blocked_) continue;
        // Strict visibility: no obstacle between two node centers (avoid edges through walls)
        if (!map_interface_->isVisible(poly->center_, neighbor->center_)) continue;

        double len = (poly->center_ - neighbor->center_).norm();
        Edge e1(neighbor, len);
        poly->edges_.push_back(e1);
        Edge e2(poly, len);
        neighbor->edges_.push_back(e2);
    }
}

void SkeletonGenerator::finishMapLoad() {
  cur_iter_polys_.clear();
  cur_iter_first_poly_ = nullptr;
  mount_polyhedron_ = nullptr;
  last_mount_polyhedron_ = nullptr;
  INFO_MSG_GREEN("[SkeletonGen] | Finish map load. Polyhedrons: " << polyhedron_map_.size());
}

double SkeletonGenerator::astarSearch(const PolyHedronPtr start_polyhedron, const PolyHedronPtr end_polyhedron,
                                      std::vector<Eigen::Vector3d> &path) {
  skeleton_astar_->astarSearch(start_polyhedron, end_polyhedron);
  skeleton_astar_->getPath(path);
  // print search info
  // INFO_MSG("[SkeletonGen] | A* search from polyhedron: " << start_polyhedron->center_.transpose() << " to polyhedron: " << end_polyhedron->center_.transpose());
  // INFO_MSG("              | A* search path length: " << path.size());
  if (!path.empty()) {
    // INFO_MSG("              | path front :" << path.front().transpose() << " back: " << path.back().transpose());
    // make sure path include start / end points
    if ((path.back() - end_polyhedron->center_).norm() > 0.01)    path.push_back(end_polyhedron->center_);
    if ((path.front() - start_polyhedron->center_).norm() > 0.01) path.insert(path.begin(), start_polyhedron->center_);
    double distance = 0.0;
    for (int i = 0; i < path.size() - 1; i++)
      distance += (path[i] - path[i+1]).norm();
    return distance;
  }

  path.push_back(start_polyhedron->center_);
  path.push_back(end_polyhedron->center_);
  return 99999;
}

double SkeletonGenerator::astarSearch(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point,
                                      std::vector<Eigen::Vector3d>& path, bool add_input_pts=false){
  PolyHedronPtr start_poly, end_poly;
  start_poly = mountCurTopoPoint(start_point, true);
  end_poly   = mountCurTopoPoint(end_point, true);
  if (start_poly == nullptr || end_poly == nullptr){
    path.clear();
    INFO_MSG_YELLOW("[SkeletonGen] | Start or End point can't mount, A* search failed.");
    return -1.0f;
  }
  bool search_succeed = skeleton_astar_->astarSearch(start_poly, end_poly);
  skeleton_astar_->getPath(path);

  // INFO_MSG("[skeletonGen] | A* search from point: ");
  // INFO_MSG("   * start : " << start_point.transpose() << " * end : " << end_point.transpose());
  // INFO_MSG("   * start polyhedron: " << start_poly->center_.transpose()<< " * end polyhedron: " << end_poly->center_.transpose());
  // INFO_MSG("   * A* search path length: " << path.size());

  double distance = 0.0;
  if (!path.empty()) {
    for (int i = 0; i < path.size() - 1; i++)
      distance += (path[i] - path[i+1]).norm();

    if (add_input_pts) {
      distance += (start_point - path.front()).norm();
      distance += (end_point - path.back()).norm();
      path.insert(path.begin(), start_point);
      path.push_back(end_point);
    }
    return distance;
  }

  path.push_back(start_point);
  path.push_back(end_point);
  return 99999;
}

PolyHedronPtr SkeletonGenerator::getObjFatherNode(const Eigen::Vector3d &cur_pos) {
  PolyHedronKDTree_FixedCenterVector polyhedron_nearest;
  getPolyhedronsNNearestWithFixedCenter(cur_pos, 5, polyhedron_nearest);
  if (polyhedron_nearest.empty()) {
    ROS_ERROR("[ObjFactory] | Can't find obj father node, return nullptr");
    return nullptr;
  }
  std::vector<Eigen::Vector3d> path;
  for (const auto& poly : polyhedron_nearest){
    if (searchPathInRawMap(cur_pos, poly.polyhedron_->center_, path, 0.2, true, false)){
      if (poly.polyhedron_->is_gate_) continue;
      if (poly.polyhedron_->area_id_ == -1) continue;
      return poly.polyhedron_;
    }
  }
  ROS_ERROR("[ObjFactory] | Can't find obj father node, return nullptr");
  return nullptr;
}

PolyHedronPtr SkeletonGenerator::getFrontierTopo(const Eigen::Vector3d &cur_pos) {
  PolyHedronKDTree_FixedCenterVector poly_nearest;
  getPolyhedronsNNearestWithFixedCenter(cur_pos, 5, poly_nearest);
  if (poly_nearest.empty()) {
    // ROS_ERROR("[SkeletonGen] | Can't find frontier topo, return nullptr");
    return nullptr;
  }
  std::vector<Eigen::Vector3d> path;
  PolyHedronPtr topo_father = nullptr;
  for (const auto& poly : poly_nearest) {
    if (poly.polyhedron_->is_gate_) continue;    // [gwq] patch for avoiding gates
    if (map_interface_->isVisible(cur_pos, poly.polyhedron_->center_)) {
      topo_father = poly.polyhedron_;
      break;
    }
  }
  if (topo_father == nullptr) {
    // INFO_MSG_YELLOW("[Ftr pather == null] Poly near num : " << poly_nearest.size());
    for (const auto& poly : poly_nearest) {
      if (poly.polyhedron_->is_gate_) continue;          // [gwq] patch for avoiding gates
      // INFO_MSG_YELLOW("No Ftr Topo Father Found, Try to Search Path");
      path.clear();
      if (searchPathInRawMap(cur_pos, poly.polyhedron_->center_, path, 0.1, true, true)){   // [gwq] search path may cause coredump, so use directily visible check first
        topo_father = poly.polyhedron_;
        break;
      }
    }
  }
  if (topo_father == nullptr){
    // ROS_ERROR("[SkeletonGen] | Can't find frontier topo, return nullptr");
  }
  return topo_father;
}

void SkeletonGenerator::getAllPolys(std::vector<PolyHedronPtr> &polyhedrons) {
  polyhedrons.clear();
  for (const auto& poly : polyhedron_map_)
    if (!poly.second->is_gate_)
      polyhedrons.push_back(poly.second);
}

PolyHedronPtr SkeletonGenerator::mountCurTopoPoint(const Eigen::Vector3d& cur_pos, bool ignore_connectivity){
  PolyHedronKDTree_FixedCenterVector polyhedrons_in_range;
  getPolyhedronsInRangeWithFixedCenter(cur_pos, 10, polyhedrons_in_range);
  if (polyhedrons_in_range.empty()) return nullptr;

  std::vector<Eigen::Vector3d> path;
  for (const auto& cur_poly : polyhedrons_in_range){
    if (cur_poly.polyhedron_->is_gate_) continue;
    if (cur_poly.polyhedron_->area_id_ == -1) continue;
    if(ignore_connectivity)
      return cur_poly.polyhedron_;
    if (searchPathInRawMap(cur_pos, polyhedrons_in_range.begin()->polyhedron_->origin_center_, path, 0.2, false, false)){
      return cur_poly.polyhedron_;
    }
  }
  return nullptr;
}

void SkeletonGenerator::updateMountedTopoPoint(const Eigen::Vector3d& cur_pos){
  PolyHedronPtr cur_poly = mountCurTopoPoint(cur_pos, false);
  cur_pos_ = cur_pos;
  if ( cur_poly != nullptr &&  last_mount_polyhedron_ != nullptr){
    if(!checkConnectivityBetweenPolyhedrons(last_mount_polyhedron_, cur_poly)){
      last_mount_polyhedron_->edges_.emplace_back(cur_poly, (cur_poly->center_ - last_mount_polyhedron_->center_).norm());
      cur_poly->edges_.emplace_back(last_mount_polyhedron_, (cur_poly->center_ - last_mount_polyhedron_->center_).norm());
      cur_iter_polys_.push_back(last_mount_polyhedron_);
      cur_iter_polys_.push_back(cur_poly);
    }
  }

  if (cur_poly != nullptr && cur_poly != last_mount_polyhedron_){
    last_mount_polyhedron_ = mount_polyhedron_;
    mount_polyhedron_      = cur_poly;
  }
}

bool SkeletonGenerator::doDenseCheckAndExpand(const Eigen::Vector3d &cur_pos, double yaw) {
  updateMountedTopoPoint(cur_pos);
  if (!checkIfPolyhedronTooDense(cur_pos)) {
    return expandSkeleton(cur_pos, yaw);

      return true;

  }
  return false;
}

bool SkeletonGenerator::expandSkeleton(const Eigen::Vector3d &start_point, double yaw){
  // init global data
  cur_yaw_       = yaw;
  cur_pos_       = start_point;
  local_box_max_ = start_point + _local_range_max;
  local_box_min_ = start_point + _local_range_min;

  cur_iter_polys_.clear();

  ros::Time t_init = ros::Time::now();
  if (sphere_sample_directions_.empty())
    sampleUnitSphere();

  if (facet_vertex_directions_.empty())
    initFacetVerticesDirection();
  // INFO_MSG_RED("[SkeletonGen] | Init time: " << (ros::Time::now() - t_init).toSec() * 1000 << "ms");

  Eigen::Vector3d expand_start_pos = start_point;
  // step0. check if there has some points that need to be processed in last iter, choose one which closest to current position
  adjustExpandStartPt(expand_start_pos);

  // step1. create initial polyhedron
  PolyHedronPtr new_polyhedron = std::make_shared<Polyhedron>(expand_start_pos, nullptr, false);
  bool init_res = initNewPolyhedron(new_polyhedron);
  if (!init_res) return false;

  cur_iter_first_poly_ = new_polyhedron;
  INFO_MSG("[SkeletonGen] | Try To EXPAND Skeleton from point: " << expand_start_pos.transpose());

  // step1.5 find first connection with old polys (it must have new connection !)
  last_mount_polyhedron_ = mount_polyhedron_;
  mount_polyhedron_      = new_polyhedron;
  if (last_mount_polyhedron_ != nullptr){
    if (!checkConnectivityBetweenPolyhedrons(last_mount_polyhedron_, new_polyhedron)){
      findNewTopoConnection(new_polyhedron);
    }
  }

  // step2. expand polyhedron
  bool need_expand = !expand_pending_frontiers_.empty();
  ros::Time start_time = ros::Time::now();
  PolyhedronFtrPtr cur_ftr;
  double expand_time_sum_in_this_iter = 0.0;     // ms
  while (!expand_pending_frontiers_.empty()){
    ros::Time t_begin = ros::Time::now();
    ros::Time t_end   = t_begin;
    cur_ftr = expand_pending_frontiers_.front();
    expand_pending_frontiers_.pop_front();
    if (cur_ftr->deleted_ || cur_ftr == nullptr) continue;

    // step3. Verify frontier for the first time; if invalid, try to adjust it
    verifyFrontier(cur_ftr);
    if (!cur_ftr->valid_){
      adjustFrontier(cur_ftr);
    }

    if (cur_ftr->valid_){
      // std::vector<PolyhedronFtrPtr> vis_ftrs;
      // vis_ftrs.push_back(cur_ftr);
      if (!processAValidFrontier(cur_ftr)){
        // INFO_MSG_RED("[SkeletonGen] | ======= Failed to process valid frontier =======");
      }

      // visualizePolyhedroneInRange(cur_pos_, 5.0);
      // INFO_MSG("Press any key to continue...");
      // getchar();
    }

    // watch dog!
    t_end = ros::Time::now();
    expand_time_sum_in_this_iter += (t_end - t_begin).toSec() * 1000;
    t_begin = t_end;
    // if (expand_time_sum_in_this_iter >= _expand_time_limit){
    //   INFO_MSG_YELLOW("[SkeletonGen] | Expanding time exceeds "<< static_cast<int>(_expand_time_limit) << ", break loop.");
    //   while (!expand_pending_frontiers_.empty()){
    //     expand_pending_frontiers_.pop_front();
    //   }
    //   break;
    // }
  }
  if (need_expand){
    ros::Time end_time = ros::Time::now();
    INFO_MSG_GREEN("\n[SkeletonGen] | -------------------------------------------");
    INFO_MSG_GREEN("[SkeletonGen] | Skeleton Expanding time: " << (end_time - start_time).toSec() * 1000 << "ms");
    INFO_MSG_GREEN("[SkeletonGen] | -------------------------------------------\n");
    visualizePolyhedroneInRange(cur_pos_, 5.0);
    return true;
  }
  return false;
}

void SkeletonGenerator::adjustExpandStartPt(Eigen::Vector3d& start_point){

  PolyHedronKDTree_FixedCenterVector polyhedrons_in_range;
  getPolyhedronsInRangeWithFixedCenter(start_point, _max_expansion_ray_length, polyhedrons_in_range);
  if (!polyhedrons_in_range.empty()){
    INFO_MSG("[SkeletonGen] | Adjust expand start point orin-startpt : " << start_point.transpose());
    Eigen::Vector3d direction = (start_point - polyhedrons_in_range.front().polyhedron_->center_).normalized();
    // do ray-cast
    std::tuple<Eigen::Vector3d, int, Eigen::Vector3d> ray_cast_result =
      rayCast(start_point, direction, _max_expansion_ray_length, 0.1);
    Eigen::Vector3d hit_point = std::get<0>(ray_cast_result);

    // No obstacle detected, likely a long corridor in this direction, need expansion
    if (std::get<1>(ray_cast_result) == -2){
      start_point =start_point + 0.5 * direction * _max_expansion_ray_length;
    }else{
      start_point = (hit_point + start_point) * 0.5;
    }
    INFO_MSG("[SkeletonGen] | adjusted-pt : " << start_point.transpose());
  }

  // if (!remain_candidate_facet_expand_pts_.empty()){
  //   remain_candidate_facet_expand_kd_tree_ = std::make_shared<KD_TREE<skeleton_gen::ikdTree_Vectoe3dType>>();
  //   remain_candidate_facet_expand_kd_tree_->Build(remain_candidate_facet_expand_pts_);
  //
  //   Vector3dKDTreeVector search_result;
  //   getCandidateNxtPosInRange(cur_pos_, _max_expansion_ray_length, search_result);
  //   if (!search_result.empty()){
  //     INFO_MSG_BLUE("[SkeletonGen] | num of candidate hit-points in range: " << search_result.size());
  //     std::vector<Eigen::Vector3d> pts;
  //     for (auto & pt : search_result) pts.push_back(pt.vec3d);
  //     visualization_msgs::MarkerArray marker_array;
  //     visualization_msgs::Marker marker;
  //     drawPoints(pts, 0.1, Eigen::Vector4d(1.0, 0, 0, 1.0), 0, "remain_candidate_facet_expand_pts", ros::Time::now(), marker);
  //     marker_array.markers.push_back(marker);
  //     skeleton_vis_pub_.publish(marker_array);
  //
  //     if ((search_result.front().vec3d - cur_pos_).norm() <= 1.0){
  //       INFO_MSG_BLUE("[SkeletonGen] | ReLocate expand start point");
  //       start_point = search_result.front().vec3d;
  //     }
  //   }
  //   remain_candidate_facet_expand_pts_.clear();
  // }
}

bool SkeletonGenerator::initNewPolyhedron(PolyHedronPtr new_polyhedron){
  ros::Time start_time = ros::Time::now();
  ros::Time previous_time = start_time;
  ros::Time current_time  = start_time;

  if (!checkInBoundingBox(new_polyhedron->center_)){
    // INFO_MSG_RED("[SkeletonGen] | Initial polyhedron center is out of bounding box, skip it." << new_polyhedron->center_.transpose());
    return false;
  }

  if (!checkInLocalUpdateRange(new_polyhedron->center_)){
    // INFO_MSG_RED("[SkeletonGen] | Initial polyhedron center is out of local update range, skip it." << new_polyhedron->center_.transpose());
    return false;
  }

  if (checkIfPolyhedronTooDense(new_polyhedron->center_)){
    // INFO_MSG_RED("[SkeletonGen] | Initial polyhedron is too dense, skip it.");
    return false;
  }

  // todo: there is a checkFloor step here; impact of skipping it unknown
  if (!new_polyhedron->is_gate_){
    // step1. sample black and white vertices and re-calculate the center of the polyhedron
    generatePolyVertices(new_polyhedron);
    current_time = ros::Time::now();
    if ((current_time - previous_time).toSec() * 1000 >= 10.0){
      INFO_MSG_RED("\n!!!!!!!!!!OVER TIME!!!!!!!!!!!");
      INFO_MSG_BLUE("[SkeletonGen] | generateBlackWhiteVertices time: "
             << (current_time - previous_time).toSec() * 1000 << "ms" << "\n"
             << " | black vertices size: " << new_polyhedron->black_vertices_.size() << "\n"
             << " | white vertices size: " << new_polyhedron->white_vertices_.size() << "\n"
             << " | gray  vertices size: " << new_polyhedron->gray_vertices_.size() );
    }
    if (new_polyhedron->black_vertices_.size() + new_polyhedron->gray_vertices_.size() < 4){
      INFO_MSG_YELLOW("[SkeletonGen] | Not enough black vertices, skip it.");
      return false;
    }
    // if (new_polyhedron->white_vertices_.empty()){
    //   INFO_MSG_YELLOW("[SkeletonGen] | No white vertices, skip it.");
    //   return false;
    // }
    centralizePolyhedronCoord(new_polyhedron);

    // step2. calculate polyhedron's radius and do judgement
    if (checkIfPolyhedronTooDense(new_polyhedron->center_))
      return false;
    if (getRadiusOfPolyhedron(new_polyhedron) < _min_node_radius){
      // INFO_MSG_YELLOW("[SkeletonGen] | Polyhedron's radius is too small, skip it.");
      return false;
    }
    // step3. judge if the polyhedron is inside another polyhedron
    ros::Time t11 = ros::Time::now();
    pair<bool, Eigen::Vector3d> check_inside_result = checkIfContainedByAnotherPolyhedron(new_polyhedron);
    INFO_MSG("[SkeletonGen] | checkIfContainedByAnotherPolyhedron time: " << (ros::Time::now() - t11).toSec() * 1000 << "ms");
    if (check_inside_result.first){
      // INFO_MSG_YELLOW("[SkeletonGen] | Polyhedron is inside another polyhedron [" << check_inside_result.second.transpose() << "], skip it.");
      return false;
    }
    // step4. Build polyhedron facets
    previous_time = ros::Time::now();
    initFacetsFromPolyhedron(new_polyhedron);
    current_time = ros::Time::now();
    // INFO_MSG_BLUE("[SkeletonGen] | initFacetsFromPolyhedron time: " << (current_time - previous_time).toSec() * 1000 << "ms");

    previous_time = ros::Time::now();
    generateFrontiers(new_polyhedron);
    current_time = ros::Time::now();
    // INFO_MSG_BLUE("[SkeletonGen] | generateFrontiers time: " << (current_time - previous_time).toSec() * 1000 << "ms");
    INFO_MSG_GREEN("-------------------------------------------------------------------------------------------\n");
  }
  // step-end record polyhedron
  recordNewPolyhedron(new_polyhedron);
  INFO_MSG_GREEN("[SkeletonGen] | Polyhedrone kdtree size: " << polyhedron_kd_tree_->size());
  return true;
}

void SkeletonGenerator::generateFrontiers(PolyHedronPtr polyhedron){  std::vector<std::vector<VertexPtr>> collision_v_groups;
  // loop all white vertex, push vertices into groups which have black vertex neighbors
  ros::Time start_time = ros::Time::now();
  INFO_MSG("--- skeleton vertices info ---");
  INFO_MSG("white vertices size: " << polyhedron->white_vertices_.size()
        << "\nblack vertices size: " << polyhedron->black_vertices_.size()
        << "\ngray  vertices size: " << polyhedron->gray_vertices_.size());
  for (VertexPtr seed_white_v : polyhedron->white_vertices_){
    if (seed_white_v->is_visited_) continue;
    seed_white_v->is_visited_ = true;

    std::vector<VertexPtr> collision_v_single_group;
    deque<VertexPtr>       white_v_bufer;
    white_v_bufer.push_back(seed_white_v);
    while (!white_v_bufer.empty()){
      VertexPtr v = white_v_bufer.front();
      white_v_bufer.pop_front();
      v->is_visited_ = true;
      for (VertexPtr v_neighbor : v->connected_vertices_){
        if (v_neighbor->type_ == Vertex::WHITE){
          if (v_neighbor->is_visited_ == true) continue;
          Eigen::Vector3d mid_pt = (v->position_ + v_neighbor->position_) / 2.0;
          // todo: check if midpoint is too close to obstacle; add candidate white vertices
          if (true)
            white_v_bufer.push_back(v_neighbor);
        }else if (v_neighbor->type_ == Vertex::BLACK || v_neighbor->type_ == Vertex::GRAY){
          v_neighbor->is_critical_ = true;
          collision_v_single_group.push_back(v_neighbor);
        }
      }
    }
    if (collision_v_single_group.size() < 3) continue;
    for (VertexPtr v : collision_v_single_group)
      v->is_visited_ = true;
    collision_v_groups.push_back(collision_v_single_group);
  }

  // Filter collision vertices: keep only those meeting the criteria
  // Acts as a filter to remove extreme noise points
  for (std::vector<VertexPtr> collision_v_single_group : collision_v_groups){
    double mean_length = 0.0;     // average distance from collision samples to center
    int longest_index = -1;
    int shortest_index = -1;
    double longest = 0;
    double shortest = 9999;
    double tolerance = 0;
    for (const VertexPtr& v : collision_v_single_group) mean_length += v->distance_to_center_;
    mean_length /= static_cast<double>(collision_v_single_group.size());
    tolerance    = mean_length * 0.3;

    int vertex_num = static_cast<int>(collision_v_single_group.size());
    for (int i = 0; i < vertex_num; i++){
      // todo: z-axis bound check here; impact of skipping unknown, omitted for now
      if (checkIfOnLocalFloorOrCeil(collision_v_single_group.at(i)->position_) == 0) continue;
      double dis = collision_v_single_group.at(i)->distance_to_center_;
      if (dis - mean_length > tolerance && dis > longest){
        longest_index = i;
        longest = dis;
      }else if (mean_length - dis > tolerance && dis < shortest){
        shortest_index = i;
        shortest = dis;
      }
    }
    if (longest_index != -1){
      collision_v_single_group.at(longest_index)->is_critical_  = false;
      collision_v_single_group.at(longest_index)->type_         = Vertex::RUBBISH;
    }
    if (shortest_index != -1){
      collision_v_single_group.at(shortest_index)->is_critical_ = false;
      collision_v_single_group.at(shortest_index)->type_        = Vertex::RUBBISH;
    }
  }
  // inflate black vertices
  /* Inflate black vertices: if a black vertex is marked as critical,
   * all connected black vertices are also marked as critical.
   * This ensures more vertex connectivity is considered during
   * skeleton construction, preventing omission of important connections. */
  for (auto colli_v_single_group : collision_v_groups)
    for (auto v : colli_v_single_group)
      for (auto v_neighbor : v->connected_vertices_)
        if (v_neighbor->type_ == Vertex::BLACK || v_neighbor->type_ == Vertex::GRAY)
          v_neighbor->is_critical_ = true;

  // create critical vertices mesh
  std::vector<Eigen::Vector3d> collision_v_for_mesh;
  for (VertexPtr black_v : polyhedron->black_vertices_){
    // todo: condition check here (checkFloorOrCeil); impact unknown, omitted
    // if (!black_v->is_critical_ && checkIfOnLocalFloorOrCeil(black_v->position_) != 0) continue;
    collision_v_for_mesh.push_back(sphere_sample_directions_.at(black_v->dir_sample_buffer_index_));
    black_v->is_critical_ = true;
  }
  for (VertexPtr gray_v : polyhedron->gray_vertices_){
    // todo: condition check here (checkFloorOrCeil); impact unknown, omitted
    if (!gray_v->is_critical_) continue;  // todo: validity to be verified
    collision_v_for_mesh.push_back(sphere_sample_directions_.at(gray_v->dir_sample_buffer_index_));
    gray_v->is_critical_ = true;
  }

  quickhull::QuickHull<double> qh;
  quickhull::HalfEdgeMesh<double, size_t> mesh = qh.getConvexHullAsMesh(&collision_v_for_mesh[0].x(), collision_v_for_mesh.size(), true);
  for (auto &facet : mesh.m_faces){
    std::vector<VertexPtr> vertices;
    quickhull::HalfEdgeMesh<double, size_t>::HalfEdge &half_edge = mesh.m_halfEdges[facet.m_halfEdgeIndex];
    for (int i = 0; i < 3; i++){
      quickhull::Vector3<double> vertex_qh = mesh.m_vertices[half_edge.m_endVertex];
      Eigen::Vector3d vertex_eigen(vertex_qh.x, vertex_qh.y, vertex_qh.z);
      vertices.push_back(getVertexFromDirection(polyhedron, vertex_eigen));
      half_edge = mesh.m_halfEdges[half_edge.m_next];
    }
    FacetPtr new_fecet = std::make_shared<Facet>(vertices, polyhedron);
    new_fecet->index_ = polyhedron->facets_.size();
    polyhedron->facets_.push_back(new_fecet);
  }

  // Calculate outwards normal for each facet
  // Generate candidate unit normals and determine final normal orientation
  for (auto & facet : polyhedron->facets_){
    Eigen::Vector3d v1 = facet->vertices_.at(1)->position_ - facet->vertices_.at(0)->position_;
    Eigen::Vector3d v2 = facet->vertices_.at(2)->position_ - facet->vertices_.at(0)->position_;
    Eigen::Vector3d candidate_normal = v1.cross(v2);
    candidate_normal.normalize();

    Eigen::Vector3d pt_to_judge = facet->center_ + candidate_normal;
    facet->out_unit_normal_ = ifPtInIpsilateralOfPlane(polyhedron->center_, pt_to_judge, facet) ?
                              candidate_normal : -candidate_normal;
  }

  // BFS to find frontier clusters
  for (auto & colli_v_single_group : collision_v_groups){
    std::vector<FacetPtr> facets_group;
    findFacetsGroupFromVertices(polyhedron, colli_v_single_group, facets_group);
    if (facets_group.empty()) continue;
    findNeighborFacets(facets_group);

    // start frontier clustering process
    std::vector<std::vector<FacetPtr>> frontier_clusters;
    for (FacetPtr facet : facets_group){
      if (facet->is_linked_) continue;
      std::vector<FacetPtr> single_cluster;
      deque<FacetPtr>       pending_facet;

      pending_facet.push_back(facet);
      while (!pending_facet.empty()){
        FacetPtr cur_facet = pending_facet.front();
        pending_facet.pop_front();
        if (cur_facet->is_linked_) continue;
        single_cluster.push_back(cur_facet);
        cur_facet->is_linked_ = true;
        for (FacetPtr neighbor_facet : cur_facet->neighbor_facets_)
          if (!neighbor_facet->is_linked_)
            pending_facet.push_back(neighbor_facet);
      }
      frontier_clusters.push_back(single_cluster);
    }
    for (auto cluster : frontier_clusters){
      std::vector<PolyhedronFtrPtr> result_frontiers;
      splitFrontier(polyhedron, cluster, result_frontiers);
      for (PolyhedronFtrPtr frontier : result_frontiers)
        polyhedron->ftrs_.push_back(frontier);
    }
  }

  // Frontier boundary filtering
  // Add bv_group for those connecting bvs having big diff in dis_to_center
  // Cluster non-frontier facets !!!!
  std::vector<FacetPtr> ignored_facets;
  for (FacetPtr cur_facet : polyhedron->facets_){
    if (cur_facet->frontier_processed_) continue;
    // todo: z-axis bound check here; impact unknown, omitted
    int ignore_cnt = 0;
    for (int i = 0; i < 3; i++){
      VertexPtr v1 = cur_facet->vertices_.at(i);
      VertexPtr v2 = cur_facet->vertices_.at((i + 1) % 3);
      if (v1->distance_to_center_ > _frontier_jump_threshold * v2->distance_to_center_ ||
          v2->distance_to_center_ > _frontier_jump_threshold * v1->distance_to_center_)
        ignore_cnt++;
    }
    if (ignore_cnt > 1){
      ignored_facets.push_back(cur_facet);
      cur_facet->frontier_processed_ = true;
    }
  }
  findNeighborFacets(ignored_facets);
  std::vector<std::vector<FacetPtr>> linked_groups;
  for (FacetPtr f : ignored_facets){
    if (f->is_linked_) continue;
    std::vector<FacetPtr> linked_facets;
    deque<FacetPtr>       pending_facet;
    pending_facet.push_back(f);
    while (!pending_facet.empty()){
      FacetPtr cur_facet = pending_facet.front();
      pending_facet.pop_front();
      linked_facets.push_back(cur_facet);
      cur_facet->is_linked_ = true;
      for (FacetPtr neighbor_facet : cur_facet->neighbor_facets_)
        if (!neighbor_facet->is_linked_)
          pending_facet.push_back(neighbor_facet);
    }
    linked_groups.push_back(linked_facets);
  }
  for (std::vector<FacetPtr> claster : linked_groups){
    PolyhedronFtrPtr new_ftr = std::make_shared<PolyhedronFtr>(claster, polyhedron);
    if (initSingleFrontier(new_ftr))
      polyhedron->ftrs_.push_back(new_ftr);
  }
  auto sort_cmp = [](const PolyhedronFtrPtr& a, const PolyhedronFtrPtr& b){
    return a->area_size_ > b->area_size_;
  };
  sort(polyhedron->ftrs_.begin(), polyhedron->ftrs_.end(), sort_cmp);
  for (PolyhedronFtrPtr cur_ftr : polyhedron->ftrs_){
    if (cur_ftr->facets_.empty()) continue;
    expand_pending_frontiers_.push_back(cur_ftr);
  }
}

// Verify frontier validity
// If valid, determine the next node position, write to frontier's next_node_pos, and set valid to true
void SkeletonGenerator::verifyFrontier(PolyhedronFtrPtr ftr){
  Eigen::Vector3d ray_cast_start_pt = ftr->proj_center_ + 2.0 * ftr->out_unit_normal_ * _search_margin;
  std::tuple<Eigen::Vector3d, int, Eigen::Vector3d> collision_res =
    rayCast(ray_cast_start_pt, ftr->out_unit_normal_, 2 * _search_margin, 0.2);
  if (std::get<1>(collision_res) != -2 && (std::get<0>(collision_res) - ray_cast_start_pt).norm() < _search_margin){
    ftr->valid_ = false;
    return;
  }

  // do ray-cast
  std::tuple<Eigen::Vector3d, int, Eigen::Vector3d> ray_cast_result =
    rayCast(ray_cast_start_pt, ftr->out_unit_normal_, _max_expansion_ray_length, 0.2);
  Eigen::Vector3d hit_point = std::get<0>(ray_cast_result);

  // No obstacle detected, likely a long corridor, need expansion
  bool need_expand = false;
  Eigen::Vector3d new_polyhedron_center;
  if (std::get<1>(ray_cast_result) == -2){
    need_expand = true;
    new_polyhedron_center = ftr->proj_center_ + 0.5 * ftr->out_unit_normal_ * _max_expansion_ray_length;
  }
  // Obstacle detected but still some distance away, likely a boundary, need expansion
  else if (getDistance(hit_point, ftr->proj_center_) > _frontier_creation_threshold){
    need_expand = true;
    new_polyhedron_center = (hit_point + ftr->proj_center_) * 0.5;
  }

  if (need_expand){
    std::vector<Eigen::Vector3d> path;
    bool need_expand_check_in_nxt_iter = false;
    if (!checkInBoundingBox(new_polyhedron_center))       return ;
    if (!checkInLocalUpdateRange(new_polyhedron_center))  {need_expand_check_in_nxt_iter = true;}
    if (checkIfPolyhedronTooDense(new_polyhedron_center)) return ;
    if (!searchPathInRawMap(ftr->master_polyhedron->center_, new_polyhedron_center, path, 0.1, true, false)) {need_expand_check_in_nxt_iter = true;}

    if (need_expand_check_in_nxt_iter){
      remain_candidate_facet_expand_pts_.emplace_back(new_polyhedron_center);
      return ;
    }

    double path_len = 0.0;
    for (int i = 0; i < path.size() - 1; i++)
      path_len += (path[i] - path[i + 1]).norm();

    ftr->master_polyhedron->temp_distance_to_nxt_poly_ = path_len + (ftr->proj_center_ - ftr->master_polyhedron->center_).norm();
    ftr->valid_         = true;
    ftr->next_node_pos_ = new_polyhedron_center;

    // INFO_MSG("[Debug] | verifyFrontier: need expand, path len: " << path_len
    //       << ", new polyhedron center: " << new_polyhedron_center.transpose());
  }
}

void SkeletonGenerator::adjustFrontier(PolyhedronFtrPtr ftr){
  // step1: try adjusting only the projection center
  if (ftr->proj_facet_ == nullptr) return;

  Eigen::Vector3d prev_proj_center = ftr->proj_center_;
  Eigen::Vector3d prev_normal      = ftr->out_unit_normal_;
  ftr->proj_center_                = ftr->proj_facet_->center_;
  verifyFrontier(ftr);
  if (!ftr->valid_){
    // step2: try adjusting only the normal vector
    ftr->proj_center_     = prev_proj_center;
    ftr->out_unit_normal_ = ftr->proj_facet_->out_unit_normal_;
    verifyFrontier(ftr);
    if (!ftr->valid_){
      // step3: try adjusting both normal and projection center
      ftr->proj_center_   = ftr->proj_facet_->center_;
      verifyFrontier(ftr);
      if (!ftr->valid_){
        // step4: try using the centroid of a neighboring facet as the frontier center
        ftr->out_unit_normal_ = prev_normal;
        for (FacetPtr cur_facet : ftr->proj_facet_->neighbor_facets_){
          if (ftr->valid_) break;
          ftr->proj_center_ = cur_facet->center_;
          verifyFrontier(ftr);
          // step5: try using the normal of a neighboring facet as the frontier normal
          if (!ftr->valid_){
            ftr->out_unit_normal_ = cur_facet->out_unit_normal_;
            verifyFrontier(ftr);
          }
        }
      }
    }
  }
}

bool SkeletonGenerator::processAValidFrontier(PolyhedronFtrPtr cur_ftr){
  PolyHedronPtr gate;
  if (cur_ftr->gate_ == nullptr){
    gate = std::make_shared<Polyhedron>(cur_ftr->proj_center_, cur_ftr, true);
    cur_ftr->gate_ = gate;
  }else
    gate = cur_ftr->gate_;

  // todo: check exists here, omitted for now
  /*
  bool floor = checkFloor(gate);
  bool bbx = checkWithinBbx(gate->coord);
  // If gate node is not on the ground or outside bbx, skip rollback and mark frontier invalid
  if (!floor || !bbx) {
    gate->rollbacked = true;
    if (!floor) {
      ROS_INFO("processFrontier: no floor");
    }
    if (!bbx) {
      ROS_INFO("processFrontier: outside bbx");
    }
    ROS_INFO("processFrontier: gate coord: (%f, %f, %f)",
              gate->coord(0), gate->coord(1), gate->coord(2));
    return false;
  }
   */

  PolyHedronPtr new_polyhedron = std::make_shared<Polyhedron>(cur_ftr->next_node_pos_, cur_ftr, false);
  bool init_success = initNewPolyhedron(new_polyhedron);
  if (init_success){
    initNewPolyhedron(gate);
    cur_ftr->master_polyhedron->connected_nodes_.push_back(gate);
    gate->connected_nodes_.push_back(cur_ftr->master_polyhedron);
    gate->connected_nodes_.push_back(new_polyhedron);
    new_polyhedron->connected_nodes_.push_back(gate);

    // process current edge
    cur_ftr->master_polyhedron->edges_.emplace_back(new_polyhedron, cur_ftr->master_polyhedron->temp_distance_to_nxt_poly_);
    new_polyhedron->edges_.emplace_back(cur_ftr->master_polyhedron, cur_ftr->master_polyhedron->temp_distance_to_nxt_poly_);
    // process candidate loopback edges
    findLoopbackConnectionFromCandidate(new_polyhedron);
    return true;
  }

  if (gate->connected_nodes_.empty()){
    cur_ftr->valid_ = false;
    for (FacetPtr facet : cur_ftr->facets_)
      facet->frontier_processed_ = false;
  }
  return false;
}

void SkeletonGenerator::findNewTopoConnection(PolyHedronPtr polyhedron){
  INFO_MSG_YELLOW("[SkeletonGen] | findNewTopo: start");
  INFO_MSG_YELLOW("              | findNewTopo: poly center: " << polyhedron->center_.transpose());
  INFO_MSG_YELLOW("              | findNewTopo: last mount poly center: " << last_mount_polyhedron_->center_.transpose());
  PolyHedronKDTree_FixedCenterVector polyhedrons_in_range_fixed_center;
  bool found_new_connection = false;
  getPolyhedronsInRangeWithFixedCenter(polyhedron->center_, 2.5 * _min_node_dense_radius, polyhedrons_in_range_fixed_center);

  ros::Time t1 = ros::Time::now();
  for (auto &p : polyhedrons_in_range_fixed_center){
    std::vector<Eigen::Vector3d> path;
    PolyHedronPtr perv_poly = p.polyhedron_;

    bool already_connected = false;
    for (auto edge : perv_poly->edges_)
      if (isSamePose(edge.poly_nxt_->center_, polyhedron->center_)){
        already_connected = true;
        break;
      }
    if (already_connected) continue;

    if (searchPathInRawMap(polyhedron->center_, perv_poly->center_, path, 0.1, true, true)){
      found_new_connection = true;
      double len = 0.0;
      for (int i = 0; i < path.size() - 1; i++)
        len += (path[i] - path[i + 1]).norm();

      INFO_MSG_YELLOW("              | findNewTopoConnection: path length: " << len);
      polyhedron->edges_.emplace_back(perv_poly, len);
      perv_poly->edges_.emplace_back(polyhedron, len);
      INFO_MSG_GREEN("               | findNewTopoConnection: found new connection!");
    }
  }
  // INFO_MSG("findNewTopo: search path time: " << (ros::Time::now() - t1).toSec() * 1000 << "ms");

  if (polyhedron->edges_.empty() || !found_new_connection){
    INFO_MSG_RED("              | findNewTopoConnection: topo connection not found, try last mount polyhedron!");
    std::vector<Eigen::Vector3d> path;
    double len = 0.0;
    if (searchPathInRawMap(polyhedron->center_, last_mount_polyhedron_->center_, path, 0.1, true, true)){
      for (int i = 0; i < path.size() - 1; i++)
        len += (path[i] - path[i + 1]).norm();
    }else{
      INFO_MSG_YELLOW("              | findNewTopoConnection: topo connection not found, try FORCE connection!");
      len = (last_mount_polyhedron_->center_ - polyhedron->center_).norm();
    }
    polyhedron->edges_.emplace_back(last_mount_polyhedron_, len);
    polyhedron->edges_.back().forceConnect();
    last_mount_polyhedron_->edges_.emplace_back(polyhedron, len);
    last_mount_polyhedron_->edges_.back().forceConnect();
  }
}

void SkeletonGenerator::findLoopbackConnectionFromCandidate(PolyHedronPtr polyhedron){
  for (auto nxt_poly : polyhedron->candidate_rollback_){
    std::vector<Eigen::Vector3d> path;
    if (searchPathInRawMap(polyhedron->center_, nxt_poly.first, path, 0.2, false, false)){   // attention here, use false to avoid mis-detection
      double len = 0.0;
      for (int i = 0; i < path.size() - 1; i++)
        len += (path[i] - path[i + 1]).norm();
      if (len <= 3.0 * _min_node_dense_radius ){
        polyhedron->edges_.emplace_back(polyhedron_map_[nxt_poly.first], len);
        polyhedron_map_[nxt_poly.first]->edges_.emplace_back(polyhedron, len);
      }
    }
  }
}

bool SkeletonGenerator::checkConnectivityBetweenPolyhedrons(PolyHedronPtr p1, PolyHedronPtr p2){
  for (auto edge : p1->edges_){
    if ((edge.poly_nxt_->center_ - p2->center_).norm() < 1e-3)
      return true;
  }
  return false;
}

// return pair<bool, Eigen::Vector3d>
// <whether inside another polyhedron, polyhedron origin center>
pair<bool, Eigen::Vector3d> SkeletonGenerator::checkIfContainedByAnotherPolyhedron(PolyHedronPtr polyhedron){

  std::unordered_map<Eigen::Vector3d, bool, Vector3dHash> polyhedron_map;
  for (const auto& p : polyhedron->black_vertices_){
    if (polyhedron_map_.find(p->collision_polyhedron_index_) != polyhedron_map_.end())
      polyhedron_map[p->collision_polyhedron_index_] = true;
  }

  for (const auto& poly : polyhedron_map){
    bool if_contained = true;
    PolyHedronPtr cur_poly = polyhedron_map_[poly.first];
    for (const auto& facet : cur_poly->facets_){
      if (ifPtInIpsilateralOfPlane(polyhedron->origin_center_, cur_poly->origin_center_, facet)){
        if_contained = false;
        break;
      }
    }
    if (if_contained){
      INFO_MSG_YELLOW("polyhedron : " << polyhedron->origin_center_.transpose()
                   << " is contained by polyhedron : " << poly.first.transpose());
      return make_pair(true, poly.first);
    }
  }
  return make_pair(false, Eigen::Vector3d(-9999, -9999, -9999));
}

double SkeletonGenerator::getRadiusOfPolyhedron(PolyHedronPtr polyhedron){
  double radius = 0.0;
  for (const VertexPtr& pt : polyhedron->black_vertices_) radius += getDistance(polyhedron->center_, pt->position_);
  for (const VertexPtr& pt : polyhedron->gray_vertices_ ) radius += getDistance(polyhedron->center_, pt->position_);
  radius /= static_cast<double>(polyhedron->black_vertices_.size() + polyhedron->gray_vertices_.size());
  polyhedron->radius_ = radius;
  return radius;
}

void SkeletonGenerator::centralizePolyhedronCoord(PolyHedronPtr polyhedron){
  if (polyhedron->black_vertices_.empty()) return;
  Eigen::Vector3d center_sum(0, 0, 0);
  for (const auto& p : polyhedron->black_vertices_)
    center_sum += p->position_;
  // for (const auto& p : polyhedron->gray_vertices_)
  //   center_sum += p->position_;

  center_sum /= static_cast<double>(polyhedron->black_vertices_.size());
  polyhedron->center_ = center_sum;
}

// Sample to generate black and white vertices
void SkeletonGenerator::generatePolyVertices(PolyHedronPtr poly){
  int num_directions = static_cast<int>(sphere_sample_directions_.size());
  for (int i = 0; i < num_directions; i++){
    Eigen::Vector3d direction = sphere_sample_directions_[i];
    if (direction.norm() < 1e-6) continue;
    // ros::Time t1 = ros::Time::now();
    std::tuple<Eigen::Vector3d, int, Eigen::Vector3d> ray_cast_result = rayCast(poly->center_, direction, _max_ray_length, 0.2);
    // INFO_MSG("generateBlackWhiteVertices: raycast time: " << (ros::Time::now() - t1).toSec() * 1000 << "ms");
    Eigen::Vector3d hit_point = std::get<0>(ray_cast_result);
    // INFO_MSG("[SkeletonGen] | hit point :" << std::get<0>(ray_cast_result).transpose()
    //   << " length :" << getDistance(polyhedron->center_, hit_point) << "raycast type:" << std::get<1>(ray_cast_result));

    poly->box_max_ = Eigen::Vector3d(std::max(hit_point(0), poly->box_max_.x()),
                                     std::max(hit_point(1), poly->box_max_.y()),
                                     std::max(hit_point(2), poly->box_max_.z()));
    poly->box_min_ = Eigen::Vector3d(std::min(hit_point(0), poly->box_min_.x()),
                                     std::min(hit_point(1), poly->box_min_.y()),
                                     std::min(hit_point(2), poly->box_min_.z()));

    // no collision, add white vertex
    if (std::get<1>(ray_cast_result) == -2){
      VertexPtr new_vertex = std::make_shared<Vertex>(hit_point, direction, Vertex::WHITE);
      poly->white_vertices_.push_back(new_vertex);
      poly->white_vertices_.back()->dir_sample_buffer_index_ = i;
      poly->white_vertices_.back()->distance_to_center_ = getDistance(poly->center_, hit_point);
    }else if (std::get<1>(ray_cast_result) == -3){ // have collision with virtual wall, add GRAY vertex
      VertexPtr new_vertex = std::make_shared<Vertex>(hit_point, direction, Vertex::GRAY);
      poly->gray_vertices_.push_back(new_vertex);
      poly->gray_vertices_.back()->dir_sample_buffer_index_ = i;
      poly->gray_vertices_.back()->distance_to_center_  = getDistance(poly->center_, hit_point);
      poly->gray_vertices_.back()->collision_polyhedron_index_ = Eigen::Vector3d(-9999, -9999, -9999);
    }else{  // have collision with obstacle, add black vertex
      VertexPtr new_vertex = std::make_shared<Vertex>(hit_point, direction, Vertex::BLACK);
      poly->black_vertices_.push_back(new_vertex);
      poly->black_vertices_.back()->dir_sample_buffer_index_ = i;
      poly->black_vertices_.back()->distance_to_center_ = getDistance(poly->center_, hit_point);
      poly->black_vertices_.back()->collision_polyhedron_index_ = std::get<2>(ray_cast_result);
      if (poly->parent_ftr_ != nullptr && std::get<2>(ray_cast_result) != Eigen::Vector3d(-9999, -9999, -9999)
          && !isSamePose(poly->parent_ftr_->master_polyhedron->origin_center_, std::get<2>(ray_cast_result))){
        poly->candidate_rollback_[std::get<2>(ray_cast_result)] = true;
      }
    }
  }
}

// 1. Standard QuickHull algorithm generates convex hull with CCW triangle vertex order
// 2. Generate convex hull of unit sphere samples to obtain facet vertex directions relative to sphere center
void SkeletonGenerator::initFacetVerticesDirection(){
  quickhull::QuickHull<double> qh;
  quickhull::HalfEdgeMesh<double, size_t> mesh =
    qh.getConvexHullAsMesh(&sphere_sample_directions_[0].x(), sphere_sample_directions_.size(), true);
  for (const auto facet : mesh.m_faces){
    quickhull::HalfEdgeMesh<double, size_t>::HalfEdge &half_edge = mesh.m_halfEdges[facet.m_halfEdgeIndex];
    quickhull::Vector3<double> vertex{};
    std::vector<Eigen::Vector3d> one_facet_vertices;

    for (int i = 0; i < 3; i++){
      vertex = mesh.m_vertices[half_edge.m_endVertex];
      one_facet_vertices.emplace_back(vertex.x, vertex.y, vertex.z);
      half_edge = mesh.m_halfEdges[half_edge.m_next];
    }
    facet_vertex_directions_.push_back(one_facet_vertices);
  }
}

// --------------- Utils ----------------- //
template<typename T>
void SkeletonGenerator::readParam(std::string param_name, T &param_val, T default_val) {
  if (!nh_.param(param_name, param_val, default_val))
    INFO_MSG_YELLOW("[SkeletonGen] | parameter " << param_name << " not found, using default value: " << default_val);
  else
    INFO_MSG_GREEN("[SkeletonGen] | parameter " << param_name << " found: " << param_val);
}

bool SkeletonGenerator::checkInBoundingBox(const Eigen::Vector3d& point){
#ifdef _MAP_TYPE_MAP_INTERFACE
  return map_interface_->isInLocalMap(point);
#endif
#ifdef  _MAP_TYPE_POINT_CLOUD
#endif
#ifdef  _MAP_TYPE_OCCUPANCY_MAP
#endif
}

bool SkeletonGenerator::checkInLocalUpdateRange(const Eigen::Vector3d& point){
  // Axis-aligned bounding box
  Eigen::Vector3d pt_in_body_frame = transPointToBodyFrame(point);
  if (pt_in_body_frame.x() > _local_x_max || pt_in_body_frame.x() < _local_x_min ||
      pt_in_body_frame.y() > _local_y_max || pt_in_body_frame.y() < _local_y_min ||
      pt_in_body_frame.z() > _local_z_max || pt_in_body_frame.z() < _local_z_min)
    return false;

  return true;
}

// return
//  1 ceil
// -1 floor
//  0 none!
int SkeletonGenerator::checkIfOnLocalFloorOrCeil(const Eigen::Vector3d& point){
  Eigen::Vector3d pt_in_body_frame = transPointToBodyFrame(point);
  if (abs(pt_in_body_frame.z() - _local_z_max) < _search_margin) return  1;
  if (abs(pt_in_body_frame.z() - _local_z_min) < _search_margin) return -1;
  return 0;
}

bool SkeletonGenerator::checkIfPolyhedronTooDense(const Eigen::Vector3d& center_pt){
  PolyHedronKDTree_FixedCenterVector polyhedrons_in_range_fixed_center;
  getPolyhedronsInRangeWithFixedCenter(center_pt,  _min_node_dense_radius, polyhedrons_in_range_fixed_center);
  // getItemsInRangeAndSortByDistance<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>
  //   (center_pt, _min_node_dense_radius, polyhedrons_in_range_fixed_center);
  if (polyhedrons_in_range_fixed_center.empty()) return false;

  PolyHedronKDTreeVector polyhedrons_in_range;
  getPolyhedronsInRange(center_pt, _min_node_dense_radius, polyhedrons_in_range);
  // getItemsInRangeAndSortByDistance<skeleton_gen::ikdTree_PolyhedronType>
  //   (center_pt, _min_node_dense_radius, polyhedrons_in_range);
  if (polyhedrons_in_range.empty()) return false;

  return true;
}

void SkeletonGenerator::recordNewPolyhedron(PolyHedronPtr polyhedron){
  if (polyhedron_map_.find(polyhedron->origin_center_) == polyhedron_map_.end()){
    polyhedron->can_reach_ = true;
    polyhedron_map_[polyhedron->origin_center_] = polyhedron;
    PolyHedronKDTreeVector add_temp;
    add_temp.emplace_back(polyhedron);
    PolyHedronKDTree_FixedCenterVector add_temp_fixed_center;
    add_temp_fixed_center.emplace_back(polyhedron);

    if (has_init_polyhedron_kdtree_){
      polyhedron_kd_tree_->Add_Points(add_temp, false);
      polyhedron_kd_tree_fixed_center_->Add_Points(add_temp_fixed_center, false);

      // if (!polyhedron->is_gate_)
      //   visualizePolyhedronVertices(polyhedron);
    }
    else{
      polyhedron_kd_tree_->Build(add_temp);
      polyhedron_kd_tree_fixed_center_->Build(add_temp_fixed_center);
      has_init_polyhedron_kdtree_ = true;
      INFO_MSG_GREEN("[SkeletonGen] | Init polyhedron kdtree success !!!");
    }
    cur_iter_polys_.push_back(polyhedron);
  }else{
    INFO_MSG_RED("[SkeletonGen] | Record new polyhedron failed");
  }
}

void SkeletonGenerator::getPolyhedronsNNearestWithFixedCenter(const Eigen::Vector3d &pt, const int &k, PolyHedronKDTree_FixedCenterVector &polyhedrons_nearest) {
  auto sort_cmp = [pt](const skeleton_gen::ikdTree_PolyhedronType_FixedCenter& p1, const skeleton_gen::ikdTree_PolyhedronType_FixedCenter& p2){
    return (p1.polyhedron_->origin_center_ - pt).norm() < (p2.polyhedron_->origin_center_ - pt).norm();
  };
  if (has_init_polyhedron_kdtree_) {
    std::vector<float> distance_list;
    skeleton_gen::ikdTree_PolyhedronType_FixedCenter polyhedron_search(nullptr);
    polyhedron_search.set_coordinate(pt);
    polyhedron_kd_tree_fixed_center_->Nearest_Search(polyhedron_search, k, polyhedrons_nearest, distance_list);
    sort(polyhedrons_nearest.begin(), polyhedrons_nearest.end(), sort_cmp);
  }else {
    INFO_MSG_RED("[SkeletonGen] | KDTree not init, can not get polyhedrons in range !");
    polyhedrons_nearest.clear();
  }
}


// P.S. Time Spend avg : 0.01ms
void SkeletonGenerator::getPolyhedronsInRange
     (const Eigen::Vector3d& pt, const double &radius, PolyHedronKDTreeVector & polyhedrons_in_range){
  auto sort_cmp = [pt](const skeleton_gen::ikdTree_PolyhedronType& p1, const skeleton_gen::ikdTree_PolyhedronType& p2){
    return (p1.polyhedron_->origin_center_ - pt).norm() < (p2.polyhedron_->origin_center_ - pt).norm();
  };

  if (has_init_polyhedron_kdtree_) {
    skeleton_gen::ikdTree_PolyhedronType polyhedron_search(nullptr);
    polyhedron_search.set_coordinate(pt);
    polyhedron_kd_tree_->Radius_Search(polyhedron_search, static_cast<double>(radius), polyhedrons_in_range);
    sort(polyhedrons_in_range.begin(), polyhedrons_in_range.end(), sort_cmp);
  } else {
    INFO_MSG_RED("[SkeletonGen] | KDTree not init, can not get polyhedrons in range !");
    polyhedrons_in_range.clear();
  }
}

void SkeletonGenerator::getPolyhedronsInRangeWithFixedCenter
     (const Eigen::Vector3d& pt, const double &radius, PolyHedronKDTree_FixedCenterVector & polyhedrons_in_range){

  auto sort_cmp = [pt](const skeleton_gen::ikdTree_PolyhedronType_FixedCenter& p1,
                       const skeleton_gen::ikdTree_PolyhedronType_FixedCenter& p2){
    return (p1.polyhedron_->center_ - pt).norm() < (p2.polyhedron_->center_ - pt).norm();
  };

  if (has_init_polyhedron_kdtree_){
    skeleton_gen::ikdTree_PolyhedronType_FixedCenter polyhedron_search(nullptr);
    polyhedron_search.set_coordinate(pt);
    polyhedron_kd_tree_fixed_center_->Radius_Search(polyhedron_search, static_cast<double>(radius), polyhedrons_in_range);
    sort(polyhedrons_in_range.begin(), polyhedrons_in_range.end(), sort_cmp);
  }else{
    INFO_MSG_RED("[SkeletonGen] | KDTree not init, can not get polyhedrons in range !");
    polyhedrons_in_range.clear();
  }
}

void SkeletonGenerator::getCandidateNxtPosInRange(const Eigen::Vector3d& pt, const double& radius, Vector3dKDTreeVector& candidate_nxt_pos_in_range){
  auto sort_cmp = [pt](const skeleton_gen::ikdTree_Vectoe3dType & p1, const skeleton_gen::ikdTree_Vectoe3dType & p2 ){
    return (p1.vec3d - pt).norm() < (p2.vec3d - pt).norm();
  };
  skeleton_gen::ikdTree_Vectoe3dType search_pt(pt);
  remain_candidate_facet_expand_kd_tree_->Radius_Search(search_pt, static_cast<double>(radius), candidate_nxt_pos_in_range);
  sort(candidate_nxt_pos_in_range.begin(), candidate_nxt_pos_in_range.end(), sort_cmp);
}

// get items in range and sort by distance to center
template <typename T>
void SkeletonGenerator::getItemsInRangeAndSortByDistance(const Eigen::Vector3d& pt, const double& radius,
                                        std::vector<T, Eigen::aligned_allocator<T>>& polyhedrons_in_range){
  auto sort_cmp = [pt](const T& p1, const T& p2){
    return (p1->center_ - pt).norm() < (p2->center_ - pt).norm();
  };
  if (has_init_polyhedron_kdtree_){
    T polyhedron_search(nullptr);
    polyhedron_search.set_coordinate(pt);
    polyhedron_kd_tree_fixed_center_->Radius_Search(polyhedron_search, static_cast<double>(radius), polyhedrons_in_range);
    sort(polyhedrons_in_range.begin(), polyhedrons_in_range.end(), sort_cmp);
  }else{
    INFO_MSG_RED("[SkeletonGen] | KDTree not init, can not get polyhedrons in range !");
    polyhedrons_in_range.clear();
  }
}

void SkeletonGenerator::sampleUnitSphere(){
  sphere_sample_directions_.clear();
  sphere_sample_directions_qh_.clear();

  // // Fibonacci sphere
  // Adjust golden angle by density factor
  // double golden_angle = M_PI * (3. - sqrt(5.)) * 1.0;
  //
  // // Generate sample points
  // for (int i = 0; i < _sampling_density; ++i) {
  //   double y = 1 - (static_cast<double>(i) / static_cast<double>(_sampling_density - 1)) * 2.0;
  //   double radius = sqrt(1 - y * y);
  //   double theta = golden_angle * i;
  //   double x = cos(theta) * radius;
  //   double z = sin(theta) * radius;
  //   sphere_sample_directions_.emplace_back(x, y, z);
  //   sphere_sample_directions_qh_.emplace_back(x, y, z);
  // }

  // N-level cube sampling
  int sample_level = _sampling_level;
  double interval = 1.0 / static_cast<double>(sample_level);
  for (int iter_y = 0; iter_y < sample_level + 1; iter_y++){
    for (int iter_z = 0; iter_z < sample_level - 1; iter_z++){
      double y = static_cast<double>(iter_y) * interval - 0.5;
      double z = static_cast<double>(iter_z + 1) * interval - 0.5;
      Eigen::Vector3d direction(0.5, y, z);
      direction.normalize();
      sphere_sample_directions_.push_back(direction);
      sphere_sample_directions_qh_.emplace_back(direction.x(), direction.y(), direction.z());

      direction = Eigen::Vector3d(-0.5, y, z);
      direction.normalize();
      sphere_sample_directions_.push_back(direction);
      sphere_sample_directions_qh_.emplace_back(direction.x(), direction.y(), direction.z());
    }
  }

  for (int iter_x = 0; iter_x < sample_level - 1; iter_x++){
    for (int iter_z = 0; iter_z < sample_level - 1; iter_z++){
      double x = static_cast<double>(iter_x + 1) * interval - 0.5;
      double z = static_cast<double>(iter_z + 1) * interval - 0.5;
      Eigen::Vector3d direction(x, 0.5, z);
      direction.normalize();
      sphere_sample_directions_.push_back(direction);
      sphere_sample_directions_qh_.emplace_back(direction.x(), direction.y(), direction.z());
      direction = Eigen::Vector3d(x, -0.5, z);
      direction.normalize();
      sphere_sample_directions_.push_back(direction);
    }
  }

  interval = 1.0 / static_cast<double>(sample_level - 1);
  for (int iter_x = 0; iter_x < sample_level - 2; iter_x++){
    for (int iter_y = 0; iter_y < sample_level - 2; iter_y++){
      double x = static_cast<double>(iter_x + 1) * interval - 0.5;
      double y = static_cast<double>(iter_y + 1) * interval - 0.5;
      Eigen::Vector3d direction(x, y, 0.5);
      direction.normalize();
      sphere_sample_directions_.push_back(direction);
      sphere_sample_directions_qh_.emplace_back(direction.x(), direction.y(), direction.z());
      direction = Eigen::Vector3d(x, y, -0.5);
      direction.normalize();
      sphere_sample_directions_.push_back(direction);
      sphere_sample_directions_qh_.emplace_back(direction.x(), direction.y(), direction.z());
    }
  }
  INFO_MSG_BLUE("[SkeletonGen] | sphere_sample_directions_ size: " << sphere_sample_directions_.size());
  visualizeSphereSampleDirections();
  ros::Duration(0.001).sleep();
}

// // return : (IF_COLLISION, index)
// // Return the nearest obstacle distance and its index
// inline pair<double, Eigen::Vector3d>
//             SkeletonGenerator::collisionCheck(const Eigen::Vector3d& point){
//   Eigen::Vector3d min_dis_node_index(-9999, -9999, -9999);
//   double min_dis = 9999.0;
//
// #ifdef _MAP_TYPE_MAP_INTERFACE
//   if (map_interface_->getInflateOccupancy(point) == global_belief::MapInterface::OCCUPIED){
//     return make_pair(min_dis, min_dis_node_index);
//   }
//   // check if have collision with polyhedron
//   PolyHedronKDTreeVector polyhedrons_in_range;
//   getPolyhedronsInRange(point, _max_ray_length * 2.0, polyhedrons_in_range);
//   for (const auto& p_now : polyhedrons_in_range){
//     PolyHedronPtr polyhedron_now = p_now.polyhedron_;
//     if (polyhedron_now->is_gate_ || polyhedron_now->is_rollbacked_)              continue;
//     if (getDistance(point, polyhedron_now->center_) > _max_ray_length + min_dis) continue;
//     // check every facet of polyhedron
//   }
// #endif
// }

// Find ray-plane intersection and check if point is inside the triangle
// return : (IF_INTERSECTION, intersection_point)
pair<bool, Eigen::Vector3d> SkeletonGenerator::findContactWithFacetInDirection
  (const FacetPtr &facet, const Eigen::Vector3d &point, const Eigen::Vector3d &direction){
  pair<bool, Eigen::Vector3d> plane_intersection = rayPlaneIntersection(point, direction,
                                                                        facet->plane_equation_(0),
                                                                        facet->plane_equation_(1),
                                                                        facet->plane_equation_(2),
                                                                        facet->plane_equation_(3));
  if (!plane_intersection.first)
    return make_pair(false, Eigen::Vector3d(0, 0, 0));
  if (!ifPointInTriangle(plane_intersection.second, facet->vertices_))
    return make_pair(false, Eigen::Vector3d(0, 0, 0));

  return make_pair(true, plane_intersection.second);
}

// return : (IF_INTERSECTION, intersection_point)
pair<bool, Eigen::Vector3d>
SkeletonGenerator::rayPlaneIntersection(const Eigen::Vector3d& rayOrigin, const Eigen::Vector3d& rayDirection,
                                        const double a, const double b, const double c, const double d){
  // Compute plane normal
  Eigen::Vector3d planeNormal(a, b, c);
  double denom = planeNormal.dot(rayDirection);
  if (std::abs(denom) < 1e-6) {
    return make_pair(false, Eigen::Vector3d(0, 0, 0));
  }
  // Compute intersection distance
  double t = -(planeNormal.dot(rayOrigin) + d) / denom;
  if (t < 0) { // intersection is behind the ray origin
    return make_pair(false, Eigen::Vector3d(0, 0, 0));
  }
  // Compute intersection coordinates
  Eigen::Vector3d intersection = rayOrigin + rayDirection * t;
  return make_pair(true, intersection);
}

bool SkeletonGenerator::ifPointInTriangle(const Eigen::Vector3d& point, const std::vector<VertexPtr>& vertices){
  if (vertices.size() != 3)
    return false;
  Eigen::Vector3d v0 = vertices[0]->position_;
  Eigen::Vector3d v1 = vertices[1]->position_;
  Eigen::Vector3d v2 = vertices[2]->position_;
  // Compute edge vectors of the triangle
  Eigen::Vector3d edge0 = v1 - v0;
  Eigen::Vector3d edge1 = v2 - v1;
  Eigen::Vector3d edge2 = v0 - v2;
  // Compute vectors from point to triangle vertices
  Eigen::Vector3d vp0 = point - v0;
  Eigen::Vector3d vp1 = point - v1;
  Eigen::Vector3d vp2 = point - v2;
  // Compute cross products
  Eigen::Vector3d c0 = edge0.cross(vp0);
  Eigen::Vector3d c1 = edge1.cross(vp1);
  Eigen::Vector3d c2 = edge2.cross(vp2);
  // Check if all cross products point in the same direction
  double dot0 = c0.dot(c1);
  double dot1 = c1.dot(c2);
  return (dot0 >= 0 && dot1 >= 0);
}

// true  opposite sides
// false same side
bool SkeletonGenerator::ifPtInIpsilateralOfPlane(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const FacetPtr facet){
  double d1 = facet->plane_equation_(0) * pt1.x() + facet->plane_equation_(1) * pt1.y() + facet->plane_equation_(2) * pt1.z() + facet->plane_equation_(3);
  double d2 = facet->plane_equation_(0) * pt2.x() + facet->plane_equation_(1) * pt2.y() + facet->plane_equation_(2) * pt2.z() + facet->plane_equation_(3);
  return !(std::signbit(d1) == std::signbit(d2));
}

Eigen::Vector3d SkeletonGenerator::transPointToBodyFrame(const Eigen::Vector3d& point_in_world){

  Eigen::Matrix4d inverse_trans_mat = Eigen::Matrix4d::Identity();
  inverse_trans_mat.block<2, 2>(0, 0) << cos(cur_yaw_), sin(cur_yaw_),
                                        -sin(cur_yaw_), cos(cur_yaw_);
  inverse_trans_mat.block<3, 1>(0, 3) = -cur_pos_;
  inverse_trans_mat(0, 3)             = -cur_pos_.x() * cos(cur_yaw_) - cur_pos_.y() * sin(cur_yaw_);
  inverse_trans_mat(1, 3)             =  cur_pos_.x() * sin(cur_yaw_) - cur_pos_.y() * cos(cur_yaw_);

  Eigen::Vector4d point_in_world_mat(point_in_world(0), point_in_world(1), point_in_world(2), 1.0);
  Eigen::Vector4d point_in_body_mat = inverse_trans_mat * point_in_world_mat;
  return point_in_body_mat.block<3, 1>(0, 0);
}

void SkeletonGenerator::initFacetsFromPolyhedron(PolyHedronPtr polyhedron){
  for (auto facet_vertices : facet_vertex_directions_){
    VertexPtr v0 = getVertexFromDirection(polyhedron, facet_vertices.at(0));
    VertexPtr v1 = getVertexFromDirection(polyhedron, facet_vertices.at(1));
    VertexPtr v2 = getVertexFromDirection(polyhedron, facet_vertices.at(2));
    v0->connected_vertices_.push_back(v1);
    v1->connected_vertices_.push_back(v2);
    v2->connected_vertices_.push_back(v0);
  }
}

// find facets whitch have all vertices in black_v_group
void SkeletonGenerator::findFacetsGroupFromVertices(PolyHedronPtr polyhedron, std::vector<VertexPtr> colli_v_group, std::vector<FacetPtr> &res){
  for (auto & cur_facet : polyhedron->facets_){
    bool good_facet = true;
    for (VertexPtr v_facet : cur_facet->vertices_){
      bool not_inclueded = true;
      for (VertexPtr v_group : colli_v_group){
        if (!v_group->is_critical_) continue;
        if (isSamePose(v_facet->position_, v_group->position_)){
          not_inclueded = false;
          break;
        }
      }
      if (not_inclueded){
        good_facet = false;
        break;
      }
    } // v_facet
    if (good_facet){
      // todo: z-axis bound check here, to be added later
      res.push_back(cur_facet);
      cur_facet->frontier_processed_ = true;
    }
  } // cur_face
}

void SkeletonGenerator::findNeighborFacets(std::vector<FacetPtr> facets){
  int num_facet = facets.size();
  for (int i = 0; i < num_facet; i++){
    const FacetPtr& f1 = facets.at(i);
    for (int j = i+1; j < num_facet; j++){
      const FacetPtr& f2 = facets.at(j);
      if (f1->neighbor_facets_.size() == 3) break;
      int same_vertex_cnt = 0;
      for (int m = 0; m < 3; m++)
        for (int n = 0; n < 3; n++)
          if (isSamePose(f1->vertices_.at(m)->position_, f2->vertices_.at(n)->position_)){
            same_vertex_cnt ++;
            break;
          }
      if (same_vertex_cnt == 2){
        f1->neighbor_facets_.push_back(f2);
        f2->neighbor_facets_.push_back(f1);
      }
    }
  }
}

void SkeletonGenerator::splitFrontier(PolyHedronPtr polyhedron, std::vector<FacetPtr> single_cluster,
                                      std::vector<PolyhedronFtrPtr> &res){
  res.clear();
  // step1. calculate mean normal of single_cluster
  if (single_cluster.empty()) return;
  if (single_cluster.size() <= _max_facets_grouped){
    Eigen::Vector3d       average_normal(0, 0, 0);
    std::vector<FacetPtr> pending_facets_for_ftr;
    for (FacetPtr cur_facet : single_cluster)
      average_normal += cur_facet->out_unit_normal_;
    average_normal.normalize();
    for (FacetPtr cur_facet : single_cluster){
      if (cur_facet->neighbor_facets_.size() >= 2){
        pending_facets_for_ftr.push_back(cur_facet);
        continue;
      }

      double angle = acos(average_normal.dot(cur_facet->out_unit_normal_));
      // angle too large, split this facet into one single frontier
      if (angle > M_PI / 2.5){
        std::vector<FacetPtr> single_facet;
        single_facet.push_back(cur_facet);
        PolyhedronFtrPtr ftr = std::make_shared<PolyhedronFtr>(single_facet, polyhedron);
        initSingleFrontier(ftr);
        res.push_back(ftr);
        continue;
      }
      pending_facets_for_ftr.push_back(cur_facet);
    }
    PolyhedronFtrPtr new_frontier = std::make_shared<PolyhedronFtr>(pending_facets_for_ftr, polyhedron);

    if (initSingleFrontier(new_frontier))
      res.push_back(new_frontier);
  } // single_cluster.size() <= _max_facets_grouped
  // If cluster size exceeds _max_facets_grouped, split into multiple frontiers (ftr chunking)
  else{
    for (FacetPtr f : single_cluster){
      if (f->is_visited_) continue;
      f->is_visited_ = true;
      Eigen::Vector3d avg_normal(0.0, 0.0, 0.0);
      std::vector<FacetPtr> small_group_facets;
      deque<FacetPtr>       pending_facets;
      pending_facets.push_back(f);

      while (!pending_facets.empty() && static_cast<int>(small_group_facets.size()) <_max_facets_grouped){
        FacetPtr cur_facet = pending_facets.front();
        pending_facets.pop_front();
        // re-calculate avg normal of small_group_facets
        if (!small_group_facets.empty()){
          if (acos(cur_facet->out_unit_normal_.dot(avg_normal)) > M_PI / _frontier_split_threshold) continue;
          avg_normal = (avg_normal * small_group_facets.size() + cur_facet->out_unit_normal_) /
                        (small_group_facets.size() + 1);
          avg_normal.normalize();
        }else
          avg_normal = cur_facet->out_unit_normal_;
        cur_facet->is_visited_ = true;
        small_group_facets.push_back(cur_facet);
        for (FacetPtr neighbor_facet : cur_facet->neighbor_facets_)
          if (!neighbor_facet->is_visited_)
            pending_facets.push_back(neighbor_facet);
      } // penign_facets
      PolyhedronFtrPtr new_frontier = std::make_shared<PolyhedronFtr>(small_group_facets, polyhedron);
      if (initSingleFrontier(new_frontier))
        res.push_back(new_frontier);
    }// single_cluster
  }
}

bool SkeletonGenerator::initSingleFrontier(PolyhedronFtrPtr cur_ftr){
  bool proj_center_found = false;

  double x0 = cur_ftr->avg_center_(0);
  double y0 = cur_ftr->avg_center_(1);
  double z0 = cur_ftr->avg_center_(2);
  double nx = cur_ftr->out_unit_normal_(0);
  double ny = cur_ftr->out_unit_normal_(1);
  double nz = cur_ftr->out_unit_normal_(2);

  for (FacetPtr cur_facet : cur_ftr->facets_){
    double a = cur_facet->plane_equation_(0);
    double b = cur_facet->plane_equation_(1);
    double c = cur_facet->plane_equation_(2);
    double d = cur_facet->plane_equation_(3);
    double t = -(a * x0 + b * y0 + c * z0 + d) / (a * nx + b * ny + c * nz);

    Eigen::Vector3d intersection =
             cur_ftr->avg_center_ + t * cur_ftr->out_unit_normal_;

    // Check if intersection is inside the triangle
    Eigen::Vector3d cross1 = (cur_facet->vertices_[1]->position_ - cur_facet->vertices_[0]->position_)
                              .cross(intersection - cur_facet->vertices_[0]->position_);
    Eigen::Vector3d cross2 = (cur_facet->vertices_[2]->position_ - cur_facet->vertices_[1]->position_)
                              .cross(intersection - cur_facet->vertices_[1]->position_);
    Eigen::Vector3d cross3 = (cur_facet->vertices_[0]->position_ - cur_facet->vertices_[2]->position_)
                              .cross(intersection - cur_facet->vertices_[2]->position_);
    if (std::signbit(cross1(0)) == std::signbit(cross2(0)) &&
        std::signbit(cross2(0)) == std::signbit(cross3(0)) &&
        std::signbit(cross3(0)) == std::signbit(cross1(0))){
      cur_ftr->proj_center_ = intersection;
      cur_ftr->proj_facet_  = cur_facet;
      cur_ftr->cos_theta_   = cur_ftr->out_unit_normal_.dot(cur_facet->out_unit_normal_) /
                              (cur_ftr->out_unit_normal_.norm() * cur_facet->out_unit_normal_.norm());
      proj_center_found = true;
      break;
    }
  }

  // Find the facet with smallest angular difference as the projection plane
  if (!proj_center_found){
    double min_angle = M_PI;
    FacetPtr best_facet = nullptr;
    for (FacetPtr f : cur_ftr->facets_){
      double angle =  acos(cur_ftr->out_unit_normal_.dot(f->out_unit_normal_));
      if (angle < min_angle){
        min_angle = angle;
        best_facet = f;
      }
    }
    if (best_facet == nullptr) return false;
    cur_ftr->proj_facet_  = best_facet;
    cur_ftr->proj_center_ = best_facet->center_;
  }

  // set vertices
  // Ensure each vertex is output only once
  for (FacetPtr cur_facet : cur_ftr->facets_){
    for (VertexPtr cur_vertex : cur_facet->vertices_){
      bool exist = false;
      for (VertexPtr v : cur_ftr->vertices_){
        if (isSamePose(cur_vertex->position_, v->position_)){
          exist = true;
          break;
        }
      }
      if (!exist)
        cur_ftr->vertices_.push_back(cur_vertex);
    }
  }
  return proj_center_found;
}

VertexPtr SkeletonGenerator::getVertexFromDirection(PolyHedronPtr polyhedron, const Eigen::Vector3d& direction){
  for (VertexPtr vertex : polyhedron->black_vertices_)
    if (isSamePose(direction, vertex->direction_in_unit_sphere_))
      return vertex;
  for (VertexPtr vertex : polyhedron->gray_vertices_)
    if (isSamePose(direction, vertex->direction_in_unit_sphere_))
      return vertex;
  for (VertexPtr vertex : polyhedron->white_vertices_)
    if (isSamePose(direction, vertex->direction_in_unit_sphere_))
      return vertex;
  return nullptr;
}

// -3: collision with virtual wall (axis-aligned-z)
// -2: collision not found within cut_off_length
// -1: collision is with map
//  0: collision is with the node of that index
// tuple <collision_point, collision_type, polyhedron_index>
std::tuple<Eigen::Vector3d, int, Eigen::Vector3d>
SkeletonGenerator::rayCast(Eigen::Vector3d orin_point, Eigen::Vector3d direction, double max_ray_length, double step_size){
  double          min_dis = 9999.9999;
  bool            is_collision_with_Poly = false;
  Eigen::Vector3d pt_collision(-9999, -9999, -9999);
  Eigen::Vector3d min_dis_node_index(-9999, -9999, -9999);
  // check with polyhedron
  if (has_init_polyhedron_kdtree_){
    PolyHedronKDTreeVector polyhedrons_in_range;
    getPolyhedronsInRange(orin_point, max_ray_length * 3.0, polyhedrons_in_range);
    for (const auto& p_now : polyhedrons_in_range){
      PolyHedronPtr polyhedron_now = p_now.polyhedron_;
      if (polyhedron_now->is_gate_)                                                continue;
      if (getDistance(orin_point, polyhedron_now->center_) > 2.0 * max_ray_length) continue;
      // check every facet of polyhedron
      for (const auto& facet : polyhedron_now->facets_){
        pair<bool, Eigen::Vector3d> intersection_result = findContactWithFacetInDirection(facet, orin_point, direction);
        if (intersection_result.first){
          double dis = getDistance(orin_point, intersection_result.second);
          if (dis < min_dis && dis < max_ray_length){
            min_dis = dis;
            pt_collision           = intersection_result.second;
            min_dis_node_index     = polyhedron_now->origin_center_;
            is_collision_with_Poly = true;
          }
        }
      }
    }
  }
  // check with map
  Eigen::Vector3d pt_check = orin_point + 0.1 * direction;
  double          length   = 0.0;
  pair<int, int>  inflate_res;
  while (length <= max_ray_length + 1e-2 && length <= min_dis){
    Eigen::Vector3d pt_check_body_frame = transPointToBodyFrame(pt_check);
    inflate_res = map_interface_->getRawInflateOccupancy(pt_check);
    if (inflate_res.first == global_belief::MapInterface::OCCUPIED){
      return std::make_tuple(pt_check, -1, min_dis_node_index);
    }
    if (pt_check_body_frame.z() < _local_z_min || pt_check_body_frame.z() > _local_z_max){
      return std::make_tuple(pt_check - 0.1 * direction, -3, min_dis_node_index);
    }
    pt_check += step_size * direction;
    length   += step_size;
  }

  if (is_collision_with_Poly) return std::make_tuple(pt_collision, 0, min_dis_node_index);

  if (inflate_res.first == global_belief::MapInterface::OCCUPIED &&
      inflate_res.second == global_belief::MapInterface::FREE)
    return std::make_tuple(orin_point + max_ray_length * direction / 1.5, -2, min_dis_node_index);

  return std::make_tuple(orin_point + max_ray_length * direction, -2, min_dis_node_index);
}

bool SkeletonGenerator::searchPathInRawMap(Eigen::Vector3d start_point, Eigen::Vector3d end_point,
                                           std::vector<Eigen::Vector3d>& path, double step_size, bool consider_uk, bool only_directly_vis = false){
#ifdef _MAP_TYPE_MAP_INTERFACE
  if ((start_point - end_point).norm() < 0.1){
    path = {start_point, end_point};
    return false;
  }

  if (!map_interface_->isInLocalMap(start_point) || !map_interface_->isInLocalMap(end_point)) {
    INFO_MSG_RED("[Skele] | search in RawMap, start or end point is not in local map, return false");
    return false;
  }

  if (only_directly_vis) {
    if (map_interface_->isVisible(start_point, end_point)) {
      path.push_back(start_point); path.push_back(end_point);
      return true;
    }
    return false;
  }
  bool res = false;
  try {
    if (map_interface_->getRawInflateOccupancy(start_point).second == global_belief::MapInterface::OCCUPIED ||
        map_interface_->getRawInflateOccupancy(end_point).second == global_belief::MapInterface::OCCUPIED)
      return false;
    if (consider_uk) res = map_interface_->searchPathConsiderUKRegion(start_point, end_point, path, step_size);
    else             res = map_interface_->searchPath(start_point, end_point, path, step_size);
  }catch (std::exception& e) {
    ROS_ERROR_STREAM("[Skele] | searchPathInRawMap, exception: " << e.what());
    res = false;
  }
  return res;
#endif
}

void SkeletonGenerator::drawFacets(std::vector<FacetPtr> facets, Eigen::Vector4d color,
                                   int id,  std::string ns, ros::Time stamp,visualization_msgs::Marker& marker){
  marker.header.frame_id = "world";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.color.a = color(3);

  for (auto facet : facets){
    for (int i = 0; i < 3; i++){
      geometry_msgs::Point point1, point2;
      point1.x = facet->vertices_.at(i)->position_(0);
      point1.y = facet->vertices_.at(i)->position_(1);
      point1.z = facet->vertices_.at(i)->position_(2);
      point2.x = facet->vertices_.at((i+1)%3)->position_(0);
      point2.y = facet->vertices_.at((i+1)%3)->position_(1);
      point2.z = facet->vertices_.at((i+1)%3)->position_(2);
      marker.points.push_back(point1);
      marker.points.push_back(point2);
    }
  }
}

// color : r, g, b, a
void SkeletonGenerator::drawPoints(const std::vector<Eigen::Vector3d> & points, double pt_scale, Eigen::Vector4d color,
                                   int id, std::string ns, ros::Time stamp, visualization_msgs::Marker& marker){
  marker.header.frame_id = "world";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type   = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
  marker.color.r = color(0); marker.color.g = color(1); marker.color.b = color(2);
  marker.color.a = color(3);
  for (auto & point : points){
    geometry_msgs::Point p;
    p.x = point(0); p.y = point(1); p.z = point(2);
    marker.points.push_back(p);
  }
}

void SkeletonGenerator::visualizeAllEdges(){
  auto eigen2geometry = [](const Eigen::Vector3d& vec) -> geometry_msgs::Point{
    geometry_msgs::Point pt;
    pt.x = vec(0); pt.y = vec(1); pt.z = vec(2);
    return pt;
  };

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker, marker_force_edge;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "edges";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03; marker.scale.y = 0.03; marker.scale.z = 0.03;
  marker.color.r = 1.0;  marker.color.g = 1.0;  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker_force_edge = marker;
  marker_force_edge.header.frame_id = "world";
  marker_force_edge.ns = "force_edge";
  marker_force_edge.color.r = 1.0; marker_force_edge.color.g = 0.0; marker_force_edge.color.b = 0.0; marker_force_edge.color.a = 1.0;
  marker_force_edge.scale.x = marker_force_edge.scale.y = marker_force_edge.scale.z = 0.05;

  std::unordered_map<Eigen::Vector3d, bool, Vector3dHash> visited_node;
  std::unordered_map<Eigen::Vector3d, bool, Vector3dHash> calculated_node;
  std::deque<PolyHedronPtr> polyhedron_queue;

  for (const auto& p : polyhedron_map_){
    if (visited_node.find(p.second->center_)!= visited_node.end()) continue;
    visited_node[p.second->center_] = true;
    polyhedron_queue.push_back(p.second);
    while (!polyhedron_queue.empty()){
      PolyHedronPtr cur_poly = polyhedron_queue.front();
      polyhedron_queue.pop_front();
      for (const auto& edge : cur_poly->edges_){
        if (visited_node.find(edge.poly_nxt_->center_) == visited_node.end()){
          polyhedron_queue.push_back(edge.poly_nxt_);
          visited_node[edge.poly_nxt_->center_] = true;
        }
        if (calculated_node.find(edge.poly_nxt_->center_) == calculated_node.end()){
          if (edge.is_force_connected_) {
            marker_force_edge.points.push_back(eigen2geometry(cur_poly->center_));
            marker_force_edge.points.push_back(eigen2geometry(edge.poly_nxt_->center_));
          }else {
            marker.points.push_back(eigen2geometry(cur_poly->center_));
            marker.points.push_back(eigen2geometry(edge.poly_nxt_->center_));
          }
        }
      }
      calculated_node[cur_poly->center_] = true;
    }
  }
  marker_array.markers.push_back(marker);
  marker_array.markers.push_back(marker_force_edge);
  skeleton_vis_pub_.publish(marker_array);
}

void SkeletonGenerator::refreshLoadedMapVisualization() {
  visualization_msgs::MarkerArray clear_array;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = "world";
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_array.markers.push_back(clear_marker);
  skeleton_vis_pub_.publish(clear_array);

  std::vector<PolyHedronPtr> polyhedrons;
  getAllPolys(polyhedrons);
  if (polyhedrons.empty()) return;

  // visualizePolygons(polyhedrons);
  // ros::Duration(0.001).sleep();
  visualizeAllEdges();
  ros::Duration(0.001).sleep();
  visualizePolyBelongsToArea();
}

void SkeletonGenerator::visualizePolyhedronVertices(PolyHedronPtr polyhedron){
  visualizeVertices(polyhedron->black_vertices_, 0);
  ros::Duration(0.001).sleep();
  visualizeVertices(polyhedron->white_vertices_, 1);
  ros::Duration(0.001).sleep();
  visualizeVertices(polyhedron->gray_vertices_, 2);
  ros::Duration(0.001).sleep();
}

void SkeletonGenerator::visualizePolyhedroneInRange(Eigen::Vector3d center_pt, double radius){
  PolyHedronKDTreeVector     polyhedrons_in_range;
  std::vector<PolyHedronPtr> polyhedrons_to_visualize;
  getPolyhedronsInRange(center_pt, radius, polyhedrons_in_range);
  for (const auto& p_now : polyhedrons_in_range)
    polyhedrons_to_visualize.push_back(p_now.polyhedron_);

  visualizePolygons(polyhedrons_to_visualize);
  ros::Duration(0.001).sleep();

  int marker_id = 0;
  int poly_num  = polyhedrons_in_range.size();
  for (const auto& cur_poly : polyhedrons_in_range){
    visualizeFacets(cur_poly.polyhedron_->facets_, marker_id);
    ros::Duration(0.001).sleep();
    marker_id ++;
  }

  ros::Duration(0.001).sleep();
  visualizeAllEdges();
}

void SkeletonGenerator::visualizePolygons(std::vector<PolyHedronPtr> polyhedrons){
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker_center, marker_orin_center;
  std::vector<Eigen::Vector3d> centers, orin_centers;
  for (auto polyhedron : polyhedrons){
    if (polyhedron->is_gate_) continue;
    centers.push_back(polyhedron->center_);
    orin_centers.push_back(polyhedron->origin_center_);
  }

  drawPoints(centers, 0.2, Eigen::Vector4d(1.0, 0.078, 0.576, 1.0), 0, "polyhedron_center", ros::Time::now(), marker_center);
  drawPoints(orin_centers, 0.2, Eigen::Vector4d(0.0,1.0, 1.0, 1.0), 0, "polyhedron_orin_center", ros::Time::now(), marker_orin_center);
  marker_array.markers.push_back(marker_center);
  marker_array.markers.push_back(marker_orin_center);
  skeleton_vis_pub_.publish(marker_array);
}

void SkeletonGenerator::visualizeFrontiers(std::vector<PolyhedronFtrPtr> ftrs){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    ros::Time stamp = ros::Time::now();
    for (auto ftr : ftrs){
      drawFacets(ftr->facets_, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), 0, "frontier_facet", stamp, marker);
      marker_array.markers.push_back(marker);
    }
    skeleton_vis_pub_.publish(marker_array);
}

void SkeletonGenerator::visualizeFacets(const std::vector<FacetPtr> &facets, const int &id){
  visualization_msgs::MarkerArray marker_array_normal, marker_array_frontier;
  visualization_msgs::Marker marker_normal, marker_frontier;

  marker_normal.header.frame_id = marker_frontier.header.frame_id = "world";
  marker_normal.header.stamp = marker_frontier.header.stamp = ros::Time::now();
  marker_normal.ns  = "normal_facet"; marker_frontier.ns = "frontier_facet";
  marker_normal.id = marker_frontier.id = id;
  marker_normal.type = marker_frontier.type = visualization_msgs::Marker::LINE_LIST;
  marker_normal.action = marker_frontier.action = visualization_msgs::Marker::ADD;
  marker_normal.pose.orientation.w = marker_frontier.pose.orientation.w = 1.0;
  marker_normal.scale.x = marker_frontier.scale.x = 0.01;
  marker_normal.scale.y = marker_frontier.scale.y =0.01;
  marker_normal.scale.z = marker_frontier.scale.z = 0.01;
  marker_normal.color.a = 0.8; marker_frontier.color.a = 1.0;

  // gray normal facets
  marker_normal.color.r = 0.5; marker_normal.color.g = 0.5; marker_normal.color.b = 0.5;
  // red frontier facets
  marker_frontier.color.r = 1.0; marker_frontier.color.g = 0.0; marker_frontier.color.b = 0.0;

  for (const auto& facet : facets){
    for (int i = 0; i < 3; i++){
      geometry_msgs::Point point1, point2;
      point1.x = facet->vertices_.at(i)->position_(0);
      point1.y = facet->vertices_.at(i)->position_(1);
      point1.z = facet->vertices_.at(i)->position_(2);
      point2.x = facet->vertices_.at((i+1)%3)->position_(0);
      point2.y = facet->vertices_.at((i+1)%3)->position_(1);
      point2.z = facet->vertices_.at((i+1)%3)->position_(2);
      if (facet->frontier_processed_){
        marker_frontier.points.push_back(point1);
        marker_frontier.points.push_back(point2);
      }
      else{
        marker_normal.points.push_back(point1);
        marker_normal.points.push_back(point2);
      }
    }
  }
  marker_array_normal.markers.push_back(marker_normal);
  skeleton_vis_pub_.publish(marker_array_normal);
  ros::Duration(0.001).sleep();
  marker_array_frontier.markers.push_back(marker_frontier);
  skeleton_vis_pub_.publish(marker_array_frontier);
}

void SkeletonGenerator::visualizeVertices(const std::vector<VertexPtr> &vertices, const int &id){
  if (vertices.empty()) return;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "vertex";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::DELETE;
  marker_array.markers.push_back(marker);
  skeleton_vis_pub_.publish(marker_array);
  ros::Duration(0.01).sleep();

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "vertex";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  if (vertices.at(0)->type_ == Vertex::BLACK){
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }if (vertices.at(0)->type_ == Vertex::WHITE){
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }else{
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }

  for (const auto& vertex : vertices){
    geometry_msgs::Point point;
    point.x = vertex->position_(0);
    point.y = vertex->position_(1);
    point.z = vertex->position_(2);
    marker.points.push_back(point);
  }
  marker_array.markers.push_back(marker);
  skeleton_vis_pub_.publish(marker_array);
}

void SkeletonGenerator::visualizeLocalRange(){

}

void SkeletonGenerator::visualizeSphereSampleDirections(){
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  drawPoints(sphere_sample_directions_, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 0, "sphere_sample_directions", ros::Time::now(), marker);
  marker_array.markers.push_back(marker);
  skeleton_vis_pub_.publish(marker_array);
}

void SkeletonGenerator::visualizePolyBelongsToArea() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "poly_belongs_to_area";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0;
  int i = 0;
  for (const auto& p : polyhedron_map_) {
    marker.pose.position.x = p.second->center_.x();
    marker.pose.position.y = p.second->center_.y();
    marker.pose.position.z = p.second->center_.z();
    marker.text = std::to_string(p.second->area_id_);
    marker.id = i++;
    marker_array.markers.push_back(marker);
  }
  skeleton_vis_pub_.publish(marker_array);
}
