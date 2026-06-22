//
// Created by gwq on 25-2-27.
//

#ifndef SKELETON_GENERATION_H
#define SKELETON_GENERATION_H

#include <ros/ros.h>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <map_interface/map_interface.hpp>
#include <std_msgs/Empty.h>

#include "../include/scene_graph/skeleton_astar.h"
#include "../include/scene_graph/skeleton_cluster.h"
#include "../include/scene_graph/data_structure.h"
#include "../include/scene_graph/ikd_Tree.h"
#include "../libs/quickhull/QuickHull.hpp"


#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_CYAN(str)   do {std::cout << "\033[36m" << str << "\033[0m" << std::endl; } while(false)

// you can choose single map type by uncommenting one of the following macros
// #define _MAP_TYPE_POINT_CLOUD
// #define _MAP_TYPE_OCCUPANCY_MAP
#define _MAP_TYPE_MAP_INTERFACE

using PolyHedronKDTree = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_PolyhedronType>;
using PolyHedronKDTreeVector = PolyHedronKDTree::PointVector;

using PolyHedronKDTree_FixedCenter = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>;
using PolyHedronKDTree_FixedCenterVector = PolyHedronKDTree_FixedCenter::PointVector;

using Vector3dKDTree = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_Vectoe3dType>;
using Vector3dKDTreeVector = Vector3dKDTree::PointVector;

using skeleton_gen::KD_TREE;

/**
 * Skeleton generator for free-space decomposition.
 *
 * Generates a topological skeleton of free space by expanding polyhedra
 * into unknown regions via sphere sampling, frontier detection, and
 * connectivity verification. Each polyhedron node in the skeleton graph
 * represents a collision-free convex region. Interfaces with A* search
 * on the skeleton graph and area (room) clustering.
 */
class SkeletonGenerator {
  public:
    typedef std::shared_ptr<SkeletonGenerator> Ptr;
    typedef std::unique_ptr<SkeletonGenerator> UPtr;
    SpectralCluster::Ptr spectral_cluster_; ///< Spectral clustering for area detection
    AreaHandler::Ptr area_handler_;         ///< Area (room) handler

    SkeletonGenerator(ros::NodeHandle& nh, ego_planner::MapInterface::Ptr &map_interface);
    ~SkeletonGenerator();
    /**
     * Check if the skeleton generator is initialized and ready.
     *
     * @return True if ready
     */
    bool ready() const;
    /**
     * Get the number of skeleton nodes (polyhedra).
     *
     * @return Node count [--]
     */
    int  getNodeNum() const;

    /**
     * Expand the skeleton by generating a new polyhedron at the frontier.
     *
     * Core function: samples points on a sphere around the start point,
     * checks collision to build black/white vertices, and constructs a
     * new convex polyhedron.
     *
     * @param[in] start_point  Expansion start position [m]
     * @param[in] yaw          Robot yaw for body-frame checks [rad]
     * @return True if a new polyhedron was generated
     */
    bool expandSkeleton(const Eigen::Vector3d &start_point, double yaw);
    /**
     * Find the mount (current) polyhedron containing the robot position.
     *
     * @param[in] cur_pos            Current robot position [m]
     * @param[in] ignore_connectivity  Whether to skip connectivity check [--]
     * @return Mounted polyhedron (nullptr if not found)
     */
    PolyHedronPtr mountCurTopoPoint(const Eigen::Vector3d& cur_pos, bool ignore_connectivity);
    /**
     * Get the polyhedron that is the parent of an object at the given position.
     *
     * @param[in] cur_pos  Object position [m]
     * @return Parent polyhedron
     */
    PolyHedronPtr getObjFatherNode(const Eigen::Vector3d& cur_pos);
    /**
     * Get the frontier polyhedron (gate) closest to the given position.
     *
     * @param[in] cur_pos  Position [m]
     * @return Frontier polyhedron
     */
    PolyHedronPtr getFrontierTopo(const Eigen::Vector3d& cur_pos);
    /**
     * Get all polyhedron nodes in the skeleton.
     *
     * @param[out] polyhedrons  Output vector of all polyhedra
     */
    void getAllPolys(std::vector<PolyHedronPtr>& polyhedrons);
    /**
     * Update the mount polyhedron at the current robot position.
     *
     * @param[in] cur_pos  Current robot position [m]
     */
    void updateMountedTopoPoint(const Eigen::Vector3d& cur_pos);
    /**
     * Dense check: verify if skeleton expansion is needed and perform it.
     *
     * @param[in] cur_pos  Current robot position [m]
     * @param[in] yaw      Current robot yaw [rad]
     * @return True if expansion was performed
     */
    bool doDenseCheckAndExpand(const Eigen::Vector3d &cur_pos, double yaw);
    /**
     * Run A* search on the skeleton graph between two positions.
     *
     * @param[in]  start_point    Start position [m]
     * @param[in]  end_point      End position [m]
     * @param[out] path           Output path waypoints [m]
     * @param[in]  add_input_pts  Add start/end points to the path [--]
     * @return Path length [m]
     */
    double astarSearch(const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point,
                       std::vector<Eigen::Vector3d>& path, bool add_input_pts);
    /**
     * Run A* search between two polyhedron nodes.
     *
     * @param[in]  start_polyhedron  Start polyhedron
     * @param[in]  end_polyhedron    End polyhedron
     * @param[out] path              Output path waypoints [m]
     * @return Path length [m]
     */
    double astarSearch(const PolyHedronPtr start_polyhedron, const PolyHedronPtr end_polyhedron,
                       std::vector<Eigen::Vector3d>& path);
    void resetForMapLoad();
    bool registerLoadedPolyhedron(const PolyHedronPtr& polyhedron);
    void finishMapLoad();

    // mutex for skeleton
    void lock(){mutex_.lock();};
    void unlock(){mutex_.unlock();};

    std::vector<PolyHedronPtr> cur_iter_polys_;
    PolyHedronPtr              cur_iter_first_poly_{nullptr};
    void refreshLoadedMapVisualization();
    void visualizePolyBelongsToArea();

  private:
    enum pointCollisionType{
      FREE = 0,
      OCCUPIED = 1,
      UNKNOWN = 2,
      CONTACT_POLYGON = 3,
    };
    struct Vector3dHash {
      std::size_t operator()(const Eigen::Vector3d& vector) const {
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };
    struct polyhedronHash {
      std::size_t operator()(const PolyHedronPtr polyhedron) const {
        Eigen::Vector3d vector = polyhedron->origin_center_;
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };
    static bool compareFrontier(PolyhedronFtrPtr f1, PolyhedronFtrPtr f2) {
      return f1->area_size_ > f2->area_size_;
    };
    std::mutex mutex_;

    //  ------- ROS related -------
    ros::NodeHandle nh_;
    ros::Publisher  skeleton_vis_pub_;
    ros::Subscriber map_inflate_sub_, cmd_sub_;

    //  ------- Utils -------
    ego_planner::MapInterface::Ptr      map_interface_;
    skeleton_astar::SkeletonAstar::Ptr  skeleton_astar_;

    //  ------- Parameters -------
    double _local_x_max, _local_x_min, _local_y_max, _local_y_min, _local_z_max, _local_z_min; ///< Local update range bounds [m]
    Eigen::Vector3d _local_range_min, _local_range_max; ///< Body-frame update range [m]
    int _map_type;                              ///< Map type: 0=point cloud, 1=occupancy, 2=map interface
    bool _is_simulation;                        ///< Whether running in simulation [--]
    double _frontier_creation_threshold;        ///< Min edge distance to consider as frontier [m]
    double _frontier_jump_threshold;            ///< Max jump distance for frontier grouping [m]
    double _frontier_split_threshold;           ///< Angle threshold for splitting frontiers [rad]
    int _min_flowback_creation_threshold;       ///< Min contact vertices for flowback creation [--]
    double _min_flowback_creation_radius_threshold; ///< Min radius for flowback creation [m]
    double _min_node_radius;                    ///< Min avg vertex-center distance to keep node [m]
    double _min_node_dense_radius;              ///< Min radius for dense node check [m]
    double _search_margin;                      ///< Ray-to-point search margin [m]
    double _max_ray_length;                     ///< Max raycast length [m]
    double _max_expansion_ray_length;           ///< Max expansion ray length (creates midpoint) [m]
    double _max_height_diff;                    ///< Max height difference for node absorption [m]
    int _sampling_density, _sampling_level;     ///< Sphere sampling parameters [--]
    int _max_facets_grouped;                    ///< Max facets per frontier group [--]
    double _resolution;                         ///< Map and raycast resolution [m]
    double _truncated_z_high;                   ///< High Z truncation for visualization [m]
    double _truncated_z_low;                    ///< Low Z truncation for visualization [m]
    double _expand_time_limit;                  ///< Expand time limit per call [s]

    /* ------------------ Development Tune ------------------ */
    bool _debug_mode;
    bool _bad_loop;

    // Visualize only the final result or the expansion process
    bool _visualize_final_result_only;
    // Visualize all or only the newest polyhedron
    bool _visualize_all;
    // Visualize outwards normal for each frontier
    bool _visualize_outwards_normal;
    // Visualize neighborhood facets for each frontier
    bool _visualize_nbhd_facets;
    // Visualize only_black polygon or black_and_white polygon
    bool _visualize_black_polygon;

    /* ------------------ Judgement flags ------------------ */
    bool has_init_polyhedron_kdtree_{false};

    //  ------- basic data -------
    PolyHedronPtr   mount_polyhedron_, last_mount_polyhedron_; ///< Current/last mounted polyhedron
    Eigen::Vector3d local_box_min_, local_box_max_;             ///< Local skeleton update range in world frame [m]
    double          cur_yaw_;                                   ///< Current robot yaw [rad]
    Eigen::Vector3d cur_pos_;                                   ///< Current robot position [m]
    std::unordered_map<Eigen::Vector3d, PolyHedronPtr, Vector3dHash> polyhedron_map_; ///< All polyhedra mapped by centroid

    KD_TREE<skeleton_gen::ikdTree_PolyhedronType>::Ptr               polyhedron_kd_tree_;
    KD_TREE<skeleton_gen::ikdTree_PolyhedronType_FixedCenter>::Ptr   polyhedron_kd_tree_fixed_center_;
    KD_TREE<skeleton_gen::ikdTree_Vectoe3dType>::Ptr                 remain_candidate_facet_expand_kd_tree_;
    Vector3dKDTreeVector                                             remain_candidate_facet_expand_pts_;

    std::vector<Eigen::Vector3d>              sphere_sample_directions_;                // directions for sphere sampling
    std::vector<quickhull::Vector3<double>>   sphere_sample_directions_qh_;             // same as above, for quickhull
    std::vector<std::vector<Eigen::Vector3d>> facet_vertex_directions_;                 // vertex samples for each facet
    deque<PolyhedronFtrPtr>                   expand_pending_frontiers_;                // frontiers waiting for expansion

    // temp data
    // std::unordered_map<PolyHedronPtr, PolyHedronPtr, polyhedronHash>   cur_iter_polyhedrons_;    // 当前轮次迭代的多面体，用作loop back检查

    // ROS functions
    void cmdCallback(const std_msgs::Empty::ConstPtr &msg);

    void getROSParams();
    void sampleUnitSphere();

    // polyhedron generation processes
    void adjustExpandStartPt(Eigen::Vector3d &start_point);
    bool initNewPolyhedron(PolyHedronPtr new_polyhedron);
    void initFacetVerticesDirection();
    void generatePolyVertices(PolyHedronPtr poly);
    void centralizePolyhedronCoord(PolyHedronPtr polyhedron);
    double getRadiusOfPolyhedron(PolyHedronPtr polyhedron);
    pair<bool, Eigen::Vector3d> checkIfContainedByAnotherPolyhedron(PolyHedronPtr polyhedron);
    void initFacetsFromPolyhedron(PolyHedronPtr polyhedron);
    void findFacetsGroupFromVertices(PolyHedronPtr polyhedron, std::vector<VertexPtr> colli_v_group, std::vector<FacetPtr> &res);
    void findNeighborFacets(std::vector<FacetPtr> facets);
    void splitFrontier(PolyHedronPtr polyhedron, std::vector<FacetPtr> single_cluster, std::vector<PolyhedronFtrPtr> &res);
    bool initSingleFrontier(PolyhedronFtrPtr cur_ftr);
    void verifyFrontier(PolyhedronFtrPtr ftr);
    void adjustFrontier(PolyhedronFtrPtr ftr);
    bool processAValidFrontier(PolyhedronFtrPtr cur_ftr);
    void generateFrontiers(PolyHedronPtr polyhedron);

    // topological functions
    void findNewTopoConnection(PolyHedronPtr polyhedron);
    void findLoopbackConnectionFromCandidate(PolyHedronPtr polyhedron);
    bool checkConnectivityBetweenPolyhedrons(PolyHedronPtr p1, PolyHedronPtr p2);

    // objects functions


    template<typename T>
    void readParam(std::string param_name, T &param_val, T default_val);
    bool checkInBoundingBox(const Eigen::Vector3d &point);
    bool checkInLocalUpdateRange(const Eigen::Vector3d &point);
    int  checkIfOnLocalFloorOrCeil(const Eigen::Vector3d &point);
    bool checkIfPolyhedronTooDense(const Eigen::Vector3d &center_pt);                 // 检查待生成多面体的中心是否距离其他多面体太近
    void getPolyhedronsInRange(const Eigen::Vector3d& pt, const double &radius, PolyHedronKDTreeVector & polyhedrons_in_range);
    void getPolyhedronsInRangeWithFixedCenter
      (const Eigen::Vector3d& pt, const double &radius, PolyHedronKDTree_FixedCenterVector & polyhedrons_in_range);
    void getPolyhedronsNNearestWithFixedCenter
      (const Eigen::Vector3d& pt, const int &k, PolyHedronKDTree_FixedCenterVector & polyhedrons_nearest);

    void getCandidateNxtPosInRange(const Eigen::Vector3d& pt, const double &radius, Vector3dKDTreeVector& candidate_nxt_pos_in_range);

    template<typename T>
    void getItemsInRangeAndSortByDistance(const Eigen::Vector3d& pt, const double &radius, std::vector<T, Eigen::aligned_allocator<T>>& polyhedrons_in_range);

    void recordNewPolyhedron(PolyHedronPtr polyhedron);
    /**
     * Raycast from a point in a direction, checking for obstacle contact.
     *
     * @param[in]  orin_point      Ray origin [m]
     * @param[in]  direction       Ray direction (not necessarily normalized)
     * @param[in]  max_ray_length  Maximum ray length [m]
     * @param[in]  step_size       Raycast step size [m]
     * @return Tuple of (hit_position [m], hit_type, hit_normal [--])
     */
    tuple<Eigen::Vector3d, int, Eigen::Vector3d> rayCast(Eigen::Vector3d orin_point, Eigen::Vector3d direction, double max_ray_length, double step_size);
    /**
     * Search for a collision-free path in the raw occupancy map.
     *
     * @param[in]  start_point     Start position [m]
     * @param[in]  end_point       End position [m]
     * @param[out] path            Output path waypoints [m]
     * @param[in]  step_size       A* step size [m]
     * @param[in]  consider_uk     Treat unknown as traversable [--]
     * @param[in]  only_directly_vis  Only use direct visibility check [--]
     * @return True if a path was found
     */
    bool searchPathInRawMap(Eigen::Vector3d start_point, Eigen::Vector3d end_point, std::vector<Eigen::Vector3d> &path, double step_size, bool
                            consider_uk, bool only_directly_vis);
    /**
     * Get the vertex of a polyhedron in a given direction.
     *
     * @param[in] polyhedron  Polyhedron to search
     * @param[in] direction   Query direction [--]
     * @return Vertex pointer, or nullptr if not found
     */
    VertexPtr getVertexFromDirection(PolyHedronPtr polyhedron, const Eigen::Vector3d &direction);

    static inline double getDistance(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2) { return (point1 - point2).norm(); }
    inline bool isSamePose(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2) { return (point1 - point2).squaredNorm() < 1e-4; }
    Eigen::Vector3d transPointToBodyFrame(const Eigen::Vector3d &point_in_world);

    // collision check with facets

    /**
     * Find the contact point of a ray with a facet.
     *
     * @param[in] facet     Facet to check
     * @param[in] point     Ray origin [m]
     * @param[in] direction Ray direction [--]
     * @return Pair of (hit flag [--], hit position [m])
     */
    pair<bool, Eigen::Vector3d> findContactWithFacetInDirection(const FacetPtr &facet, const Eigen::Vector3d &point, const Eigen::Vector3d &direction);
    static inline pair<bool, Eigen::Vector3d> rayPlaneIntersection (const Eigen::Vector3d& rayOrigin,
                                                                    const Eigen::Vector3d& rayDirection,
                                                                    double a, double b, double c, double d);
    static inline bool ifPointInTriangle(const Eigen::Vector3d &point, const std::vector<VertexPtr> &vertices);
    static inline bool ifPtInIpsilateralOfPlane(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const FacetPtr facet);

    void visualizePolyhedroneInRange(Eigen::Vector3d center_pt, double radius);
    void visualizePolygons(std::vector<PolyHedronPtr> polyhedrons);
    void visualizeFrontiers(std::vector<PolyhedronFtrPtr> ftrs);
    void visualizeFacets(const std::vector<FacetPtr> &facets, const int &id);
    void drawFacets(std::vector<FacetPtr> facets, Eigen::Vector4d color, int id,
                    std::string ns, ros::Time stamp, visualization_msgs::Marker &marker);
    void drawPoints(const std::vector<Eigen::Vector3d> & points, double pt_scale, Eigen::Vector4d color,
                    int id, std::string ns, ros::Time stamp, visualization_msgs::Marker &marker);

    void visualizePolyhedronVertices(PolyHedronPtr polyhedron);
    void visualizeAllEdges();
    void visualizeLocalRange();

    // Debug vis
    void visualizeVertices(const std::vector<VertexPtr> &vertices, const int &id);
    void visualizeSphereSampleDirections();
};

#endif //SKELETON_GENERATION_H
