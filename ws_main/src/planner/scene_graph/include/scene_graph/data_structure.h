//
// Created by gwq on 25-2-27.
//
# ifndef SKELETON_GENERATION_DATA_STRUCTURE_HPP
# define SKELETON_GENERATION_DATA_STRUCTURE_HPP

#include <ros/ros.h>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <scene_graph/EncodeMask.h>

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

class SkeletonGenerator;
class ObjectFactory;
struct ObjectNode;
struct ObjectEdge;

class Vertex;
class Facet;
class Polyhedron;
class PolyhedronFtr;
class Edge;
class PolyhedronCluster;

typedef std::shared_ptr<SkeletonGenerator> SkeletonGeneratorPtr;
typedef std::shared_ptr<ObjectNode>        ObjectNodePtr;

typedef std::shared_ptr<Vertex>        VertexPtr;
typedef std::shared_ptr<Facet>         FacetPtr;
typedef std::shared_ptr<Polyhedron>    PolyHedronPtr;
typedef std::shared_ptr<PolyhedronFtr> PolyhedronFtrPtr;
typedef std::shared_ptr<Edge>          EdgePtr;

/**
 * @class ColorGenerator
 * @brief A class to generate vibrant, highly distinguishable colors from integer IDs.
 */
class ColorGenerator {
public:
  /**
   * @brief Generates a color from an integer ID.
   * @param id The integer object ID.
   * @return Eigen::Vector3d An (r, g, b) vector with components in the range [0, 1].
   */
  static Eigen::Vector3d getColorById(int id) {
    // Use the golden ratio to distribute hues for maximum visual separation
    const double golden_ratio_conjugate = 0.61803398875;
    double hue = fmod(id * golden_ratio_conjugate, 1.0);

    // Set high saturation and value for vibrancy
    HSV hsv;
    hsv.h = hue * 360.0; // Hue in degrees [0, 360]
    hsv.s = 0.9;         // Saturation in [0, 1]
    hsv.v = 0.95;        // Value in [0, 1]

    return hsvToRgb(hsv);
  }

private:
  // Helper structure for HSV color
  struct HSV {
    double h, s, v; // h:[0, 360], s:[0, 1], v:[0, 1]
  };

  // Internal helper function to convert HSV to RGB
  static Eigen::Vector3d hsvToRgb(HSV hsv) {
    Eigen::Vector3d rgb;
    double c = hsv.v * hsv.s;
    double h_prime = fmod(hsv.h / 60.0, 6);
    double x = c * (1 - fabs(fmod(h_prime, 2) - 1));
    double m = hsv.v - c;

    if (0 <= h_prime && h_prime < 1) {
      rgb << c, x, 0;
    } else if (1 <= h_prime && h_prime < 2) {
      rgb << x, c, 0;
    } else if (2 <= h_prime && h_prime < 3) {
      rgb << 0, c, x;
    } else if (3 <= h_prime && h_prime < 4) {
      rgb << 0, x, c;
    } else if (4 <= h_prime && h_prime < 5) {
      rgb << x, 0, c;
    } else if (5 <= h_prime && h_prime < 6) {
      rgb << c, 0, x;
    } else {
      rgb << 0, 0, 0;
    }

    rgb.array() += m;
    return rgb;
  }
};


/**
 * Edge between object nodes in the scene graph.
 *
 * Connects an object to either a skeleton polyhedron or another object,
 * forming the object-skeleton graph structure.
 */
struct ObjectEdge {
  enum EdgeType {
    UNKNOWN = 0,
    WITH_SKELETON = 1,
    WITH_OBJECT = 2
  } father_type{UNKNOWN};
  std::string edge_description;
  PolyHedronPtr polyhedron_father{nullptr};
  ObjectNodePtr object_father{nullptr};
  std::vector<ObjectNodePtr> object_child;
};

/**
 * Object node in the scene graph.
 *
 * Represents a detected semantic object with its position, point cloud,
 * OBB (oriented bounding box), label, CLIP feature, and detection
 * filtering state machine.
 */
struct ObjectNode {
  typedef std::shared_ptr<ObjectNode> Ptr;
  int id;                     ///< Unique object identifier [--]
  std::string label;          ///< Semantic label
  double conf;                ///< Detection confidence [0, 1]
  Eigen::VectorXd label_feature; ///< CLIP / semantic feature vector (512-d)
  Eigen::Vector3d pos;        ///< Object centroid [m]
  Eigen::Vector3d color;      ///< Visualization color [0, 1]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; ///< Object point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obb_corners, obb_axis; ///< OBB corners and axes [m]

  ObjectEdge edge;            ///< Edge to skeleton or other objects

  // filters
  bool is_alive{true};                        ///< Whether the object is still alive [--]
  ros::Time last_detection_time;              ///< Last detection timestamp [s]
  unsigned int detection_count{0};            ///< Number of successful detections [--]

  ObjectNode(){
    pos   = Eigen::Vector3d::Zero();
    label = "None";
    id    = -1;
    conf  = 0.0f;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    obb_corners.reset(new pcl::PointCloud<pcl::PointXYZ>);
    obb_axis.reset(new pcl::PointCloud<pcl::PointXYZ>);
    label_feature = Eigen::VectorXd::Zero(512);
  }
  /**
   * Check whether the object is sufficiently detected.
   *
   * @param[in] threshold  Minimum detection count required [--]
   * @return True if detection count >= threshold
   */
  bool isWellDetected(int threshold) const {
    return detection_count >= threshold;
  }
};

/**
 * Processed segmentation input data.
 *
 * Stores one frame of depth + RGB + mask data from the segmentation
 * pipeline, along with the camera-to-world transform and label.
 */
struct ProcessedCLoudInput {
  cv::Mat         depth_img;      ///< Depth image
  cv::Mat         rgb_img;        ///< RGB image
  cv::Mat         mask;           ///< Segmentation mask
  Eigen::Vector3d pos;            ///< Camera position [m]
  Eigen::Matrix4d tf;             ///< Camera-to-world transform [m]
  std::string     label;          ///< Semantic label
  double          conf;           ///< Detection confidence [0, 1]
  Eigen::Vector3d pt_color;       ///< Point cloud color [0, 1]
  Eigen::VectorXd label_feature;  ///< Semantic feature vector (512-d)
  ProcessedCLoudInput (const cv::Mat& depth_in, const cv::Mat& rgb_in, const cv::Mat& mask_in,
                       const Eigen::Matrix4d& tf_in, const std::string& label_in, const Eigen::VectorXd & label_feature_in,
                       const double & conf_in, const Eigen::Vector3d& pt_color_in) {
    rgb_img   = rgb_in.clone();
    depth_img = depth_in.clone();
    mask      = mask_in.clone();
    tf        = tf_in;
    pos       = tf_in.block<3, 1>(0, 3);
    label     = label_in;
    conf      = conf_in;
    pt_color  = pt_color_in;
    if (label_feature_in.size() != 512) {
      INFO_MSG_RED("*** ERROR: label_feature_in.size() != 512, program dumped out ***");
      exit(1);
    }
    label_feature = label_feature_in;
  };
};

/**
 * Sampling vertex of a polyhedron.
 *
 * A vertex is a candidate point on the free-space boundary, classified
 * as black (obstacle-contact), white (free-space), gray (boundary check),
 * or rubbish (discarded). Stores position, direction, and connectivity.
 */
class Vertex{
public:
  enum VertexType{
    BLACK   = 0,  ///< Obstacle-contact sample point
    WHITE   = 1,  ///< Free-space sample point
    GRAY    = 2,  ///< Boundary check point
    RUBBISH = 3   ///< Discarded sample point
  };
  bool                   is_visited_;           ///< Whether this vertex has been processed [--]
  bool                   is_critical_;           ///< Whether this vertex is critical for structure [--]
  VertexType             type_;                  ///< Vertex type (BLACK/WHITE/GRAY/RUBBISH)
  Eigen::Vector3d        position_;               ///< Vertex position [m]
  Eigen::Vector3d        direction_in_unit_sphere_; ///< Direction from center on unit sphere [--]
  int                    dir_sample_buffer_index_; ///< Index in sphere sample buffer [--]
  std::vector<VertexPtr> connected_vertices_;    ///< Connected vertices via edges [--]
  double                 distance_to_center_;    ///< Distance from polyhedron center [m]
  /**
   * Construct a vertex with position, direction, and type.
   *
   * @param[in] position                     Vertex position [m]
   * @param[in] direction_in_unit_sphere    Direction from center on unit sphere [--]
   * @param[in] type                        Vertex type
   */
  Vertex(Eigen::Vector3d position, Eigen::Vector3d direction_in_unit_sphere, VertexType type){
    position_ = position;
    direction_in_unit_sphere_ = direction_in_unit_sphere;
    type_ = type;
    is_visited_ = false;
    is_critical_ = false;
  }
  Vertex() {}
  ~Vertex() {}
};

/**
 * Triangular facet of a polyhedron.
 *
 * A facet is a triangle formed by three vertices, defining the boundary
 * between free and occupied space. Each facet has an outwards-pointing
 * unit normal, a plane equation, and references to neighboring facets.
 */
class Facet{
public:
  int index_;                    ///< Facet index [--]
  Eigen::Vector3d out_unit_normal_;    ///< Outwards unit normal [--]
  Eigen::Vector3d center_;             ///< Facet centroid (average of 3 vertices) [m]
  Eigen::Vector4d plane_equation_;     ///< Plane equation (a, b, c, d): ax+by+cz+d=0 [--]
  std::vector<VertexPtr>  vertices_;           ///< Three vertices of the triangle [--]
  std::vector<FacetPtr>   neighbor_facets_;    ///< Adjacent facets sharing edges [--]
  PolyHedronPtr           master_polyhedron_;  ///< Parent polyhedron [--]
  bool frontier_processed_;  ///< Whether this facet has been processed for frontier generation [--]
  bool is_linked_;           ///< Whether this facet is linked to another polyhedron [--]
  bool is_visited_;          ///< Visited flag for split frontier traversal [--]
  /**
   * Construct a facet from three vertices belonging to a polyhedron.
   *
   * @param[in] vertices             Vector of 3 vertex pointers
   * @param[in] master_polyhedron    Parent polyhedron
   */
  Facet(const std::vector<VertexPtr> &vertices, const PolyHedronPtr &master_polyhedron){
    vertices_ = vertices;
    master_polyhedron_ = master_polyhedron;
    center_ = (vertices_.at(0)->position_ + vertices_.at(1)->position_ + vertices_.at(2)->position_) / 3.0;
    const Eigen::Vector3d v1 = vertices.at(1)->position_ - vertices.at(0)->position_;
    const Eigen::Vector3d v2 = vertices.at(2)->position_ - vertices.at(0)->position_;
    Eigen::Vector3d normal = v1.cross(v2);
    normal.normalize();
    const double a = normal(0); const double b = normal(1); const double c = normal(2);
    const double d = -normal.dot(vertices.at(0)->position_);
    plane_equation_ = Eigen::Vector4d(a, b, c, d);
    frontier_processed_ = false;
    is_linked_   = false;
    is_visited_  = false;
  }
  Facet() {}
  ~Facet() {}
};

/**
 * Free-space polyhedron node in the skeleton graph.
 *
 * Represents a convex polyhedron approximating free space, generated by
 * sphere-sampling in the plane tangent to a frontier. Stores black/white
 * vertices, facets, frontiers, connectivity, objects, and area association.
 * Each polyhedron corresponds to a node in the topological skeleton graph.
 */
class Polyhedron{
private:
  struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& vector) const {
      std::size_t h1 = std::hash<double>()(vector.x());
      std::size_t h2 = std::hash<double>()(vector.y());
      std::size_t h3 = std::hash<double>()(vector.z());
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };
public:
  typedef std::shared_ptr<Polyhedron> Ptr;
  Eigen::Vector3d               center_, origin_center_;      ///< Current and original centroid [m]
  bool                          is_gate_, is_rollbacked_;     ///< Whether this node is a gate/rollback [--]
  bool                          can_reach_;                    ///< Whether this node is reachable [--]
  double                        radius_;                       ///< Estimated radius [m]
  std::vector<VertexPtr>        black_vertices_, white_vertices_, gray_vertices_; ///< Sampled vertices by type
  std::vector<FacetPtr>         facets_;                       ///< Triangular facets of the polyhedron
  std::vector<PolyhedronFtrPtr> ftrs_;                         ///< Frontiers belonging to this polyhedron
  std::vector<PolyHedronPtr>    connected_nodes_;               ///< Connected polyhedra (including gates)
  std::vector<Edge>             edges_;                         ///< Edges to connected nodes
  PolyhedronFtrPtr              parent_ftr_;                    ///< Parent frontier that generated this polyhedron
  std::unordered_map<Eigen::Vector3d, bool, Vector3dHash> candidate_rollback_; ///< Candidate rollback positions

  Eigen::Vector3d box_min_, box_max_;  ///< Axis-aligned bounding box [m]

  std::map<int, ObjectNodePtr> objs_;  ///< Objects associated with this polyhedron [--]

  int area_id_{-1};                    ///< ID of the area (cluster) this polyhedron belongs to [--]

  double temp_distance_to_nxt_poly_;   ///< A* distance to next polyhedron [m]

  /**
   * Construct a polyhedron at a given centroid.
   *
   * @param[in] center     Centroid position [m]
   * @param[in] parent_ftr Parent frontier that generated this node
   * @param[in] is_gate    Whether this is a gate node
   */
  Polyhedron(const Eigen::Vector3d center, const PolyhedronFtrPtr &parent_ftr, const bool is_gate = false){
    center_         = center;
    origin_center_  = center;
    parent_ftr_     = parent_ftr;
    is_gate_        = is_gate;
    can_reach_      = false;
    radius_         = 0.0;
    black_vertices_.clear();
    white_vertices_.clear();
    connected_nodes_.clear();
    edges_.clear();
    candidate_rollback_.clear();
    facets_.clear();
    ftrs_.clear();
    temp_distance_to_nxt_poly_ = 0.0;
    box_max_ = Eigen::Vector3d(-99999.0, -99999.0, -99999.0);
    box_min_ = Eigen::Vector3d(99999.0, 99999.0, 99999.0);
  }
  Polyhedron() {}
  ~Polyhedron() {}
};

/**
 * Frontier of a polyhedron.
 *
 * A frontier is a set of adjacent facets on a polyhedron that face toward
 * unmapped/unknown space. Each frontier may generate a new polyhedron via
 * expansion. Tracks area, normal direction, projection center, and the
 * gate polyhedron connecting to the expanded node.
 */
class PolyhedronFtr{
public:
  int index;                           ///< Frontier index [--]
  Eigen::Vector3d avg_center_;         ///< Average of facet centroids [m]
  Eigen::Vector3d out_unit_normal_;    ///< Average outwards unit normal [--]
  Eigen::Vector3d proj_center_;        ///< Projected center on a facet [m]
  double cos_theta_;                   ///< Cosine between frontier normal and facet normal [--]
  double area_size_;                   ///< Total area of frontier facets [m^2]
  Eigen::Vector3d next_node_pos_;      ///< Position of the expanded node [m]
  bool valid_;                         ///< Whether this frontier is valid for expansion [--]
  bool deleted_;                       ///< Whether this frontier has been deleted [--]
  PolyHedronPtr master_polyhedron;     ///< Parent polyhedron that generated this frontier
  PolyHedronPtr gate_;                 ///< Gate polyhedron (the expanded node)
  FacetPtr proj_facet_;                ///< Projection facet for normal computation
  std::vector<FacetPtr>  facets_;      ///< Facets belonging to this frontier
  std::vector<VertexPtr> vertices_;    ///< Boundary vertices of the frontier

  /**
   * Construct a frontier from a cluster of facets.
   *
   * @param[in] facets  Facets forming this frontier
   * @param[in] master  Parent polyhedron
   */
  PolyhedronFtr(std::vector<FacetPtr> facets, PolyHedronPtr master){
    auto addSingleFacetAreaSize = [] (FacetPtr facet) -> double{
        Eigen::Vector3d v1 = facet->vertices_.at(1)->position_ - facet->vertices_.at(0)->position_;
        Eigen::Vector3d v2 = facet->vertices_.at(2)->position_ - facet->vertices_.at(0)->position_;
        return 0.5 * v1.cross(v2).norm();
    };
    facets_                    = facets;
    master_polyhedron          = master;
    int num_facet              = facets_.size();
    area_size_                 = 0.0;
    Eigen::Vector3d coord_sum  = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();

    for (int i = 0; i < num_facet; i++) {
      coord_sum  += facets_.at(i)->center_;
      normal_sum += facets_.at(i)->out_unit_normal_;
      area_size_ += addSingleFacetAreaSize(facets_.at(i));
    }
    avg_center_           = coord_sum / num_facet;
    out_unit_normal_ = normal_sum / num_facet;

    valid_   = false;
    deleted_ = false;
    gate_    = nullptr;
  }
  PolyhedronFtr(){}
  ~PolyhedronFtr() = default;
};

/**
 * Edge connecting two polyhedra in the skeleton graph.
 *
 * Stores the target polyhedron, path between centroids, and
 * traversability cost (path length).
 */
class Edge{
public:
  PolyHedronPtr poly_nxt_;            ///< Target (neighbor) polyhedron
  double length_;                     ///< Edge length (path distance) [m]
  double weight_;                     ///< Edge weight for graph algorithms [--]
  bool is_force_connected_{false};    ///< Whether this edge was force-connected [--]
  std::vector<Eigen::Vector3d> path_; ///< Waypoints along the connecting path [m]
  /**
   * Construct an edge with direct distance.
   *
   * @param[in] poly_nxt  Target polyhedron
   * @param[in] length    Edge length [m]
   */
  Edge(PolyHedronPtr poly_nxt, double length): poly_nxt_(poly_nxt), length_(length) { path_.clear(); }
  /**
   * Construct an edge with a path.
   *
   * @param[in] poly_nxt       Target polyhedron
   * @param[in] path           Waypoints along the path [m]
   * @param[in] do_path_reverse  Whether to reverse the path order
   */
  Edge(PolyHedronPtr poly_nxt, std::vector<Eigen::Vector3d> path, bool do_path_reverse): poly_nxt_(poly_nxt){
    if (do_path_reverse){
      path_ = path;
      std::reverse(path_.begin(), path_.end());
    }
    length_ = 0.0;
    for (int i = 1; i < path_.size(); i++)
      length_ += (path_.at(i) - path_.at(i-1)).norm();
  }
  void forceConnect(){is_force_connected_ = true;};
  ~Edge() {}
};

/**
 * Cluster (area / room) of polyhedra.
 *
 * An area is a set of polyhedra grouped together by spectral clustering or
 * community detection, representing a room or functional region. Stores
 * bounding box, centroid, room label/description, and associated objects.
 */
class PolyhedronCluster {
public:
  typedef std::shared_ptr<PolyhedronCluster> Ptr;
  PolyhedronCluster() {
    polys_.clear();
    objects_.clear();
    room_label_ = room_description_ = "";
    id_      = 0;
    color_   = Eigen::Vector3d(1.0, 0.0, 0.0);
    box_min_ = Eigen::Vector3d(99999.0, 99999.0, 99999.0);
    box_max_ = Eigen::Vector3d(-99999.0, -99999.0, -99999.0);
    center_  = Eigen::Vector3d::Zero();
    num_ftrs_= 0;
  }
  std::vector<PolyHedronPtr>   polys_;       ///< Polyhedra belonging to this cluster
  std::vector<ObjectNodePtr> objects_;       ///< Objects belonging to this cluster
  std::string     room_label_, room_description_; ///< Room classification label and description
  Eigen::Vector3d box_min_, box_max_;         ///< Bounding box of the area [m]
  Eigen::Vector3d center_;                    ///< Area centroid (averaged position) [m]
  Eigen::Vector3d color_;                     ///< Visualization color [0, 1]
  unsigned int    id_;                        ///< Area identifier [--]
  int             num_ftrs_;                  ///< Number of frontiers in this area [--]
  int             last_obj_num_{0};           ///< Last object count for tracking changes [--]
  std::map<int, bool> nbr_area_;              ///< Neighboring area IDs and connectivity

  void addPoly(PolyHedronPtr& poly, bool change_poly_mount){
    polys_ .push_back(poly);
    box_max_ = Eigen::Vector3d(std::max(box_max_.x(), poly->box_max_.x()), std::max(box_max_.y(), poly->box_max_.y()), std::max(box_max_.z(), poly->box_max_.z()));
    box_min_ = Eigen::Vector3d(std::min(box_min_.x(), poly->box_min_.x()), std::min(box_min_.y(), poly->box_min_.y()), std::min(box_min_.z(), poly->box_min_.z()));
    center_  = center_ + (poly->center_ - center_) / polys_.size();
    if (change_poly_mount) poly->area_id_ = id_;
  };
  void addObject(ObjectNodePtr obj) {
    if (obj->isWellDetected(5)) {
      objects_.push_back(obj);
    }
  }
  void clearObjs() {
    last_obj_num_ = objects_.size();
    objects_.clear();
  }

  void resetClusterWithPolys(std::vector<PolyHedronPtr>& polys, bool reset_semantics=true) {
    box_min_ = Eigen::Vector3d(99999.0, 99999.0, 99999.0);
    box_max_ = Eigen::Vector3d(-99999.0, -99999.0, -99999.0);
    center_  = Eigen::Vector3d::Zero();
    num_ftrs_     = 0;
    if (reset_semantics){
      room_label_   = room_description_ = "Unknown";
    }
    last_obj_num_ = objects_.size();
    nbr_area_.clear();
    objects_.clear();
    polys_.clear();
    for (auto& poly : polys) addPoly(poly, true);
  }
};

#endif //SKELETON_GENERATION_DATA_STRUCTURE_HPP