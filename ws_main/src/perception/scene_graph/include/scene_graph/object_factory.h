//
// Created by gwq on 7/11/25.
//

#ifndef OBJECT_FACTORY_H
#define OBJECT_FACTORY_H

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

#include "../scene_graph/data_structure.h"
#include "../scene_graph/ikd_Tree.h"
#include "../scene_graph/skeleton_generation.h"

#include <algorithm>
#include <deque>
#include <thread>
#include <chrono>
#include <future>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <random>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/common.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <scene_graph/pt_cloud_tools.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <scene_graph/EncodeMask.h>


typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::Image, scene_graph::EncodeMask> DepthRawMaskSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::CompressedImage, scene_graph::EncodeMask> DepthCompressedMaskSyncPolicy;

using ObjectKDTree = skeleton_gen::KD_TREE<skeleton_gen::ikdTree_ObjectDataType>;
using ObjectKDTreeNodeVector = ObjectKDTree::PointVector;

/**
 * Object detection, tracking, and fusion factory.
 *
 * Processes segmented point clouds from RGB-D data, extracts oriented
 * bounding boxes, computes CLIP semantic features, and maintains a
 * persistent object map with spatial/semantic similarity merging.
 * Supports multi-view detection, temporal filtering, and KD-tree
 * based nearest-neighbor queries for object association.
 */
class ObjectFactory {
public:
    typedef std::shared_ptr<ObjectFactory> Ptr;
    typedef std::unique_ptr<ObjectFactory> UPtr;

    /**
     * Input data structure for a single frame of segmentation output.
     */
    struct SemanticDataInput {
        cv::Mat cur_depth_, cur_rgb_;                               /**< Depth and RGB images */
        nav_msgs::Odometry cur_depth_odom_;                  /**< Depth sensor odometry */
        Eigen::Matrix4d    cur_tf_;                        /**< Camera-to-world transform [m] */
        Eigen::Vector3d    cur_pos_;                      /**< Camera position [m] */
        scene_graph::EncodeMask::ConstPtr cur_semantic_recv_msg_;     /**< Segmentation result message */
    };

    ObjectFactory(ros::NodeHandle& nh);
    ObjectFactory(ros::NodeHandle& nh, SkeletonGeneratorPtr skel_gen_ptr);
    ObjectFactory(ros::NodeHandle& nh, const std::string& param_prefix,
                  const std::string& topic_prefix);
    ~ObjectFactory();

    /**
     * Start the object processing loop.
     *
     * Enables input acceptance and notifies waiting threads.
     * Must be called after construction to begin processing.
     */
    void runThisModule();

    /**
     * Stop the object processing loop.
     *
     * Disables input acceptance and pauses processing threads.
     */
    void stopThisModule();

    /**
     * Start a fresh detection session.
     *
     * Cancels current session, resets map state,
     * re-enables input, and notifies threads.
     */
    void startFreshSession();

    /**
     * Cancel the current detection session.
     *
     * Clears pending messages and waits for active processing to finish.
     */
    void cancelSession();

    /**
     * Stop processing and return objects with sufficient detections.
     *
     * @return List of object nodes that passed the detection counter threshold
     */
    std::vector<ObjectNode::Ptr> stopAndSnapshot();

    /**
     * Lock the internal mutex.
     */
    void lock(){mutex_.lock();};

    /**
     * Unlock the internal mutex.
     */
    void unlock(){mutex_.unlock();};

    /**
     * Check whether the main processing loop is allowed to run.
     *
     * @return True if allow_thread_run_ is set
     */
    bool ok();

    /**
     * Get object-skeleton edges for scene graph visualization.
     *
     * @param[in]  poly_clusterId_map  Mapping of polyhedron centroids to area IDs
     * @param[out] edges               Output edge end points [m]
     */
    void getObjectEdgesWithArea(const std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus>& poly_clusterId_map,
                                std::vector<std::vector<Eigen::Vector3d>>& edges);

    /**
     * Get pointer to the full object map.
     *
     * @return Pointer to the map of object ID to ObjectNode
     */
    std::map<int, ObjectNode::Ptr>* getAllObjs(){return &object_map_;};

    /**
     * Check if an object has enough detections for persistence.
     *
     * @param[in]  obj_node  Object node to check [--]
     * @return True if detection count meets threshold
     */
    bool objInGoodDetection(const ObjectNode::Ptr& obj_node) const {return obj_node->detection_count >= _detection_counter_thresh;};

    /**
     * Reset runtime state for a fresh map load.
     *
     * Clears all object maps, queues, and KD-tree.
     */
    void resetForMapLoad();

    /**
     * Register a pre-existing object into the map.
     *
     * @param[in]  obj_node           Object to register [--]
     * @param[in]  need_more_detection Whether to keep in pending queue [--]
     * @return True if registration succeeded
     */
    bool registerLoadedObject(const ObjectNode::Ptr& obj_node, bool need_more_detection);

    /**
     * Finalize map loading state.
     *
     * Clears temporary update buffers after a map load.
     */
    void finishMapLoad();

    /**
     * Publish visualization markers for debugging.
     *
     * @param[in]  force_full_refresh  Force redraw all markers [--]
     */
    void visualizeResult(bool force_full_refresh = false);

    std::map<int, ObjectNode::Ptr> object_map_, object_map_needMoreDetection_;

private:

    /**
     * Initialize parameters, subscribers, and processing threads.
     */
    void init();
    std::mutex mutex_;
    std::shared_ptr<SkeletonGenerator> skel_gen_ptr_;

    double _camera_fx, _camera_fy, _camera_cx, _camera_cy;  ///< Camera intrinsics [pixel]
    double _lidar_cam_tx, _lidar_cam_ty, _lidar_cam_tz;       ///< LiDAR-camera extrinsic translation [m]
    double _lidar_cam_pitch, _lidar_cam_roll, _lidar_cam_yaw; ///< LiDAR-camera extrinsic rotation [rad]
    double _voxel_size;           ///< Voxel grid size for downsampling [m]
    double _max_depth;            ///< Maximum depth range [m]
    double _min_depth;            ///< Minimum depth range [m]
    double _max_ray_length;       ///< Maximum ray length for depth filtering [m]
    double _std_dev_thresh;       ///< Standard deviation threshold for outlier removal [--]
    int    _mean_k;               ///< K for mean filter / statistical outlier removal [--]
    int    _max_threads;          ///< Maximum threads for parallel processing [--]
    double _fov_vertical;         ///< Vertical field of view [rad]
    double _fov_horizontal;       ///< Horizontal field of view [rad]
    int    _cam_resolution_h, _cam_resolution_w; ///< Camera resolution [pixel]

    int    _obj_cloud_num_thresh;       ///< Min point cloud size for valid object [--]
    int    _detection_counter_thresh;   ///< Detection count threshold for persistence [--]

    int    _max_deque_size;

    bool   _depth_cloud_disp_all;
    bool   _use_camera_intrinsics;
    bool   _use_realsense;

    bool skeleton_enabled_;
    bool accepting_input_{false};
    int active_processing_count_{0};
    std::string param_prefix_;
    std::string topic_prefix_;

    std::vector<Eigen::Vector3d> depth_directions_;

    ros::NodeHandle nh_;
    ros::Subscriber segment_result_sub_;
    ros::Publisher  obj_pt_cloud_all_pub_, odom_depth_pub_;
    ros::Publisher  obj_detection_vis_pub_, obj_all_vis_pub_, obj_update_vis_pub_, obj_update_pt_cloud_pub_;
    tf::TransformBroadcaster depth_world_frame_tf_broadcaster_;

    // Object Data
    PolyHedronPtr cur_polyhedron_, last_polyhedron_;
    deque<SemanticDataInput> semantic_msg_queue_;
    SemanticDataInput cur_data_;
    int  object_max_id_{-1};
    bool object_kdtree_initialized_{false};
    std::vector<int> cur_update_ids_;
    std::vector<ObjectNode::Ptr> cur_update_objs_, cur_add_objs_, cur_update_all_;
    std::vector<std::pair<Eigen::Vector3d, ObjectNode::Ptr>> update_existing_objects_;

    std::vector<ObjectNode::Ptr> cur_observe_results_;
    skeleton_gen::KD_TREE<skeleton_gen::ikdTree_ObjectDataType>::Ptr object_kd_tree_;

    // Object Thread
    std::condition_variable condition_var_;
    std::condition_variable drain_condition_var_;
    bool allow_thread_run_{false};
    double _filter_run_duration;
    int    _obj_main_thread_run_hz;
    bool obj_filter_thread_running_{false}, obj_process_thread_running_{false};
    std::unique_ptr<std::thread> object_filter_thread_, object_process_thread_;
    /**
     * Background thread for filtering poorly detected objects.
     */
    void objectFilterThread();
    /**
     * Background thread for processing semantic data frames.
     */
    void objectProcessThread();


    /**
     * Callback for incoming segmentation results.
     *
     * @param[in]  msg  Encoded segmentation mask message
     */
    void segmentationResultCallback(const scene_graph::EncodeMask::ConstPtr& msg);

    /**
     * Extract a colored point cloud from depth, RGB, and mask images.
     *
     * @param[in]  depth_img  Depth image [m]
     * @param[in]  rgb_img    RGB image [--]
     * @param[in]  mask       Segmentation mask [--]
     * @param[in]  color      Assigned point color [RGB]
     * @return Extracted point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractCloud(const cv::Mat& depth_img, const cv::Mat &rgb_img, const cv::Mat& mask, const Eigen::Vector3d &color);

    /**
     * Apply voxel grid and statistical outlier filtering.
     *
     * @param[in]  cloud_in  Input point cloud [m]
     * @return Filtered point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteringCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in);


    /**
     * Push input data into the processing queue.
     *
     * @param[in]  data  Semantic data input [--]
     */
    void pushDataInDeque(const SemanticDataInput& data);

    /**
     * Process a single masked object into an ObjectNode.
     *
     * @param[in]  input  Processed cloud input [--]
     * @return Object node with cloud, OBB, and semantic features
     */
    ObjectNode::Ptr processSingleObject(const ProcessedCLoudInput &input);

    /**
     * Execute one iteration of semantic processing.
     *
     * Dequeues a message, spawns threads for each mask,
     * merges results into the persistent object map.
     */
    void doSemanticProcessingOnce();


    /**
     * Precompute ray directions from camera FOV.
     *
     * @param[in]  vertical_fov  Vertical field of view [rad]
     */
    void calculateDepthDirectionsFromVerticalFov(double vertical_fov);

    /**
     * Compute oriented bounding box from a point cloud.
     *
     * @param[in]   cloud        Input point cloud [m]
     * @param[out]  obb_corners  8 corner points of OBB [m]
     */
    void getOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &obb_corners);

    /**
     * Compute axis-aligned bounding box from a point cloud.
     *
     * @param[in]   cloud         Input point cloud [m]
     * @param[out]  aab_corners   8 corner points of AABB [m]
     */
    void getAxisAlignedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &aab_corners);

    /**
     * Calculate intersection volume between two bounding boxes.
     *
     * @param[in]  box1  First bounding box corners [m]
     * @param[in]  box2  Second bounding box corners [m]
     * @return Intersection volume [m^3]
     */
    double calculateBoxIntersection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& box1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& box2);

    /**
     * Calculate spatial similarity between two objects.
     *
     * @param[in]  obj1  First object [--]
     * @param[in]  obj2  Second object [--]
     * @return Similarity score [--]
     */
    double calculateSpatialSimilarity(const ObjectNode::Ptr& obj1, const ObjectNode::Ptr& obj2);

    /**
     * Calculate semantic feature similarity between two objects.
     *
     * @param[in]  obj1  First object [--]
     * @param[in]  obj2  Second object [--]
     * @return Cosine similarity score [--]
     */
    double calculateSemanticSimilarity(const ObjectNode::Ptr &obj1, const ObjectNode::Ptr &obj2);

    /**
     * Merge source object into target object.
     *
     * Combines point clouds, updates semantic features,
     * and recomputes bounding boxes.
     * @param[in,out]  obj_src     Source object to merge from
     * @param[in,out]  obj_target  Target object to merge into
     */
    void mergeObjAIntoB(ObjectNode::Ptr& obj_src, ObjectNode::Ptr& obj_target);

    /**
     * Merge an observed object into the persistent object map.
     *
     * @param[in,out]  cur_obj  Observed object to merge
     */
    void mergeObjectIntoMap(ObjectNode::Ptr &cur_obj);

    /**
     * Get objects within a radius of the center position.
     *
     * @param[in]  center             Query center [m]
     * @param[in]  radius             Search radius [m]
     * @param[out] objects_in_range   Result objects
     * @return True if results found
     */
    bool getObjectsInRange(const Eigen::Vector3d &center, double radius, ObjectKDTreeNodeVector &objects_in_range);
    /**
     * Get nearest N objects to the center position.
     *
     * @param[in]  center            Query center [m]
     * @param[in]  n                 Number of nearest objects [--]
     * @param[out] objects_nearest_n Result objects
     * @return True if results found
     */
    bool getObjectsNearestN(const Eigen::Vector3d &center, int n, ObjectKDTreeNodeVector &objects_nearest_n);

    /**
     * Add a new object to the map and KD-tree.
     *
     * @param[in,out]  obj_node  Object node to add
     */
    void addNewObject(ObjectNode::Ptr& obj_node);

    /**
     * Delete a single object from the KD-tree.
     *
     * @param[in]  obj_node  Object node to delete [--]
     * @return True if deletion succeeded
     */
    bool deleteObjectInTree(const ObjectNode::Ptr &obj_node);

    /**
     * Delete multiple objects from the KD-tree.
     *
     * @param[in]  obj_nodes  List of object nodes to delete [--]
     * @return True if any deletion succeeded
     */
    bool deleteObjectInTree(const std::vector<ObjectNode::Ptr> &obj_nodes);

    /**
     * Update an existing object position in the KD-tree.
     *
     * @param[in]  obj_node  Object node with updated position [m]
     */
    void updateObjectInTree(const ObjectNode::Ptr& obj_node);

    /**
     * Batch update object positions in the KD-tree.
     *
     * @param[in]  update_existing_objects  List of (old_pos, obj) pairs [m]
     */
    void updateExistingObjectInKdtree(const std::vector<std::pair<Eigen::Vector3d, ObjectNode::Ptr>> & update_existing_objects);

    // visualization utils

    /**
     * Create a DELETEALL refresh marker.
     *
     * @param[in]  ns         Namespace for the marker [--]
     * @param[in]  type       Marker type [--]
     * @param[in]  timestamp  ROS timestamp [s]
     * @return DELETEALL marker
     */
    visualization_msgs::Marker visualizeRefresh(const std::string ns, const int type, const ros::Time &timestamp);

    /**
     * Populate a marker with object bounding box wireframe.
     *
     * @param[in,out]  marker       Marker to populate
     * @param[in]      obj_node     Object node [--]
     * @param[in]      id           Marker ID [--]
     * @param[in]      timestamp    ROS timestamp [s]
     * @param[in]      use_axis_box Use AABB instead of OBB [--]
     */
    void visualizeObjBoundingBox(visualization_msgs::Marker & marker, const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp, bool
                                 use_axis_box);

    /**
     * Populate a marker with object position sphere.
     *
     * @param[in,out]  marker    Marker to populate
     * @param[in]      obj_node  Object node [--]
     * @param[in]      id        Marker ID [--]
     * @param[in]      timestamp ROS timestamp [s]
     */
    void visualizeObjPosition(visualization_msgs::Marker & marker, const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp);

    /**
     * Populate a marker with object text label.
     *
     * @param[in,out]  marker    Marker to populate
     * @param[in]      obj_node  Object node [--]
     * @param[in]      id        Marker ID [--]
     * @param[in]      timestamp ROS timestamp [s]
     */
    void visualizeObjLabel(visualization_msgs::Marker & marker, const ObjectNode::Ptr& obj_node, int id, const ros::Time &timestamp);

    /**
     * Populate a marker with all object-skeleton edges.
     *
     * @param[in,out]  marker  Marker to populate
     */
    void visualizeObjEdgeAll(visualization_msgs::Marker & marker);

    /**
     * Publish visualization markers for recently updated objects.
     */
    void visualizeUpdateObjects();

    /**
     * Remove visualization markers for deleted objects.
     *
     * @param[in]  objs_to_delete  List of objects to remove from display [--]
     */
    void deVisualizeObjects(const std::vector<ObjectNode::Ptr> &objs_to_delete);


    /**
     * Generate a random RGB color.
     *
     * @return Random color vector [0-255, RGB]
     */
    Eigen::Vector3d getRandomColor();

    /**
     * Convert Eigen vector to geometry_msgs::Point.
     *
     * @param[in]  pt  Eigen 3D point [m]
     * @return Geometry point
     */
    inline geometry_msgs::Point eigenToGeoPt(const Eigen::Vector3d& pt);

    /**
     * Convert PCL point to geometry_msgs::Point.
     *
     * @param[in]  pt  PCL 3D point [m]
     * @return Geometry point
     */
    inline geometry_msgs::Point pclToGeoPt(const pcl::PointXYZ& pt);

    /**
     * Read a ROS parameter with fallback default.
     *
     * @param[in]      node        ROS node handle
     * @param[in]      param_name  Parameter name [--]
     * @param[in,out]  param_val   Output parameter value
     * @param[in]      default_val Default value if not found
     */
    template<typename T>
    void readParam(ros::NodeHandle &node, std::string param_name, T &param_val, T default_val);

    /**
     * Get the full parameter name with prefix.
     *
     * @param[in]  name  Base parameter name [--]
     * @return Prefixed parameter name
     */
    std::string prefixedParam(const std::string& name) const;

    /**
     * Get the full topic name with prefix.
     *
     * @param[in]  name  Base topic name [--]
     * @return Prefixed topic name
     */
    std::string prefixedTopic(const std::string& name) const;

    /**
     * Decode a Realsense compressed depth image.
     *
     * @param[in]  msg  Compressed depth image message [--]
     * @return Decoded depth image as CV_16U
     */
    cv::Mat decodeRealsenseCompressedDepth(const sensor_msgs::CompressedImage& msg);
};
#endif //OBJECT_FACTORY_H
