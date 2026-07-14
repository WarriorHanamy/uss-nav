//
// Created by gwq on 8/16/24.
//

#ifndef SRC_PERCEPTION_DATA_FACTORY_H
#define SRC_PERCEPTION_DATA_FACTORY_H
#include <Eigen/Eigen>
#include <std_msgs/UInt16MultiArray.h>
#include <quadrotor_msgs/MultiPoseGraph.h>
#include <quadrotor_msgs/HgridMsg.h>
#include <quadrotor_msgs/FrontierMsg.h>
#include <quadrotor_msgs/PerceptionMsg.h>
#include <exploration/frontier_finder.h>
#include <mission_executive/frontier_manager.h>

namespace ego_planner{
class PerceptionDataMsgFactory {
public:
    PoseGraph posegraph;
    quadrotor_msgs::PerceptionMsg perception_msg;
    map<int, vector<int>> keypose_to_frontieridx;
    Eigen::Vector3d hgrid_min_, hgrid_max_, hgrid_resoloution_;
    Eigen::Vector3i grid_num_;

    /**
     * Construct the factory from a perception message and grid resolution.
     *
     * @param[in] msg_in                     Perception message
     * @param[in] inflate_gridmap_resolution Grid map resolution [m/voxel]
     */
    PerceptionDataMsgFactory(const quadrotor_msgs::PerceptionMsg msg_in,
                             const double & inflate_gridmap_resolution);
    ~PerceptionDataMsgFactory();
    /**
     * Convert a geometry_msgs::Point position to a flat HGrid address.
     *
     * @param[in] pos  Position [m]
     * @return Flat HGrid index
     */
    int posToHgridAdress(const geometry_msgs::Point& pos);
    /**
     * Convert an Eigen::Vector3d position to a flat HGrid address.
     *
     * @param[in] pos  Position [m]
     * @return Flat HGrid index
     */
    int posToHgridAdress(const Eigen::Vector3d & pos);
    /**
     * Convert a flat HGrid address back to a 3D position.
     *
     * @param[in] adress  Flat HGrid index
     * @return Position [m]
     */
    Eigen::Vector3d adressToPos(const int& adress);
    /**
     * Convert geometry_msgs::Point to Eigen::Vector3d.
     *
     * @param[in]  p_in  Input point [m]
     * @param[out] p_out Output vector [m]
     */
    static void geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
    /**
     * Convert Eigen::Vector3d to geometry_msgs::Point.
     *
     * @param[in]  p_in  Input vector [m]
     * @param[out] p_out Output point [m]
     */
    static void vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
    /**
     * Convert Eigen::Vector3d to geometry_msgs::Point (return-by-value).
     *
     * @param[in] p_in  Input vector [m]
     * @return Output point [m]
     */
    static geometry_msgs::Point vec3d2GeoPt(const Eigen::Vector3d &p_in);
    /**
     * Convert geometry_msgs::Point to Eigen::Vector3d (return-by-value).
     *
     * @param[in] p_in  Input point [m]
     * @return Output vector [m]
     */
    static Eigen::Vector3d geoPt2Vec3d(const geometry_msgs::Point &p_in);

private:
    double inflate_gridmap_resolution_;
    int    VOID_HGRID_DATA_;
};

class PerceptionMergeFactory {
private:
    quadrotor_msgs::PerceptionMsg map_res_;
    unordered_map<int, int> before_ftrID_to_after_ftrID_;
    std::unique_ptr<PerceptionDataMsgFactory> map_fac1_, map_fac2_;

    double inflate_gridmap_resolution_;
    double grid_size_res_;
    Eigen::Vector3d size_res_, resolution_res_, hgrid_min_res_, hgrid_max_res_;
    Eigen::Vector3i grid_num_res_;
    int grid_data_res_size_;

public:
    PerceptionMergeFactory();
    ~PerceptionMergeFactory();
    /**
     * Initialize the merge with two perception messages.
     *
     * @param[in] map1                      First perception message
     * @param[in] map2                      Second perception message
     * @param[in] inflate_gridmap_resolution Grid resolution [m/voxel]
     */
    void mergeInit(quadrotor_msgs::PerceptionMsg &map1, quadrotor_msgs::PerceptionMsg &map2,
                   const double inflate_gridmap_resolution);
    /**
     * Execute the full merge pipeline.
     *
     * @return True if merge succeeded
     */
    bool merge();
    /**
     * Merge two HGrid structures into one.
     */
    void mergeTwoHgrids();
    /**
     * Merge two frontier messages, keeping the first IDs and appending unique items from the second.
     *
     * @param[out] ftr_res   Result merged frontier message
     * @param[in]  ftr_msg2  Second frontier message to merge
     */
    static void mergeTwoFrontiersExceptId(quadrotor_msgs::FrontierMsg &ftr_res,
                                   const quadrotor_msgs::FrontierMsg &ftr_msg2);
    /**
     * Initialize an HGrid message with a given buffer size.
     *
     * @param[out] hgrid_res   HGrid message to initialize
     * @param[in]  buffer_size Buffer size [voxels]
     */
    static void initMemoryOfHgird(quadrotor_msgs::HgridMsg &hgrid_res, const int& buffer_size);
    /**
     * Get the merged result as a perception message.
     *
     * @param[out] map_res  Result perception message
     */
    void getMergeResult(quadrotor_msgs::PerceptionMsg &map_res);
    /**
     * Get a new grid position from a flat HGrid address with a center offset.
     *
     * @param[in] adress  Flat HGrid index
     * @param[in] inc     Center offset [m]
     * @return Grid position [m]
     */
    Eigen::Vector3d getNewGridPosFromAdress(const int& adress, const double &inc);
    /**
     * Convert geometry_msgs::Point to Eigen::Vector3d.
     *
     * @param[in]  p_in  Input point [m]
     * @param[out] p_out Output vector [m]
     */
    static void geoPt2Vec3d(const geometry_msgs::Point &p_in, Eigen::Vector3d &p_out);
    /**
     * Convert Eigen::Vector3d to geometry_msgs::Point.
     *
     * @param[in]  p_in  Input vector [m]
     * @param[out] p_out Output point [m]
     */
    static void vec3d2GeoPt(const Eigen::Vector3d &p_in, geometry_msgs::Point &p_out);
    /**
     * Convert Eigen::Vector3d to geometry_msgs::Point (return-by-value).
     *
     * @param[in] p_in  Input vector [m]
     * @return Output point [m]
     */
    static geometry_msgs::Point vec3d2GeoPt(const Eigen::Vector3d &p_in);
    /**
     * Convert geometry_msgs::Point to Eigen::Vector3d (return-by-value).
     *
     * @param[in] p_in  Input point [m]
     * @return Output vector [m]
     */
    static Eigen::Vector3d geoPt2Vec3d(const geometry_msgs::Point &p_in);
};

}

#endif //SRC_PERCEPTION_DATA_FACTORY_H
