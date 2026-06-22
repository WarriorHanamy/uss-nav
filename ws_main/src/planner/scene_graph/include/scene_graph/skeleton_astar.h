//
// Created by gwq on 25-3-18.
//

#ifndef SKELETON_ASTAR_H
#define SKELETON_ASTAR_H

#include "Eigen/Eigen"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "unordered_map"
#include "queue"
#include "../include/scene_graph/data_structure.h"

namespace skeleton_astar{

  /**
   * A* search node on the skeleton graph.
   *
   * Each node corresponds to a polyhedron in the skeleton, with
   * accumulated cost (g), heuristic (h), and parent reference.
   */
  class AstarNode{
  public:
    typedef std::shared_ptr<AstarNode> Ptr;
    PolyHedronPtr              polyhedron_;  ///< Associated polyhedron node
    Eigen::Vector3d            pos_;         ///< Node position (polyhedron centroid) [m]
    double                     cost_g_;      ///< Accumulated cost from start [m]
    double                     cost_h_;      ///< Heuristic cost to goal [m]
    double                     cost_f_;      ///< Total cost f = g + h [m]
    std::shared_ptr<AstarNode> parent_;      ///< Parent node for path reconstruction
    /**
     * Construct an A* node.
     *
     * @param[in] poly    Associated polyhedron
     * @param[in] cost_g  Accumulated cost [m]
     * @param[in] cost_h  Heuristic cost [m]
     * @param[in] parent  Parent node
     */
    AstarNode(PolyHedronPtr poly, double cost_g, double cost_h, std::shared_ptr<AstarNode> parent):
      polyhedron_(poly), cost_g_(cost_g), cost_h_(cost_h), parent_(parent){
      cost_f_ = cost_g_ + cost_h_;
      pos_ = polyhedron_->center_;
    }
    ~AstarNode(){}

    void calF(){
      cost_f_ = cost_g_ + cost_h_;
    }
  };

  /**
   * A* path search on the skeleton graph.
   *
   * Searches for the shortest path between two polyhedra using the
   * skeleton graph edges and Euclidean distance heuristic.
   * Used by SkeletonGenerator for topology-aware navigation planning.
   */
  class SkeletonAstar{
  public:
    typedef std::shared_ptr<SkeletonAstar> Ptr;
    struct AstarNodeCompare{
      bool operator()(const AstarNode::Ptr& node1, const AstarNode::Ptr& node2) const{
        return node1->cost_f_ > node2->cost_f_;
      }
    };
    struct Vector3dHash{
      std::size_t operator()(const Eigen::Vector3d& vector) const {
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };

    SkeletonAstar(ros::NodeHandle& nh){
      INFO_MSG_GREEN("[SkeletonAstar] Init complete !");
      nh_ = nh;
      tie_breaker_ = 1.0 + 1.0 / 1000;
      open_list_map_.clear();
      closed_list.clear();
      vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("skeleton_vis", 1);
    }
    ~SkeletonAstar(){}
    /**
     * Euclidean distance heuristic between two positions.
     *
     * @param[in] pos_a  First position [m]
     * @param[in] pos_b  Second position [m]
     * @return Euclidean distance [m]
     */
    inline double getEuclHeu(Eigen::Vector3d pos_a, Eigen::Vector3d pos_b);
    /**
     * Run A* search on the skeleton graph between two polyhedra.
     *
     * @param[in] poly_start  Start polyhedron
     * @param[in] poly_end    Goal polyhedron
     * @return True if a path was found
     */
    bool astarSearch(PolyHedronPtr poly_start, PolyHedronPtr poly_end);
    /**
     * Get the found path as a vector of positions.
     *
     * @param[out] path  Path waypoints [m]
     */
    void getPath(std::vector<Eigen::Vector3d>& path);
    void getNeighborPolyhedronsNotInCloseList(AstarNode::Ptr cur_node, std::vector<AstarNode::Ptr>& neighbor_nodes);
    void visualizePath();

  private:
    double tie_breaker_;
    ros::NodeHandle nh_;
    ros::Publisher  vis_pub_;
    Eigen::Vector3d end_pos_;
    std::vector<Eigen::Vector3d> path_;
    std::priority_queue<AstarNode::Ptr, std::vector<AstarNode::Ptr>, AstarNodeCompare> open_list_;
    std::unordered_map<Eigen::Vector3d, AstarNode::Ptr, Vector3dHash> open_list_map_;
    std::unordered_map<Eigen::Vector3d, AstarNode::Ptr, Vector3dHash> closed_list;
  };
}

#endif //SKELETON_ASTAR_H
