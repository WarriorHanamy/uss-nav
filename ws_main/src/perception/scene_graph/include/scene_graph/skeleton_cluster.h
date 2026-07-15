//
// Created by gwq on 8/11/25.
//

#ifndef SPECTRAL_CLUSTER_H
#define SPECTRAL_CLUSTER_H

#include "../include/scene_graph/data_structure.h"
#include "../include/scene_graph/hungarian_alg.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "igraph/igraph.h"
#include "../libs/libleidenalg/include/Optimiser.h"
#include "../libs/libleidenalg/include/ModularityVertexPartition.h"
#include "../libs/libleidenalg/include/CPMVertexPartition.h"

/**
 * Hash function for Eigen::Vector3d used in unordered_map keys.
 */
struct Vector3dHash_SpecClus {
    std::size_t operator()(const Eigen::Vector3d& vector) const {
        std::size_t h1 = std::hash<double>()(vector.x());
        std::size_t h2 = std::hash<double>()(vector.y());
        std::size_t h3 = std::hash<double>()(vector.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * Spectral clustering for polyhedron area detection.
 *
 * Clusters polyhedron nodes into areas (rooms) using spectral
 * clustering on a similarity matrix derived from centroid distances.
 * The Gaussian kernel bandwidth sigma_sq controls cluster granularity.
 */
class SpectralCluster {
public:
    typedef std::shared_ptr<SpectralCluster> Ptr;
    SpectralCluster(ros::NodeHandle& nh, double sigma_sq): nh_(nh), sigma_sq_(sigma_sq) {
        cluster_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/skeleton/cluster_vis", 2, true);
    };
    ~SpectralCluster() = default;
    /**
     * Compute spectral clustering on the given polyhedra.
     *
     * @param[in]  polys_without_gate  Input polyhedra (excluding gates)
     * @param[out] clusters            Output area clusters
     */
    void calculate(const std::vector<PolyHedronPtr>&polys_without_gate, std::vector<PolyhedronCluster>& clusters);

private:
    ros::NodeHandle& nh_;
    ros::Publisher cluster_vis_pub_;

    double sigma_sq_{1.0};    ///< Gaussian kernel bandwidth for similarity [m^2]
    unsigned int k_{0};       ///< Number of clusters [--]
    /**
     * Compute similarity matrix from polyhedron centroid distances.
     *
     * @param[out] W      Similarity matrix
     * @param[out] ED     Euclidean distance matrix
     * @param[in]  polys  Input polyhedra
     */
    void calSimilarityMatrix(Eigen::MatrixXd& W, Eigen::MatrixXd& ED, std::vector<PolyHedronPtr> polys);
    /**
     * Compute degree matrix from similarity matrix.
     *
     * @param[in]  W  Similarity matrix
     * @param[out] D  Degree matrix
     */
    void calDegreeMatrix(Eigen::MatrixXd& W, Eigen::MatrixXd& D);
    /**
     * Compute Laplacian matrix L = D - W.
     *
     * @param[in]  W  Similarity matrix
     * @param[in]  D  Degree matrix
     * @param[out] L  Laplacian matrix
     */
    void calLaplacianMatrix(Eigen::MatrixXd& W, Eigen::MatrixXd& D, Eigen::MatrixXd& L);
    /**
     * Compute eigenvalues and eigenvectors of the Laplacian.
     *
     * @param[in]  L  Laplacian matrix
     * @param[out] U  Eigenvector matrix
     */
    void calLaplacianEigen(Eigen::MatrixXd& L, Eigen::MatrixXd& U);
    /**
     * K-means clustering on eigenvector rows.
     *
     * @param[in] points   Input points (eigenvector rows)
     * @param[in] k        Number of clusters [--]
     * @param[in] max_iter Maximum iterations [--]
     * @return Cluster labels per point
     */
    std::vector<int> kmeans(const Eigen::MatrixXd& points, int k, int max_iter);

    /**
     * Publish cluster visualization markers.
     *
     * @param[in] clusters  Cluster data to visualize
     */
    void visualizeClusters(const std::vector<PolyhedronCluster>& clusters);
};

/**
 * Area (room) handler for the skeleton graph.
 *
 * Manages incremental area updates as new polyhedra are added
 * to the skeleton. Uses community detection (Louvain/Leiden on
 * the polyhedron adjacency graph) for area refinement.
 * Maintains area-neighbor relationships and un/predicted area lists.
 */
class AreaHandler {
public:
    typedef std::shared_ptr<AreaHandler> Ptr;
    AreaHandler(ros::NodeHandle& nh): nh_(nh) {
        cluster_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/skeleton/cluster_vis", 2, true);
        edge_weight_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/skeleton/edge_weight_vis", 2, true);
    };
    ~AreaHandler() = default;
    /**
     * Get the current area clusters.
     *
     * @param[out] clusters  Output area cluster list
     */
    void getCurAreas(std::vector<PolyhedronCluster::Ptr>& clusters);
    /**
     * Get the area ID for a given polyhedron.
     *
     * @param[in] poly  Polyhedron pointer
     * @return Area ID [--]
     */
    int getAreaFromPoly(const PolyHedronPtr &poly);
    /**
     * Incrementally update areas when new polyhedra are added.
     *
     * @param[in] new_polys  Vector of newly added polyhedra
     */
    void incrementalUpdateAreas(const vector<PolyHedronPtr>& new_polys);
    
    void resetForMapLoad();
    /**
     * Register a loaded area from disk.
     *
     * @param[in] area  Loaded area cluster
     * @return True if registration succeeded
     */
    bool registerLoadedArea(const PolyhedronCluster::Ptr& area);
    void finishMapLoad();
    std::map<int, PolyhedronCluster::Ptr> area_map_;
    std::map<int, bool> areas_need_predict_, areas_need_delete_;

    /**
     * Publish all current cluster markers.
     */
    void visualizeClusters();

private:
    ros::NodeHandle& nh_;
    ros::Publisher cluster_vis_pub_, edge_weight_vis_pub_;
    std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus> poly_cluster_map_;
    std::mutex mutex_;

    int max_area_id_{0};                      ///< Next available area ID (increment after use) [--]

    void mutexLock() {mutex_.lock();};
    void mutexUnlock() {mutex_.unlock();};
    /**
     * Community detection using Leiden algorithm with CPM partition.
     *
     * @param[in]     polys_all      All polyhedra in the graph
     * @param[out]    partition_res  Resulting partition
     * @param[in]     resolution     Resolution parameter [--]
     */
    void communityDetection(vector<PolyHedronPtr> &polys_all, std::unique_ptr<CPMVertexPartition>& partition_res, double resolution);
    /**
     * Find neighbor areas for a given area.
     *
     * @param[in] cur_area_id  Area ID to query
     */
    void findCurAreaNbrs(int cur_area_id);
    /**
     * Visualize edge weights between adjacent polyhedra.
     *
     * @param[in] polys       Polyhedra list
     * @param[in] edges_data  Edge connectivity data
     * @param[in] edge_weights  Edge weight values [--]
     */
    void visualizeEdgeWeights(const std::vector<PolyHedronPtr>& polys, const std::vector<igraph_integer_t>& edges_data, const std::vector<double>& edge_weights);
    /**
     * Draw a 3D bounding box marker.
     *
     * @param[out] marker    Output marker
     * @param[in]  min       Minimum corner [m]
     * @param[in]  max       Maximum corner [m]
     * @param[in]  id        Marker ID [--]
     * @param[in]  color     RGB color [--]
     * @param[in]  line_width  Line width [m]
     */
    void drawBoundingBox(visualization_msgs::Marker& marker, const Eigen::Vector3d& min, const Eigen::Vector3d& max,
                         int id, const Eigen::Vector3d &color, float line_width);
    /**
     * Convert Eigen::Vector3d to geometry_msgs::Point.
     *
     * @param[in] pt  Input point [m]
     * @return ROS geometry point
     */
    inline geometry_msgs::Point eigenToGeoPt(const Eigen::Vector3d& pt);
};

#endif //SPECTRAL_CLUSTER_H
