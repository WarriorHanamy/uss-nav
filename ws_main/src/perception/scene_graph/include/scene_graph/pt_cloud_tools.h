//
// Created by gwq on 7/17/25.
//

#ifndef BOX_INTERSECTION_SERVER_H
#define BOX_INTERSECTION_SERVER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <memory>
#include <omp.h>

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

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <ros/ros.h>
#include <omp.h>

/**
 * Point cloud overlap calculator.
 *
 * Computes the fraction of points in cloud B that have a neighbor
 * within a distance threshold in cloud A, using PCL KD-tree.
 */
class PointCloudOverlapCalculator {
public:
    /**
     * Calculate what fraction of cloud B points overlap with cloud A.
     *
     * @param[in] cloud_a             Reference point cloud
     * @param[in] cloud_b             Query point cloud
     * @param[in] distance_threshold  Neighbor distance threshold [m]
     * @return Overlap ratio [0, 1] [--]
     */
    double calculateOverlapBInA(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_a,
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_b,
        double distance_threshold = 0.01) {

        // 1. Basic checks
        if (!cloud_a || !cloud_b || cloud_a->empty() || cloud_b->empty()) {
            return 0.0;
        }

        // 2. Smart NaN handling (avoid unnecessary copies)
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_a_ptr = cloud_a;
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_b_ptr = cloud_b;

        // Only filter when cloud contains NaN
        if (!cloud_a->is_dense) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_a, *temp, indices);
            cloud_a_ptr = temp;
        }

        // Note: For cloud B, NaN removal is not strictly needed if using indices, but kept for simplicity
        if (!cloud_b->is_dense) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud_b, *temp, indices);
            cloud_b_ptr = temp;
        }

        if (cloud_a_ptr->empty() || cloud_b_ptr->empty()) return 0.0;

        // 3. Build KD-Tree
        // KdTreeFLANN is typically faster than pcl::search::KdTree
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud_a_ptr);

        int overlap_count = 0;
        double squared_dist_thresh = distance_threshold * distance_threshold;

        // 4. Parallel computation (OpenMP)
        // reduction(+:overlap_count) ensures thread-safe counting
        #pragma omp parallel for reduction(+:overlap_count) num_threads(4)
        for (size_t i = 0; i < cloud_b_ptr->size(); ++i) {
            const auto& point = cloud_b_ptr->points[i];

            // Declare vector inside parallel region for thread-local storage
            // Allocation overhead is negligible compared to parallel speedup
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            // Execute search
            if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                if (pointNKNSquaredDistance[0] < squared_dist_thresh) {
                    overlap_count++;
                }
            }
        }

        return static_cast<double>(overlap_count) / static_cast<double>(cloud_b_ptr->size());
    }
};


/**
 * 3D cuboid intersection volume server.
 *
 * Computes the intersection volume of two axis-aligned cuboids
 * defined by their 8 corner points using plane clipping and
 * the divergence theorem for convex polyhedron volume calculation.
 */
class CloudIntersectionServer {
private:
    struct Plane {
        float a, b, c, d;  ///< Plane equation: ax+by+cz+d=0 [--]
    };

public:
    CloudIntersectionServer() = default;
    ~CloudIntersectionServer() = default;

    /**
     * Compute intersection volume of two cuboids.
     *
     * @param[in] cuboid1  First cuboid (8 corners) [m]
     * @param[in] cuboid2  Second cuboid (8 corners) [m]
     * @return Intersection volume [m^3]
     */
    double calculateIntersectionVolume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid1,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid2) {
        // Validate input
        if (!validateInput(cuboid1, cuboid2)) {
            ROS_ERROR("Validation failed: each cuboid needs 8 vertices");
            return 0.0;
        }

        // Extract 6 faces of the cuboid
        std::vector<Plane> faces1 = extractFaces(cuboid1);
        std::vector<Plane> faces2 = extractFaces(cuboid2);

        // Separating axis theorem check
        if (areBoxesSeparated(cuboid1, cuboid2, faces1, faces2)) {
            return 0.0;
        }

        // Compute intersection polyhedron
        pcl::PointCloud<pcl::PointXYZ>::Ptr intersection = clipCuboidByPlanes(cuboid1, faces2);
        intersection = clipCuboidByPlanes(intersection, faces1);

        // Compute intersection volume
        return computeConvexPolyhedronVolume(intersection);
    }

private:
    /**
     * Validate input cuboids have 8 vertices each.
     *
     * @param[in] cuboid1  First cuboid [m]
     * @param[in] cuboid2  Second cuboid [m]
     * @return True if both cuboids have exactly 8 vertices [--]
     */
    bool validateInput(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid1,
                       const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid2) const {
        return (cuboid1->size() == 8 && cuboid2->size() == 8);
    }

    /**
     * Extract 6 faces from a cuboid defined by 8 corners.
     *
     * @param[in] cuboid  Cuboid corner points (8 vertices) [m]
     * @return List of 6 plane equations [--]
     */
    std::vector<Plane> extractFaces(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid) const {
        std::vector<Plane> faces;

        // Bottom face (0-1-2-3)
        faces.push_back(computePlane(cuboid->points[0], cuboid->points[1], cuboid->points[2]));
        // Top face (4-5-6-7)
        faces.push_back(computePlane(cuboid->points[4], cuboid->points[7], cuboid->points[6]));
        // Front face (0-3-7-4)
        faces.push_back(computePlane(cuboid->points[0], cuboid->points[4], cuboid->points[7]));
        // Back face (1-5-6-2)
        faces.push_back(computePlane(cuboid->points[1], cuboid->points[2], cuboid->points[6]));
        // Left face (0-1-5-4)
        faces.push_back(computePlane(cuboid->points[0], cuboid->points[1], cuboid->points[5]));
        // Right face (3-2-6-7)
        faces.push_back(computePlane(cuboid->points[3], cuboid->points[7], cuboid->points[6]));

        return faces;
    }

    /**
     * Compute plane equation from three points.
     *
     * @param[in] p1  First point [m]
     * @param[in] p2  Second point [m]
     * @param[in] p3  Third point [m]
     * @return Plane coefficients (a, b, c, d) [--]
     */
    Plane computePlane(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) const {
        Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        Eigen::Vector3f normal = v1.cross(v2).normalized();

        Plane plane;
        plane.a = normal.x();
        plane.b = normal.y();
        plane.c = normal.z();
        plane.d = -(normal.x() * p1.x + normal.y() * p1.y + normal.z() * p1.z);
        return plane;
    }

    /**
     * Check if a point lies in front of a plane.
     *
     * @param[in] point  Query point [m]
     * @param[in] plane  Plane equation [--]
     * @return True if point is in front of the plane [--]
     */
    bool isPointInFrontOfPlane(const pcl::PointXYZ& point, const Plane& plane) const {
        return plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d > 0;
    }

    /**
     * Check if two cuboids are separated using SAT.
     *
     * @param[in] cuboid1  First cuboid [m]
     * @param[in] cuboid2  Second cuboid [m]
     * @param[in] faces1   Faces of first cuboid [--]
     * @param[in] faces2   Faces of second cuboid [--]
     * @return True if cuboids are separated [--]
     */
    bool areBoxesSeparated(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid1,
                           const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid2,
                           const std::vector<Plane>& faces1,
                           const std::vector<Plane>& faces2) const {
        // Check all possible separating axes: face normals of both cuboids
        for (const auto& face : faces1) {
            Eigen::Vector3f axis(face.a, face.b, face.c);
            if (areShapesSeparated(cuboid1, cuboid2, axis))
                return true;
        }

        for (const auto& face : faces2) {
            Eigen::Vector3f axis(face.a, face.b, face.c);
            if (areShapesSeparated(cuboid1, cuboid2, axis))
                return true;
        }

        return false;
    }

    /**
     * Check if two shapes are separated along a given axis.
     *
     * @param[in] shape1  First point cloud [m]
     * @param[in] shape2  Second point cloud [m]
     * @param[in] axis    Separation axis [--]
     * @return True if projections do not overlap [--]
     */
    bool areShapesSeparated(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& shape1,
                           const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& shape2,
                           const Eigen::Vector3f& axis) const {
        float min1 = FLT_MAX, max1 = -FLT_MAX;
        float min2 = FLT_MAX, max2 = -FLT_MAX;

        // Project shape1 onto axis
        for (const auto& point : shape1->points) {
            float projection = point.x * axis.x() + point.y * axis.y() + point.z * axis.z();
            min1 = std::min(min1, projection);
            max1 = std::max(max1, projection);
        }

        // Project shape2 onto axis
        for (const auto& point : shape2->points) {
            float projection = point.x * axis.x() + point.y * axis.y() + point.z * axis.z();
            min2 = std::min(min2, projection);
            max2 = std::max(max2, projection);
        }

        // Check projection overlap
        return (max1 < min2) || (max2 < min1);
    }

    /**
     * Clip a cuboid by a set of half-planes.
     *
     * @param[in] cuboid  Cuboid to clip [m]
     * @param[in] planes  Clipping planes [--]
     * @return Clipped polyhedron [m]
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipCuboidByPlanes(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cuboid,
        const std::vector<Plane>& planes) const {
        pcl::PointCloud<pcl::PointXYZ>::Ptr result =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cuboid);

        for (const auto& plane : planes) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp =
                boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            // Clip each edge
            for (size_t i = 0; i < 12; ++i) {
                // 12 edges of the cuboid
                size_t v1_idx, v2_idx;
                getEdgeVertices(i, v1_idx, v2_idx);

                const pcl::PointXYZ& v1 = result->points[v1_idx];
                const pcl::PointXYZ& v2 = result->points[v2_idx];

                bool v1_in = isPointInFrontOfPlane(v1, plane);
                bool v2_in = isPointInFrontOfPlane(v2, plane);

                if (v1_in && v2_in) {
                    // Both in front, keep second point
                    temp->push_back(v2);
                } else if (v1_in && !v2_in) {
                    // First in front, second behind, compute intersection
                    pcl::PointXYZ intersection = computeIntersection(v1, v2, plane);
                    temp->push_back(intersection);
                } else if (!v1_in && v2_in) {
                    // First behind, second in front, compute intersection and keep second
                    pcl::PointXYZ intersection = computeIntersection(v1, v2, plane);
                    temp->push_back(intersection);
                    temp->push_back(v2);
                }
                // Both behind, keep nothing
            }

            // Remove duplicates
            removeDuplicatePoints(temp);
            result = temp;
        }

        return result;
    }

    /**
     * Compute intersection point of a line segment with a plane.
     *
     * @param[in] v1     Start point of segment [m]
     * @param[in] v2     End point of segment [m]
     * @param[in] plane  Clipping plane [--]
     * @return Intersection point [m]
     */
    pcl::PointXYZ computeIntersection(const pcl::PointXYZ& v1, const pcl::PointXYZ& v2, const Plane& plane) const {
        Eigen::Vector3f dir(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
        float denominator = plane.a * dir.x() + plane.b * dir.y() + plane.c * dir.z();

        if (std::abs(denominator) < 1e-6) {
            // Edge parallel to plane
            return v1;
        }

        // Manual clamp
        float t = -(plane.a * v1.x + plane.b * v1.y + plane.c * v1.z + plane.d) / denominator;
        t = std::max(0.0f, std::min(1.0f, t));

        pcl::PointXYZ result;
        result.x = v1.x + t * dir.x();
        result.y = v1.y + t * dir.y();
        result.z = v1.z + t * dir.z();
        return result;
    }

    /**
     * Remove duplicate points within a distance threshold.
     *
     * @param[inout] cloud  Point cloud to deduplicate [m]
     */
    void removeDuplicatePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
        const float threshold = 1e-4;
        std::vector<bool> keep(cloud->size(), true);

        for (size_t i = 0; i < cloud->size(); ++i) {
            if (!keep[i]) continue;

            for (size_t j = i + 1; j < cloud->size(); ++j) {
                if (!keep[j]) continue;

                float dx = cloud->points[i].x - cloud->points[j].x;
                float dy = cloud->points[i].y - cloud->points[j].y;
                float dz = cloud->points[i].z - cloud->points[j].z;

                if (dx*dx + dy*dy + dz*dz < threshold*threshold) {
                    keep[j] = false;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (keep[i]) {
                temp->push_back(cloud->points[i]);
            }
        }

        *cloud = *temp;
    }

    /**
     * Get the two vertex indices for a given edge.
     *
     * @param[in]  edge_idx  Edge index (0-11) [--]
     * @param[out] v1        First vertex index [--]
     * @param[out] v2        Second vertex index [--]
     */
    void getEdgeVertices(size_t edge_idx, size_t& v1, size_t& v2) const {
        // Look up vertex indices for given edge index
        // 12 edges defined by cuboid vertex ordering
        static const size_t edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0},  // Bottom edges
            {4, 5}, {5, 6}, {6, 7}, {7, 4},  // Top edges
            {0, 4}, {1, 5}, {2, 6}, {3, 7}   // Vertical edges
        };

        v1 = edges[edge_idx][0];
        v2 = edges[edge_idx][1];
    }

    /**
     * Compute volume of a convex polyhedron using divergence theorem.
     *
     * @param[in] polyhedron  Convex polyhedron vertices [m]
     * @return Volume [m^3]
     */
    double computeConvexPolyhedronVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr& polyhedron) const {
        if (polyhedron->size() < 4) return 0.0;

        // Simplified volume calculation
        // Full convex polyhedron volume calculation needed for production

        // Example: tetrahedron case
        if (polyhedron->size() == 4) {
            Eigen::Vector3f v0(polyhedron->points[0].x, polyhedron->points[0].y, polyhedron->points[0].z);
            Eigen::Vector3f v1(polyhedron->points[1].x, polyhedron->points[1].y, polyhedron->points[1].z);
            Eigen::Vector3f v2(polyhedron->points[2].x, polyhedron->points[2].y, polyhedron->points[2].z);
            Eigen::Vector3f v3(polyhedron->points[3].x, polyhedron->points[3].y, polyhedron->points[3].z);

            // Tetrahedron volume formula
            return std::abs((v1 - v0).dot((v2 - v0).cross(v3 - v0))) / 6.0;
        }

        // More complex polyhedra need advanced computation
        // Return 0 as placeholder
        return 0.0;
    }
};

#endif //BOX_INTERSECTION_SERVER_H
