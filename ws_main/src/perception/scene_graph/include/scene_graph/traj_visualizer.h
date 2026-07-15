// Created by gwq on 9/23/25.
//

#ifndef TRAJ_VISUALIZER_H
#define TRAJ_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>

/**
 * Trajectory visualizer with speed-based color gradient.
 *
 * Publishes a LINE_STRIP marker with per-point color based on
 * instantaneous velocity (blue=slow, red=fast). Used for
 * visualizing executed trajectories in RViz.
 */
class TrajectoryVisualizer {
public:
    TrajectoryVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param<std::string>("/traj_vis/map_frame_id", map_frame_id_, "world");
        nh_.param<double>("/traj_vis/min_speed", min_speed_, 0.0);   ///< [m/s]
        nh_.param<double>("/traj_vis/max_speed", max_speed_, 0.65);  ///< [m/s]
        nh_.param<double>("/traj_vis/distance_threshold", distance_threshold_, 0.1); ///< [m]

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/scene_graph/traj_with_vel", 10);
        initMarker();
    }

    /**
     * Add a trajectory point with velocity for speed-based coloring.
     *
     * @param[in] pos  Point position [m]
     * @param[in] vel  Instantaneous velocity [m/s]
     */
    void addPoint(const Eigen::Vector3d& pos, double vel) {
        // Skip if distance to last point is below threshold (suppresses duplicates)
        if (calculateDistance(pos, last_point_) > distance_threshold_) {

            geometry_msgs::Point new_ros_point;
            new_ros_point.x = pos.x();
            new_ros_point.y = pos.y();
            new_ros_point.z = pos.z();

            trajectory_marker_.points.push_back(new_ros_point);
            trajectory_marker_.colors.push_back(speedToColor(vel));

            trajectory_marker_.header.stamp = ros::Time::now();
            marker_pub_.publish(trajectory_marker_);

            last_point_ = pos;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker trajectory_marker_;

    std::string odom_topic_;
    std::string map_frame_id_;
    double min_speed_;            ///< Minimum speed for color mapping [m/s]
    double max_speed_;            ///< Maximum speed for color mapping [m/s]
    double distance_threshold_;   ///< Minimum distance between trajectory points [m]

    Eigen::Vector3d last_point_;  ///< Previous point for distance check [m]

    /**
     * Euclidean distance between two 3D points.
     *
     * @param[in] p1  First point [m]
     * @param[in] p2  Second point [m]
     * @return Distance [m]
     */
    double calculateDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return (p1 - p2).norm();
    }

    /**
     * Initialize the LINE_STRIP marker with default parameters.
     */
    void initMarker() {
        trajectory_marker_.header.frame_id = map_frame_id_;
        trajectory_marker_.header.stamp = ros::Time::now();
        trajectory_marker_.ns = "trajectory";
        trajectory_marker_.id = 0;
        trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory_marker_.action = visualization_msgs::Marker::ADD;
        trajectory_marker_.pose.orientation.w = 1.0;
        trajectory_marker_.scale.x            = 0.05;
    }

    /**
     * Map speed to a color (blue=slow, cyan=medium, red=fast).
     *
     * @param[in] speed  Instantaneous velocity [m/s]
     * @return RGBA color
     */
    std_msgs::ColorRGBA speedToColor(double speed) {
        double normalized_speed = (speed - min_speed_) / (max_speed_ - min_speed_);
        normalized_speed = std::max(0.0, std::min(1.0, normalized_speed));
        double hue = (1.0 - normalized_speed) * 240.0 / 360.0;
        return hsvToRgb(hue, 1.0, 1.0);
    }

    /**
     * Convert HSV to RGBA color.
     *
     * @param[in] h  Hue [0, 1] [--]
     * @param[in] s  Saturation [0, 1] [--]
     * @param[in] v  Value [0, 1] [--]
     * @return RGBA color
     */
    std_msgs::ColorRGBA hsvToRgb(double h, double s, double v) {
        std_msgs::ColorRGBA rgb_color;
        rgb_color.a = 0.8;
        int i = floor(h * 6);
        double f = h * 6 - i;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);

        switch (i % 6) {
            case 0: rgb_color.r = v, rgb_color.g = t, rgb_color.b = p; break;
            case 1: rgb_color.r = q, rgb_color.g = v, rgb_color.b = p; break;
            case 2: rgb_color.r = p, rgb_color.g = v, rgb_color.b = t; break;
            case 3: rgb_color.r = p, rgb_color.g = q, rgb_color.b = v; break;
            case 4: rgb_color.r = t, rgb_color.g = p, rgb_color.b = v; break;
            case 5: rgb_color.r = v, rgb_color.g = p, rgb_color.b = q; break;
        }
        return rgb_color;
    }
};

#endif //TRAJ_VISUALIZER_H