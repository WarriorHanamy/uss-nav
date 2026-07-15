#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <algorithm> // required for std::lower_bound

class FovWorldFilterNode {
public:
    FovWorldFilterNode() {
        nh_.reset(new ros::NodeHandle("~"));

        // Parameter config
        nh_->param<std::string>("odom_topic", odom_topic_, "/ekf_quat/ekf_odom");
        nh_->param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        nh_->param<std::string>("output_frame", output_frame_, "world");
        
        // Fix: use floating-point division to avoid integer truncation to zero
        nh_->param<double>("fov_limit_angle", fov_limit_angle_, 45.0 / 180.0 * M_PI);

        // Time sync tolerance
        nh_->param<double>("time_sync_threshold", time_sync_threshold_, 0.05); // slightly relaxed threshold

        // Distance limit parameters
        nh_->param<double>("max_distance", max_distance_, 50.0); // max distance limit [m]

        odom_sub_ = nh_->subscribe(odom_topic_, 100, &FovWorldFilterNode::odomCallback, this);
        cloud_sub_ = nh_->subscribe(cloud_topic_, 2, &FovWorldFilterNode::cloudCallback, this);
        cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("/cloud_fov_limited", 2);

        ROS_INFO("FovWorldFilterNode initialized. Method: Optimized Binary Search.");
    }

private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    std::string odom_topic_;
    std::string cloud_topic_;
    std::string output_frame_;
    double fov_limit_angle_;
    double time_sync_threshold_;
    double max_distance_;

    std::deque<nav_msgs::Odometry> odom_queue_;
    std::mutex odom_mutex_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_queue_.push_back(*msg);
        
        // Keep queue size moderate to avoid OOM, but long enough to cover latency
        if (odom_queue_.size() > 5000) odom_queue_.pop_front();
    }

    /**
     * @brief Find closest odometry by timestamp using binary search
     * Performance: O(log N)
     */
    bool findClosestOdom(const ros::Time& cloud_time, nav_msgs::Odometry& result_odom) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        if (odom_queue_.empty()) return false;

        // 1. Fast boundary check: return early if query time is out of range
        if (cloud_time < odom_queue_.front().header.stamp - ros::Duration(time_sync_threshold_) ||
            cloud_time > odom_queue_.back().header.stamp + ros::Duration(time_sync_threshold_)) {
            return false;
        }

        // 2. Binary search (std::lower_bound)
        // Find first iterator with header.stamp >= cloud_time
        auto lower = std::lower_bound(odom_queue_.begin(), odom_queue_.end(), cloud_time,
            [](const nav_msgs::Odometry& msg, const ros::Time& t) {
                return msg.header.stamp < t;
            });

        // 3. Find nearest neighbor
        // lower points to element >= time. Compare lower and prev(lower) for closest
        auto best_it = odom_queue_.end();
        double min_diff = std::numeric_limits<double>::max();

        // Check element at lower (right neighbor)
        if (lower != odom_queue_.end()) {
            double diff = std::fabs((lower->header.stamp - cloud_time).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                best_it = lower;
            }
        }

        // Check previous element (left neighbor)
        if (lower != odom_queue_.begin()) {
            auto prev = std::prev(lower);
            double diff = std::fabs((prev->header.stamp - cloud_time).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                best_it = prev;
            }
        }

        // 4. Verify threshold
        if (best_it != odom_queue_.end() && min_diff <= time_sync_threshold_) {
            result_odom = *best_it;
            return true;
        }

        return false;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // 1. Find matching Odometry (high-performance)
        nav_msgs::Odometry best_odom;
        if (!findClosestOdom(cloud_msg->header.stamp, best_odom)) {
            // Use THROTTLE to avoid spam, suggest adjusting queue size or threshold
            ROS_WARN_THROTTLE(1.0, "Odom sync failed. Cloud Time: %.3f. Queue Size: %lu", 
                              cloud_msg->header.stamp.toSec(), odom_queue_.size());
            return;
        }

        // 2. Build transform matrix T_world_to_body
        // Use float (Affine3f) for faster point cloud processing
        const auto& pos = best_odom.pose.pose.position;
        const auto& ori = best_odom.pose.pose.orientation;
        
        Eigen::Translation3f translation(pos.x, pos.y, pos.z);
        Eigen::Quaternionf quaternion(ori.w, ori.x, ori.y, ori.z);
        
        // Odom is Body->World, we need inverse World->Body
        Eigen::Affine3f t_world_to_body = (translation * quaternion.toRotationMatrix()).inverse();

        // 3. Convert input point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Pre-allocate memory to avoid repeated reallocation during push_back
        output_cloud->points.reserve(input_cloud->size());

        // Cache limit values, avoid casting inside loop
        float limit_angle_f = static_cast<float>(fov_limit_angle_);
        float max_distance_f = static_cast<float>(max_distance_);

        // 4. Iterate and filter (compute-intensive section)
        for (const auto& world_point : input_cloud->points) {
            // P_body = T_world_to_body * P_world
            // Manual unrolled multiply is slightly faster, but Eigen with -O3 is comparable and more readable
            Eigen::Vector3f pt_world_vec(world_point.x, world_point.y, world_point.z);
            Eigen::Vector3f pt_body_vec = t_world_to_body * pt_world_vec;

            // --- Performance optimization ---
            // Assume FOV is front-facing sector < 180 deg (e.g. +/- 80 deg)
            // If x < 0, point is behind, skip expensive atan2
            if (pt_body_vec.x() < 0.001f) continue; 

            // Use atan2f (float) instead of atan2 (double)
            float angle = std::atan2(pt_body_vec.y(), pt_body_vec.x());

            // Check both FOV angle and distance limit
            if (std::fabs(angle) <= limit_angle_f) {
                // Compute distance to UAV (in body frame)
                float distance = pt_body_vec.norm();
                
                // If distance within limit, keep original world-frame point
                if (distance <= max_distance_f) {
                    output_cloud->points.push_back(world_point);
                }
            }
        } 

        // 5. Publish
        if (!output_cloud->empty()) {
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*output_cloud, output_msg);
            
            output_msg.header = cloud_msg->header; 
            output_msg.header.frame_id = output_frame_;

            cloud_pub_.publish(output_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fov_world_filter_node");
    FovWorldFilterNode node;
    ros::spin();
    return 0;
}