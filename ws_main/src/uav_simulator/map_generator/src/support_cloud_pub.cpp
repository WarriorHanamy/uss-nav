/**
 * support_cloud_pub.cpp - 仿真 prior_local 支撑点云合并节点 (C++ / PCL)
 *
 * 移植自 FAST_LIO (/home/zhywwyzh/workspace/FAST_LIO) 的 prior_local 机制:
 * 加载支撑PCD, 离群滤波 + 体素降采样一次并建 KdTree; 以无人机当前位姿为中心,
 * 按 prior_local_mode 截取先验局部点云, 合并到 pcl_render_node 模拟扫描结果中发布。
 *
 *   prior_local_mode = 0: 截球 (用 radius)
 *   prior_local_mode = 1: 长方体 (yaw 跟随设备朝向, 上下恒沿世界 z 轴, 用 6 方向延伸距离)
 *                         先用长方体外接球做 radiusSearch 粗筛 (FLANN 无长方体查询接口),
 *                         再把候选点转到 "yaw 对齐、z 竖直" 局部系按 6 方向边界精筛。
 *
 * 移动阈值优化 (prior_local_motion_*): 位移不足时复用上次结果; 模式1下原地转 yaw 超阈值仍重算。
 *
 * 参数命名对齐实机 FAST_LIO (1:1, 仅 support_pcd_path 例外):
 *   support_map_pcd_name   → ~support_pcd_path   (FAST_LIO 是 ROOT_DIR/PCD 相对名, 本节点传入完整路径)
 *   prior_local_radius      → ~prior_local_radius
 *   prior_local_leaf        → ~prior_local_leaf
 *   prior_local_mode        → ~prior_local_mode
 *   prior_box_extend_*      → ~prior_box_extend_*
 *   prior_local_motion_*    → ~prior_local_motion_*
 *   prior_outlier_filter/<key> → ~prior_outlier_filter/<key>
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

typedef pcl::PointXYZ PointT;

class SupportCloudPub
{
public:
  SupportCloudPub(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), has_odom_(false), latest_odom_yaw_(0.0f),
      has_cache_(false), last_yaw_(0.0f)
  {
    // ---- 参数 ----
    pnh_.param<std::string>("support_pcd_path", support_pcd_path_, "");
    pnh_.param("prior_local_radius", radius_, 5.0);
    pnh_.param("prior_local_leaf", leaf_size_, 0.1);

    // 截取形状: 0=球(用 radius), 1=长方体(用下方 6 方向延伸距离)
    pnh_.param("prior_local_mode", prior_local_mode_, 0);
    pnh_.param("prior_box_extend_forward",  box_fwd_,   10.0);
    pnh_.param("prior_box_extend_backward", box_bwd_,   10.0);
    pnh_.param("prior_box_extend_left",     box_left_,  10.0);
    pnh_.param("prior_box_extend_right",    box_right_, 10.0);
    pnh_.param("prior_box_extend_up",       box_up_,    5.0);
    pnh_.param("prior_box_extend_down",     box_down_,  5.0);

    // 移动阈值优化 (对齐 FAST_LIO prior_local_motion_*)
    pnh_.param("prior_local_motion_check",      motion_check_,      false);
    pnh_.param("prior_local_motion_thresh",     motion_thresh_,     0.5);
    pnh_.param("prior_local_motion_yaw_thresh", motion_yaw_thresh_, 0.1);

    // 离群点滤波 (对齐 FAST_LIO prior_outlier_filter)
    pnh_.param("prior_outlier_filter/enable", outlier_enable_, true);
    pnh_.param("prior_outlier_filter/type", outlier_type_, 1);
    pnh_.param("prior_outlier_filter/radius", outlier_ror_radius_, 1.0);
    pnh_.param("prior_outlier_filter/min_neighbors", outlier_ror_min_nb_, 3);
    pnh_.param("prior_outlier_filter/mean_k", outlier_sor_mean_k_, 20);
    pnh_.param("prior_outlier_filter/std_threshold", outlier_sor_std_thresh_, 2.0);

    prior_cache_.reset(new pcl::PointCloud<PointT>);

    // ---- 先创建发布/订阅 (让加载期间cloud也能直通) ----
    cloud_merged_pub_   = pnh_.advertise<sensor_msgs::PointCloud2>("cloud_merged", 10);
    support_only_pub_   = pnh_.advertise<sensor_msgs::PointCloud2>("support_only", 10);
    cloud_sub_ = pnh_.subscribe<sensor_msgs::PointCloud2>("cloud", 10,
                                                           &SupportCloudPub::cloudCallback, this);
    odom_sub_ = pnh_.subscribe<nav_msgs::Odometry>("odom", 50,
                                                   &SupportCloudPub::odomCallback, this);

    // ---- 加载支撑 PCD (阻塞, 期间cloud直通) ----
    loadSupportPCD();

    int n_pts = support_cloud_ ? (int)support_cloud_->points.size() : 0;
    ROS_INFO("[SupportCloud] init done, mode=%d, support_pts=%d, radius=%.1fm, leaf=%.2fm",
             prior_local_mode_, n_pts, radius_, leaf_size_);
    if (prior_local_mode_ == 1)
      ROS_INFO("[SupportCloud] box extend(F/B/L/R/U/D)=%.2f/%.2f/%.2f/%.2f/%.2f/%.2f",
               box_fwd_, box_bwd_, box_left_, box_right_, box_up_, box_down_);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher  cloud_merged_pub_, support_only_pub_;
  ros::Subscriber cloud_sub_, odom_sub_;

  // 支撑点云 (载入时降采样一次, KdTree 建于降采样图上)
  pcl::PointCloud<PointT>::Ptr support_cloud_;
  pcl::KdTreeFLANN<PointT>::Ptr support_kdtree_;

  // odom
  bool has_odom_;
  Eigen::Vector3f latest_odom_pos_;
  float latest_odom_yaw_;          // 绕世界 z 轴偏航 (模式1长方体朝向用)

  // 先验局部点云缓存 (移动阈值优化)
  pcl::PointCloud<PointT>::Ptr prior_cache_;
  bool has_cache_;
  Eigen::Vector3f last_pos_;
  float last_yaw_;

  // 参数
  std::string support_pcd_path_;
  double radius_, leaf_size_;
  int    prior_local_mode_;
  double box_fwd_, box_bwd_, box_left_, box_right_, box_up_, box_down_;
  bool   motion_check_;
  double motion_thresh_, motion_yaw_thresh_;
  bool   outlier_enable_;
  int    outlier_type_;
  double outlier_ror_radius_;
  int    outlier_ror_min_nb_;
  int    outlier_sor_mean_k_;
  double outlier_sor_std_thresh_;

  // ================================================================
  void loadSupportPCD()
  {
    if (support_pcd_path_.empty())
    {
      ROS_WARN("[SupportCloud] support_pcd_path is empty");
      support_cloud_.reset(new pcl::PointCloud<PointT>);
      return;
    }

    // 用 PCLPointCloud2 中转, 正确处理多 COUNT padding 字段的 PCD
    pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
    if (pcl::io::loadPCDFile(support_pcd_path_, *blob) == -1)
    {
      ROS_ERROR("[SupportCloud] cannot load PCD: %s", support_pcd_path_.c_str());
      support_cloud_.reset(new pcl::PointCloud<PointT>);
      return;
    }
    pcl::PointCloud<PointT>::Ptr raw(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*blob, *raw);
    // 验证前3个点
    for (int i = 0; i < std::min(3, (int)raw->points.size()); i++)
      ROS_INFO("[SupportCloud]   pt[%d] = (%.3f, %.3f, %.3f)  isFinite=%d",
               i, raw->points[i].x, raw->points[i].y, raw->points[i].z,
               pcl::isFinite(raw->points[i]));
    ROS_INFO("[SupportCloud] loaded PCD: %s, %zu pts",
             support_pcd_path_.c_str(), raw->points.size());

    // 离群点滤波 (PCL的ROR/SOR内部会处理NaN)
    if (outlier_enable_ && !raw->empty())
      raw = outlierFilter(raw);

    // 体素降采样一次 (对齐 FAST_LIO prior_local_leaf), KdTree 建于降采样图
    pcl::PointCloud<PointT>::Ptr ds(new pcl::PointCloud<PointT>);
    if (leaf_size_ > 0.0 && raw->points.size() > 1)
    {
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(raw);
      voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      voxel.filter(*ds);
    }
    else
    {
      ds = raw;
    }

    support_cloud_ = ds;
    support_kdtree_.reset(new pcl::KdTreeFLANN<PointT>);
    if (!support_cloud_->empty())
      support_kdtree_->setInputCloud(support_cloud_);

    ROS_INFO("[SupportCloud] support cloud ready: %zu pts (downsampled, leaf=%.2f)",
             support_cloud_->points.size(), leaf_size_);
  }

  // ----------------------------------------------------------------
  pcl::PointCloud<PointT>::Ptr outlierFilter(pcl::PointCloud<PointT>::Ptr cloud)
  {
    size_t before = cloud->points.size();

    if (outlier_type_ == 1)
    {
      // ROR
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(cloud);
      ror.setRadiusSearch(outlier_ror_radius_);
      ror.setMinNeighborsInRadius(outlier_ror_min_nb_);
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
      ror.filter(*filtered);
      cloud = filtered;
    }
    else if (outlier_type_ == 2)
    {
      // SOR
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(outlier_sor_mean_k_);
      sor.setStddevMulThresh(outlier_sor_std_thresh_);
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
      sor.filter(*filtered);
      cloud = filtered;
    }

    size_t removed = before - cloud->points.size();
    if (removed > 0)
      ROS_INFO("[SupportCloud] outlier filter(type=%d): %zu → %zu (removed %zu, %.1f%%)",
               outlier_type_, before, cloud->points.size(),
               removed, 100.0 * removed / before);
    return cloud;
  }

  // ================================================================
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    latest_odom_pos_ = Eigen::Vector3f(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
    // 从四元数提取 yaw (仅绕世界 z 轴, 与 pitch/roll 无关), 对齐 FAST_LIO atan2(R(1,0),R(0,0))
    Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();
    latest_odom_yaw_ = std::atan2(R(1, 0), R(0, 0));
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (!has_odom_)
    {
      cloud_merged_pub_.publish(msg);
      return;
    }

    // 1) 提取先验局部点云 (mode 0 球 / mode 1 长方体)
    pcl::PointCloud<PointT>::Ptr support_pts =
        extractSupportPoints(latest_odom_pos_, latest_odom_yaw_);

    // 2) 转换原始 cloud (模拟雷达扫描)
    pcl::PointCloud<PointT>::Ptr orig_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *orig_cloud);

    // 3) 单独发布先验支撑点云 (供调试/可视化区分来源)
    if (support_pts && !support_pts->empty())
    {
      sensor_msgs::PointCloud2 support_msg;
      pcl::toROSMsg(*support_pts, support_msg);
      support_msg.header = msg->header;
      support_only_pub_.publish(support_msg);
    }

    // 4) 合并 → 雷达扫描 + 先验支撑 (并集)
    pcl::PointCloud<PointT> merged;
    merged += *orig_cloud;
    if (support_pts && !support_pts->empty())
      merged += *support_pts;

    // 5) 发布合并后点云
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(merged, out);
    out.header = msg->header;
    cloud_merged_pub_.publish(out);
  }

  // ----------------------------------------------------------------
  // 提取先验局部点云 (mode 0 截球 / mode 1 长方体), 含移动阈值缓存复用
  pcl::PointCloud<PointT>::Ptr extractSupportPoints(const Eigen::Vector3f &center, float yaw)
  {
    if (!support_cloud_ || support_cloud_->empty() || !support_kdtree_)
      return prior_cache_;

    // 移动阈值优化 (对齐 FAST_LIO prior_local_thread 696-711):
    // 位移不足则复用缓存; 模式1下原地转 yaw 超阈值仍需重算 (长方体朝向跟随 yaw)
    if (motion_check_ && has_cache_ &&
        (center - last_pos_).norm() < static_cast<float>(motion_thresh_))
    {
      bool need_recompute = false;
      if (prior_local_mode_ == 1)
      {
        float dyaw = yaw - last_yaw_;
        while (dyaw >  static_cast<float>(M_PI)) dyaw -= 2.0f * static_cast<float>(M_PI);
        while (dyaw < -static_cast<float>(M_PI)) dyaw += 2.0f * static_cast<float>(M_PI);
        if (std::fabs(dyaw) >= static_cast<float>(motion_yaw_thresh_))
          need_recompute = true;
      }
      if (!need_recompute)
        return prior_cache_;
    }

    PointT search_pt;
    search_pt.x = center.x();
    search_pt.y = center.y();
    search_pt.z = center.z();
    std::vector<int> indices;
    std::vector<float> sq_dists;

    pcl::PointCloud<PointT>::Ptr pts(new pcl::PointCloud<PointT>);

    if (prior_local_mode_ == 0)
    {
      // 模式0: 截球 (KdTree 半径搜索)
      support_kdtree_->radiusSearch(search_pt, radius_, indices, sq_dists);
      pts->points.reserve(indices.size());
      for (int idx : indices)
        pts->points.push_back(support_cloud_->points[idx]);
    }
    else
    {
      // 模式1: 长方体截取 (yaw 跟随设备, z 轴恒为世界竖直)
      // 阶段1: 用长方体外接球做 radiusSearch 粗筛 (FLANN 无长方体查询接口)
      //        外接球半径 = 长方体中心到最远顶点距离
      double rx = std::max(box_fwd_, box_bwd_);
      double ry = std::max(box_left_, box_right_);
      double rz = std::max(box_up_, box_down_);
      double bound_r = std::sqrt(rx * rx + ry * ry + rz * rz);

      support_kdtree_->radiusSearch(search_pt, bound_r, indices, sq_dists);

      // 阶段2: 候选点转到 "yaw 对齐、z 竖直" 局部系做长方体边界判断
      double cy = std::cos(yaw);
      double sy = std::sin(yaw);
      pts->points.reserve(indices.size());
      for (int idx : indices)
      {
        const PointT &p = support_cloud_->points[idx];
        double dx = p.x - center.x();
        double dy = p.y - center.y();
        double dz = p.z - center.z();
        double lx =  cy * dx + sy * dy;   // 前向 (绕世界 z 轴反旋 yaw)
        double ly = -sy * dx + cy * dy;   // 左向
        double lz =  dz;                  // 上向 (竖直, 与 pitch/roll 无关)

        if (lx >= -box_bwd_   && lx <= box_fwd_  &&
            ly >= -box_right_ && ly <= box_left_ &&
            lz >= -box_down_  && lz <= box_up_)
          pts->points.push_back(p);
      }
    }

    pts->width = pts->points.size();
    pts->height = 1;
    pts->is_dense = true;

    // 更新缓存
    prior_cache_ = pts;
    has_cache_ = true;
    last_pos_ = center;
    last_yaw_ = yaw;
    return pts;
  }
};

// ================================================================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "support_cloud_pub");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SupportCloudPub node(nh, pnh);
  ros::spin();
  return 0;
}
