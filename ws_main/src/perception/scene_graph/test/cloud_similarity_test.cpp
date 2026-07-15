#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
// Assumes pt_cloud_tools.h and INFO_MSG macro are correctly included and defined
// #include <../include/scene_graph/pt_cloud_tools.h>

#include <iostream>
#include <cmath>
#include <random>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

// Helper: convert hex color string to RGB struct
struct RgbColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// Accept hex string (e.g. "#FF0000") and return RgbColor
RgbColor hexToRgb(const std::string& hex_color) {
    if (hex_color.length() != 7 || hex_color[0] != '#') {
        // Default to white
        return {255, 255, 255};
    }

    // Parse R, G, B hex values from positions 1, 3, 5
    unsigned int r_val, g_val, b_val;
    std::stringstream ss;

    // Parse R
    ss << std::hex << hex_color.substr(1, 2);
    ss >> r_val;
    ss.clear(); // Clear state flags

    // Parse G
    ss << std::hex << hex_color.substr(3, 2);
    ss >> g_val;
    ss.clear();

    // Parse B
    ss << std::hex << hex_color.substr(5, 2);
    ss >> b_val;

    return {(uint8_t)r_val, (uint8_t)g_val, (uint8_t)b_val};
}

// 📌 Change: function now accepts a color parameter (hex string)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateRandomPointsInSphere(
    const Eigen::Vector4f& sphere_center,
    float sphere_radius,
    int num_points,
    const std::string& hex_color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-sphere_radius, sphere_radius);

    // Convert color parameter to RGB struct
    RgbColor color = hexToRgb(hex_color);

    for (int i = 0; i < num_points; ++i)
    {
        pcl::PointXYZRGB point;
        float x, y, z;
        float distance;

        do {
            x = dis(gen);
            y = dis(gen);
            z = dis(gen);
            distance = std::sqrt(x * x + y * y + z * z);
        } while (distance > sphere_radius);

        point.x = x + sphere_center[0];
        point.y = y + sphere_center[1];
        point.z = z + sphere_center[2];

        // 📌 Change: use fixed color
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;

        cloud->points[i] = point;
    }

    return cloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_points_in_sphere");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_test", 1);

    // 📌 Define two different colors (e.g. red and blue)
    const std::string color1_hex = "#FF0000"; // red
    const std::string color2_hex = "#0000FF"; // blue
    int num_points = 10000;
    float radius = 1.0;

    // 📌 Change: pass specified color on call
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = generateRandomPointsInSphere(
        Eigen::Vector4f(0.0, 0.0, 0.0, 1.0), radius, num_points, color1_hex);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = generateRandomPointsInSphere(
        Eigen::Vector4f(1.0, 0.0, 0.0, 1.0), radius, num_points, color2_hex);

    // --- Omitted irrelevant parts of original code, e.g. pt_cloud_tools.h dependency ---

    // Voxel filter both point clouds
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize(0.07f, 0.07f, 0.07f);

    sor.setInputCloud(cloud1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*filtered_cloud1);

    sor.setInputCloud(cloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*filtered_cloud2);

    // Reassign to cloud1 and cloud2 (if you want filtered versions for downstream computation)
    cloud1 = filtered_cloud1;
    cloud2 = filtered_cloud2;


    // Assume INFO_MSG is defined, otherwise comment out or replace with ROS_INFO
    // INFO_MSG("cloud1 size : " << cloud1->size() << " cloud2 size : " << cloud2->size());
    ROS_INFO("cloud1 size : %zu, cloud2 size : %zu", cloud1->size(), cloud2->size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Merge point clouds
    *cloud_all = *cloud1;
    *cloud_all += *cloud2;

    // Assume PointCloudOverlapCalculator is available, otherwise comment out
    /*
    PointCloudOverlapCalculator cloud_similarity_server{};
    std::cout << "cloud similarity score: " << cloud_similarity_server.calculateOverlapBInA(cloud1, cloud2, 0.07f)<< std::endl;
    */

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_all, output);
    output.header.frame_id = "world";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}