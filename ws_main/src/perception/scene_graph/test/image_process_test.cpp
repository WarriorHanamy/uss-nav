//
// Created by gwq on 12/7/25.
//
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

// Define sync policy: use approximate time sync due to possible transmission delay
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::CompressedImage,
    sensor_msgs::CompressedImage
> MySyncPolicy;

class RealsenseProcessor {
private:
    ros::NodeHandle nh;

    // Message filter subscribers
    message_filters::Subscriber<sensor_msgs::CompressedImage> color_sub;
    message_filters::Subscriber<sensor_msgs::CompressedImage> depth_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    // Camera intrinsics and distortion parameters
    cv::Mat K, D;
    cv::Mat map1, map2;
    cv::Size image_size;
    cv::Mat new_camera_matrix;

public:
    RealsenseProcessor() :
        sync(MySyncPolicy(10), color_sub, depth_sub), // Sync queue length 10
        image_size(640, 480)
    {
        // 1. Set subscription topics
        // Confirm topic names based on actual setup
        // If align_depth is enabled, depth topic usually contains aligned_depth_to_color
        std::string color_topic = "/camera/color/image_raw/compressed";
        std::string depth_topic = "/camera/aligned_depth_to_color/image_raw/compressedDepth";

        ROS_INFO("Subscribing to:\n  Color: %s\n  Depth: %s", color_topic.c_str(), depth_topic.c_str());

        color_sub.subscribe(nh, color_topic, 1);
        depth_sub.subscribe(nh, depth_topic, 1);

        // Register callback
        sync.registerCallback(boost::bind(&RealsenseProcessor::callback, this, _1, _2));

        // 2. Initialize calibration parameters
        initCalibration();
    }

    // Initialize intrinsics and undistortion maps
    void initCalibration() {
        // Intrinsics K (based on data provided earlier)
        double k_data[] = {
            387.3385009765625, 0.0, 321.9053649902344,
            0.0, 386.7434387207031, 245.8605499267578,
            0.0, 0.0, 1.0
        };
        K = cv::Mat(3, 3, CV_64F, k_data);

        // Distortion coefficients D
        double d_data[] = {
            -0.0561770461499691, 0.06432759761810303,
            -9.787999260879587e-06, 0.00038565468275919557,
            -0.0207914300262928
        };
        D = cv::Mat(1, 5, CV_64F, d_data);

        // Precompute remap tables (Init Undistort Maps)
        // alpha=1 retains all pixels, alpha=0 crops black borders
        new_camera_matrix = cv::getOptimalNewCameraMatrix(K, D, image_size, 1, image_size, 0);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), new_camera_matrix,
                                    image_size, CV_16SC2, map1, map2);

        ROS_INFO("Calibration initialized.");
    }

    // --- Corrected depth decoding function ---
    cv::Mat decodeRealsenseCompressedDepth(const sensor_msgs::CompressedImageConstPtr &msg) {
        // 1. Validate format
        // Standard ROS compressedDepth header length is 12 bytes
        const size_t header_size = 12;

        if (msg->data.size() <= header_size) {
            ROS_ERROR("Compressed depth data is too short!");
            return cv::Mat();
        }

        // 2. Parse header parameters (optional, for debugging)
        // [0-3: config/enum], [4-7: depth_max], [8-11: depth_quantization]
        // Here we mainly care about decompression, skip directly, but keep reading logic for reference
        float depth_quant_a, depth_quant_b;
        memcpy(&depth_quant_a, &msg->data[4], sizeof(float));
        memcpy(&depth_quant_b, &msg->data[8], sizeof(float));
        // ROS_DEBUG("Depth Quant parameters: %f, %f", depth_quant_a, depth_quant_b);

        // 3. Core step: skip 12-byte header, extract raw image data
        // Note: need to slice from msg->data (vector)
        const std::vector<uint8_t> imageData(msg->data.begin() + header_size, msg->data.end());

        // 4. Decode with OpenCV
        // Key flag: cv::IMREAD_UNCHANGED (or -1).
        // Only this flag guarantees decoding 16-bit (CV_16U) raw depth, otherwise it gets converted to 8-bit BGR.
        cv::Mat decoded_img = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

        if (decoded_img.empty()) {
            ROS_ERROR("Failed to decode compressed depth image (imdecode returned empty)");
            return cv::Mat();
        }
        return decoded_img;
    }

    // Helper: convert 16-bit depth to pseudo-color for display
    cv::Mat colorizeDepth(const cv::Mat& depth_16u) {
        cv::Mat depth_8u, depth_color;
        // Normalize: assume 0-3m (3000mm) range
        depth_16u.convertTo(depth_8u, CV_8UC1, 255.0 / 3000.0);
        cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_JET);
        return depth_color;
    }

    void callback(const sensor_msgs::CompressedImageConstPtr& color_msg,
                  const sensor_msgs::CompressedImageConstPtr& depth_msg)
    {
        try {
            // --- A. Decode Color (standard JPEG compression) ---
            cv::Mat color_raw = cv::imdecode(cv::Mat(color_msg->data), cv::IMREAD_COLOR);

            // --- B. Decode Depth (use corrected function for compressedDepth) ---
            cv::Mat depth_raw = decodeRealsenseCompressedDepth(depth_msg);

            if (color_raw.empty() || depth_raw.empty()) return;

            // --- C. Undistort ---
            cv::Mat color_undist, depth_undist;

            // 1. Color: linear interpolation, smooth image
            cv::remap(color_raw, color_undist, map1, map2, cv::INTER_LINEAR);

            // 2. Depth: nearest neighbor interpolation, NEVER use Linear
            // Prevents non-existent intermediate values at depth edges
            cv::remap(depth_raw, depth_undist, map1, map2, cv::INTER_NEAREST);

            // --- D. Visualization ---
            // Concatenated display: left raw, right undistorted
            cv::Mat depth_raw_vis = colorizeDepth(depth_raw);
            cv::Mat depth_undist_vis = colorizeDepth(depth_undist);

            cv::Mat row_color, row_depth, combined;
            cv::hconcat(color_raw, color_undist, row_color);
            cv::hconcat(depth_raw_vis, depth_undist_vis, row_depth);
            cv::vconcat(row_color, row_depth, combined);

            // Labels
            cv::putText(combined, "Raw Color", cv::Point(20, 30), 0, 0.8, cv::Scalar(0,255,0), 2);
            cv::putText(combined, "Undistorted Color", cv::Point(640+20, 30), 0, 0.8, cv::Scalar(0,255,0), 2);
            cv::putText(combined, "Raw Depth", cv::Point(20, 480+30), 0, 0.8, cv::Scalar(255,255,255), 2);
            cv::putText(combined, "Undistorted Depth (NN)", cv::Point(640+20, 480+30), 0, 0.8, cv::Scalar(255,255,255), 2);

            cv::imshow("RealSense Undistort View", combined);
            cv::waitKey(1);

        } catch (std::exception& e) {
            ROS_ERROR("Exception in callback: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_undistort_node");

    RealsenseProcessor processor;

    ros::spin();
    return 0;
}