#ifndef HEADERS_H
#define HEADERS_H

// C++
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <thread>
#include <string>
#include <unordered_map>

// OpenCV & Bridge
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// TF / Geometry
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Images / PointCloud
#include <sophus/se3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// vision_msgs (2D/3D) 
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

// libtorch (YOLO)
#include <torch/script.h>
#include <torch/torch.h>

// Project local
#include "ViewerAR.h"
#include "YoloDetect.h"
#include <System.h>

// RGB function
inline std::array<uint8_t,3> class_color(int cid) {
    uint32_t v = 0x9e3779b9u ^ (uint32_t)cid * 0x45d9f3bu;
    return {
        uint8_t(64 + ( v        &0xFF)/2),
        uint8_t(64 + ((v >> 8 ) &0xFF)/2),
        uint8_t(64 + ((v >>16 ) &0xFF)/2)
    };
}

inline uint32_t pack_rgb(uint8_t r,uint8_t g,uint8_t b) {
    uint32_t u; 
    uint8_t bgra[4] = {b,g,r,0}; 
    std::memcpy(&u,bgra,4); 
    return u;
}

#endif// HEADERS_H
