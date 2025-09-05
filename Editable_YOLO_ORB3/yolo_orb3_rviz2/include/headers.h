#ifndef HEADERS_H
#define HEADERS_H

// C++
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <thread>

// OpenCV & Bridge
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// TF / Geometry
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Images / PointCloud
#include <sophus/se3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// vision_msgs (2D/3D) 
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

// libtorch (YOLO)
#include <torch/script.h>
#include <torch/torch.h>

// Project local
#include "ViewerAR.h"
#include "YoloDetect.h"
#include <System.h>

#endif// HEADERS_H
