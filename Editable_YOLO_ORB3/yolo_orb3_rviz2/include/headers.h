#ifndef HEADERS_H
#define HEADERS_H

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <fstream>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <thread>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>

#include <torch/script.h>
#include <torch/torch.h>
#include "ViewerAR.h"
#include "YoloDetect.h"
#include "../../../include/System.h"

// Rviz2 header //
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#endif// HEADERS_H