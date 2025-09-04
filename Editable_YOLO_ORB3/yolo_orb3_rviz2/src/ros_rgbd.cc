/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM)
        : Node("rgbd_slam_node"), mpSLAM(pSLAM)
    {
        rgb_sub_.subscribe(this, "/camera/color/image_raw_low");
        depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw_low");

        sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&ImageGrabber::GrabRGBD, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,
                  const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
    {
        // RGB 이미지 변환
        cv_bridge::CvImagePtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvCopy(msgRGB, msgRGB->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB exception: %s", e.what());
            return;
        }

        // Depth 이미지 변환
        cv_bridge::CvImagePtr cv_ptrD;
        try
        {
            cv_ptrD = cv_bridge::toCvCopy(msgD, msgD->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge Depth exception: %s", e.what());
            return;
        }

        double timestamp = msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9;
        mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, timestamp);
    }

    ORB_SLAM3::System* mpSLAM;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cerr << "\nUsage: ros2 run yolo_orb3_ros2 rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    auto node = std::make_shared<ImageGrabber>(&SLAM);
    rclcpp::spin(node);

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();
    return 0;
}

