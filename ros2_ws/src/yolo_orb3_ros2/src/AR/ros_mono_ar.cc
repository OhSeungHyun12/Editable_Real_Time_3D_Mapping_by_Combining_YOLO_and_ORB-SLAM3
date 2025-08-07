#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <torch/script.h>
#include <torch/torch.h>
#include "YoloDetect.h"
#include "ViewerAR.h"
#include "../../../include/System.h"

using namespace std;

std::mutex mtx;
std::condition_variable cv_frame_ready;
cv::Mat sharedFrame;
bool newFrameAvailable = false;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM)
    : Node("mono_slam_ar_node"), mpSLAM(pSLAM)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));
    }

private:
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image.clone();
        mpSLAM->TrackMonocular(frame, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

        {
            std::lock_guard<std::mutex> lock(mtx);
            sharedFrame = frame.clone();
            newFrameAvailable = true;
        }
        cv_frame_ready.notify_one();
    }

    ORB_SLAM3::System* mpSLAM;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

// YOLO 스레드
void YoloThread(YoloDetection* yolo, ORB_SLAM3::ViewerAR* viewer)
{
    while (rclcpp::ok())
    {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame_ready.wait(lock, [] { return newFrameAvailable; });

            frame = sharedFrame.clone();
            newFrameAvailable = false;
        }

        if (!frame.empty())
        {
            yolo->GetImage(frame);

            if (!yolo->mRGB.empty() && yolo->Detect())
            {
                std::vector<ORB_SLAM3::Detection> convertedDetections;
                for (const auto& det : yolo->mvDetections)
                {
                    ORB_SLAM3::Detection d;
                    d.x = det.box.x;
                    d.y = det.box.y;
                    d.w = det.box.width;
                    d.h = det.box.height;
                    d.label = det.label;
                    d.confidence = det.confidence;
                    convertedDetections.push_back(d);
                }

                viewer->SetDetections(convertedDetections);
            }
            else
            {
                std::cout << "[YOLO] Detection failed" << std::endl;
            }
        }
        else
        {
            std::cout << "[YOLO] Read RGB failed! Frame empty." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cerr << "\nUsage: ros2 run yolo_orb3_ros2 mono_ar path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    YoloDetection yolo;
    ORB_SLAM3::ViewerAR viewerAR;
    viewerAR.SetSLAM(&SLAM);

    std::thread yoloThread(YoloThread, &yolo, &viewerAR);
    auto node = std::make_shared<ImageGrabber>(&SLAM);
    rclcpp::spin(node);

    yoloThread.join();

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    rclcpp::shutdown();

    return 0;
}
