#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iomanip>

#include <sophus/se3.hpp>
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

std::atomic<bool> g_stop_signal(false);                 // Global termination signal flag

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ORB_SLAM3::ViewerAR* pViewer, std::ofstream& trajectory_file)
    : Node("mono_slam_ar_node"), mpSLAM(pSLAM), mpViewer(pViewer), mTrajectoryFile(trajectory_file)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

        mTrajectoryFile << "# Camera Trajectory" << endl;
        mTrajectoryFile << "# timestamp tx ty tz qx qy qz qw" << endl;
    }

private:
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (g_stop_signal) return;                      // Stop processing when a termination signal is received

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image.clone();
        double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        //====================================================//
        //======= Start camera trajectory saving logic =======//
        //====================================================//

        Sophus::SE3f Tcw = mpSLAM->TrackMonocular(frame, tframe);

        if (!Tcw.unit_quaternion().isApprox(Eigen::Quaternionf(0,0,0,0)))
        {
            // 월드 좌표계 기준 카메라 Pose (Twc) 계산 (Sophus 사용)
            Sophus::SE3f Twc = Tcw.inverse();

            // Translation과 Quaternion 추출 (Sophus 사용)
            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            // TUM 형식으로 파일에 저장: timestamp tx ty tz qx qy qz qw
            mTrajectoryFile << fixed << setprecision(7) << tframe << " "
                            << twc.x() << " " << twc.y() << " " << twc.z() << " "
                            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        //====================================================//
        //======== End camera trajectory saving logic ========//
        //====================================================//

        // Sends the latest frame to the viewer.
        if (mpViewer) {
            mpViewer->SetLatestFrame(frame);
        }

        // Passing to YOLO thread
        {
            std::lock_guard<std::mutex> lock(mtx);
            sharedFrame = frame.clone();
            newFrameAvailable = true;
        }
        cv_frame_ready.notify_one();
    }

    ORB_SLAM3::System* mpSLAM;
    ORB_SLAM3::ViewerAR* mpViewer;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::ofstream& mTrajectoryFile;
};

void YoloThread(YoloDetection* yolo, ORB_SLAM3::ViewerAR* viewer)
{
    while (rclcpp::ok() && !g_stop_signal.load()) {     // g_stop_signal.load -> Added termination signal check
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame_ready.wait(lock, [] { return newFrameAvailable || g_stop_signal.load(); });

            if (g_stop_signal.load()) break;

            frame = sharedFrame.clone();
            newFrameAvailable = false;
        }

        if (!frame.empty()) {
            yolo->GetImage(frame);
            if (!yolo->mRGB.empty() && yolo->Detect()) {
                std::vector<ORB_SLAM3::Detection> convertedDetections;
                convertedDetections.reserve(yolo->mvDetections.size());
                for (const auto& det : yolo->mvDetections) {
                    ORB_SLAM3::Detection d;
                    d.x = det.box.x; d.y = det.box.y;
                    d.w = det.box.width; d.h = det.box.height;
                    d.label = det.label; d.confidence = det.confidence;
                    convertedDetections.push_back(d);
                }
                viewer->SetDetections(convertedDetections);
            }
        }

        std::cout << "YOLO thread finished\n" << std::endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3) {
        std::cerr << "\nUsage: ros2 run yolo_orb3_ros2 mono_ar path_to_vocabulary path_to_settings\n";
        return 1;
    }

    // --- Creating and initializing CameraTrajectory.txt ---
    std::ofstream trajectoryFile;
    trajectoryFile.open("CameraTrajectory.txt", std::ios::out | std::ios::trunc);
    if (!trajectoryFile.is_open()) {
        std::cerr << "FATAL: Could not open CameraTrajectory.txt for writing.\n";
        return 1;
    }
    // ---

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);  // ORB default viewer is OFF (to avoid conflicts with Pangolin windows)

    YoloDetection yolo;
    ORB_SLAM3::ViewerAR viewerAR;
    viewerAR.SetSLAM(&SLAM);
    viewerAR.SetStopSignal(&g_stop_signal);             // Pass the address of the termination signal flag to the Viewer

    std::thread yolo_thread(YoloThread, &yolo, &viewerAR);
    std::thread viewer_thread(&ORB_SLAM3::ViewerAR::Run, &viewerAR);

    auto node = std::make_shared<ImageGrabber>(&SLAM, &viewerAR, trajectoryFile);
    rclcpp::spin(node);

    g_stop_signal.store(true);                          
    cv_frame_ready.notify_all();                        

    // Wait until the thread terminates normally
    yolo_thread.join();
    viewer_thread.join();

    SLAM.Shutdown();

    trajectoryFile.close();
    cout << "Trajectory saved to CameraTrajectory.txt" << endl;

    rclcpp::shutdown();

    cout << "Shutdown complete." << endl;
    return 0;
}
