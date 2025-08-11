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

// === (선택) YOLO 박스 좌표 스케일 관련 옵션 ===
static constexpr bool kDetIsNormalized   = false; // det.box가 [0,1] 정규화라면 true
static constexpr bool kDetIsModelInput   = false;  // det.box가 모델입력(예: 640x640) 좌표면 true
static constexpr int  kYoloInputSize     = 640;   // YOLO 입력 한 변(정사각형 가정)

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ORB_SLAM3::ViewerAR* pViewer)
    : Node("mono_slam_ar_node"), mpSLAM(pSLAM), mpViewer(pViewer) 
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));
    }

private:
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image.clone();
        double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // 1) SLAM에 전달
        mpSLAM->TrackMonocular(frame, tframe);

        // 2) 뷰어에 최신 프레임 갱신
        if (mpViewer) {
            mpViewer->SetLatestFrame(frame);
            RCLCPP_INFO(this->get_logger(), "[VIEWER] SetLatestFrame: %dx%d", frame.cols, frame.rows);
        }

        // 3) YOLO 스레드로 전달
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
};

// YOLO 스레드
void YoloThread(YoloDetection* yolo, ORB_SLAM3::ViewerAR* viewer)
{
    while (rclcpp::ok()) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mtx);
            cv_frame_ready.wait(lock, [] { return newFrameAvailable; });
            frame = sharedFrame.clone();
            newFrameAvailable = false;
        }

        if (!frame.empty()) {
            const int W = frame.cols, H = frame.rows;

            yolo->GetImage(frame);
            if (!yolo->mRGB.empty() && yolo->Detect()) {
                std::vector<ORB_SLAM3::Detection> convertedDetections;
                convertedDetections.reserve(yolo->mvDetections.size());

                for (const auto& det : yolo->mvDetections) {
                    ORB_SLAM3::Detection d;

                    if (kDetIsNormalized) {
                        // 정규화 [0,1] → 픽셀
                        d.x = static_cast<int>(det.box.x * W);
                        d.y = static_cast<int>(det.box.y * H);
                        d.w = static_cast<int>(det.box.width  * W);
                        d.h = static_cast<int>(det.box.height * H);
                    } else if (kDetIsModelInput) {
                        // 모델 입력(예: 640x640) 좌표 → 원본(WxH)로 역매핑 (letterbox 가정)
                        float scale = std::min(1.f * kYoloInputSize / W, 1.f * kYoloInputSize / H);
                        int pad_x = static_cast<int>((kYoloInputSize - W * scale) / 2);
                        int pad_y = static_cast<int>((kYoloInputSize - H * scale) / 2);

                        float cx = (det.box.x - pad_x) / scale;
                        float cy = (det.box.y - pad_y) / scale;
                        float cw = det.box.width  / scale;
                        float ch = det.box.height / scale;

                        d.x = static_cast<int>(cx);
                        d.y = static_cast<int>(cy);
                        d.w = static_cast<int>(cw);
                        d.h = static_cast<int>(ch);
                    } else {
                        // 이미 원본 프레임 기준 픽셀 좌표라고 가정
                        d.x = det.box.x; d.y = det.box.y;
                        d.w = det.box.width; d.h = det.box.height;
                    }

                    d.label = det.label;
                    d.confidence = det.confidence;
                    convertedDetections.push_back(d);
                }

                std::cout << "[YOLO->VIEWER] mvDetections=" << yolo->mvDetections.size()
                          << "  converted=" << convertedDetections.size() << std::endl;

                viewer->SetDetections(convertedDetections);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3) {
        std::cerr << "\nUsage: ros2 run yolo_orb3_ros2 mono_ar path_to_vocabulary path_to_settings\n";
        return 1;
    }

    // ORB_SLAM3 Viewer OFF(중요): 우리 Pangolin 창과 충돌 방지
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);

    YoloDetection yolo;
    ORB_SLAM3::ViewerAR viewerAR;
    viewerAR.SetSLAM(&SLAM);
    viewerAR.SetFPS(30.f); // 필요시 YAML fps로 교체
    // viewerAR.SetCameraCalibration(fx, fy, cx, cy); // 원하면 YAML에서 읽어 설정

    std::thread yolo_thread(YoloThread, &yolo, &viewerAR);
    std::thread viewer_thread(&ORB_SLAM3::ViewerAR::Run, &viewerAR);

    auto node = std::make_shared<ImageGrabber>(&SLAM, &viewerAR);
    rclcpp::spin(node);

    // 종료 순서(간단화)
    yolo_thread.join();
    viewer_thread.join();

    SLAM.Shutdown();
    // 임시로 저장 비활성(종료 타이밍 segfault 완화)
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    rclcpp::shutdown();

    return 0;
}
