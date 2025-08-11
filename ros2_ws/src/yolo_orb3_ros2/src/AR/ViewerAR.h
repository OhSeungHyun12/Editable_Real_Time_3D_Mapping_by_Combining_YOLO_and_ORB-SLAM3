#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <mutex>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

namespace ORB_SLAM3
{
    class System;    // no core edits

    struct Detection 
    {
        int x, y, w, h;
        std::string label;
        float confidence;
    };

    class ViewerAR
    {
        public:
        ViewerAR()
            : mpSystem(nullptr), mFPS(0.f), mT(0.f), fx(0.f), fy(0.f), cx(0.f), cy(0.f) {}
            
        void SetSLAM(System* pSystem) { mpSystem = pSystem; }

        void SetFPS(float fps) 
        {
            mFPS = fps; mT = (fps > 0.f ? 1000.f / fps : 0.f); 
        }

        void SetCameraCalibration(float fx_, float fy_, float cx_, float cy_) 
        {
            fx = fx_; fy = fy_; cx = cx_; cy = cy_;
        }

        // 최신 프레임 전달(ROS 콜백)
        void SetLatestFrame(const cv::Mat& im) 
        {
            std::lock_guard<std::mutex> lk(mMutexLatestFrame);
            mLatestFrame = im.clone();
        }

        // YOLO 감지 결과 전달(디버깅 코드 포함)
        void SetDetections(const std::vector<Detection>& dets) 
        {
            std::lock_guard<std::mutex> lk(mMutexDetections);
            mDetections = dets;
            std::cout << "[VIEWER] SetDetections size=" << mDetections.size() << std::endl;
        }

        // Pangolin 렌더 루프
        void Run();

        private:
            System* mpSystem;

            // 최신 프레임 & 뮤텍스
            cv::Mat mLatestFrame;
            std::mutex mMutexLatestFrame;

            // 감지 결과 & 뮤텍스
            std::vector<Detection> mDetections;
            std::mutex mMutexDetections;

            // 렌더 페이싱/내부파라미터(옵션)
            float mFPS;   // 입력 FPS
            float mT;     // 1 프레임 시간(ms)
            float fx, fy, cx, cy;
    };

    // 네임스페이스 내부 free 함수
    void DrawBoundingBoxes(cv::Mat& im_bgr, const std::vector<Detection>& dets);

} // namespace ORB_SLAM3
#endif // VIEWERAR_H
