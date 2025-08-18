#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3 {

class System;
class MapPoint;

struct Detection {
    int x, y, w, h;
    std::string label;
    float confidence;
};

class ViewerAR {
public:
    ViewerAR()
        : mpSystem(nullptr), mFPS(0.f), mT(0.f), fx(0.f), fy(0.f), cx(0.f), cy(0.f) {}

    void SetSLAM(System* pSystem) { mpSystem = pSystem; }
    void SetFPS(float fps) { mFPS = fps; mT = (fps > 0.f ? 1000.f / fps : 0.f); }
    void SetCameraCalibration(float fx_, float fy_, float cx_, float cy_) { fx=fx_; fy=fy_; cx=cx_; cy=cy_; }

    // ROS callbacks pass the latest frame.
    void SetLatestFrame(const cv::Mat& im) {
        std::lock_guard<std::mutex> lk(mMutexLatestFrame);
        mLatestFrame = im.clone();
    }

    // Send YOLO detection result
    void SetDetections(const std::vector<Detection>& dets) {
        std::lock_guard<std::mutex> lk(mMutexDetections);
        mDetections = dets;
    }

    // Pangolin render loop
    void Run();

private:
    static void DrawBoundingBoxes(cv::Mat& im_bgr, const std::vector<Detection>& dets);
    static void DrawTrackedPoints(const std::vector<cv::KeyPoint>& vKeys,
                                  const std::vector<MapPoint*>& vMPs,
                                  cv::Mat& im);
    static void AddTextToImage(const std::string& s, cv::Mat& im, int r, int g, int b);

    System* mpSystem;

    // Latest Frame
    cv::Mat mLatestFrame;
    std::mutex mMutexLatestFrame;

    // Detection results
    std::vector<Detection> mDetections;
    std::mutex mMutexDetections;

    // Render parameters
    float mFPS, mT;
    float fx, fy, cx, cy;
};

} // namespace ORB_SLAM3
#endif // VIEWERAR_H
