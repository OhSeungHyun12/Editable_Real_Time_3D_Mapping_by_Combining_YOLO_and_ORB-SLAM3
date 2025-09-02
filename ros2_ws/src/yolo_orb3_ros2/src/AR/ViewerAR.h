#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
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
        : mpSystem(nullptr), mpStopSignal(nullptr) {}

    void SetSLAM(System* pSystem) { mpSystem = pSystem; }
    void SetStopSignal(std::atomic<bool>* stop_signal) { mpStopSignal = stop_signal; }      // Function that sets the termination signal flag

    void SetLatestFrame(const cv::Mat& im) {
        std::lock_guard<std::mutex> lk(mMutexLatestFrame);
        mLatestFrame = im.clone();
    }

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

    std::atomic<bool>* mpStopSignal;            // Pointer to the termination signal

    cv::Mat mLatestFrame;
    std::mutex mMutexLatestFrame;

    std::vector<Detection> mDetections;
    std::mutex mMutexDetections;
};

} // namespace ORB_SLAM3
#endif // VIEWERAR_H
