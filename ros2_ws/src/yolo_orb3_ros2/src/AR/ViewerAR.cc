#include "ViewerAR.h"
#include "../../../include/System.h"      
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <algorithm>

namespace ORB_SLAM3 {

void ViewerAR::Run()
{
    if (!mpSystem) {
        std::cerr << "FATAL: SLAM System pointer is null in ViewerAR.\n";
        return;
    }

    // 1) Wait for the first frame
    cv::Mat first;
    while (first.empty()) {
        {
            std::lock_guard<std::mutex> lk(mMutexLatestFrame);
            if (!mLatestFrame.empty()) first = mLatestFrame.clone();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const int w = first.cols, h = first.rows;

    // 2) Pangolin Reset
    pangolin::CreateWindowAndBind("YOLO-ORB-SLAM3: Current Frame", w, h);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    pangolin::DisplayBase().SetBounds(0.0, 1.0, 0.0, 1.0);
    pangolin::GlTexture tex(w, h, GL_RGBA8, false, 0, GL_RGBA, GL_UNSIGNED_BYTE);

    while (!pangolin::ShouldQuit()) {
        // Latest frame snapshot
        cv::Mat im_bgr;
        {
            std::lock_guard<std::mutex> lk(mMutexLatestFrame);
            if (!mLatestFrame.empty()) im_bgr = mLatestFrame.clone();
        }

        // SLAM State/Point Snapshot
        int status = mpSystem->GetTrackingState();
        auto vKeys = mpSystem->GetTrackedKeyPointsUn();
        auto vMPs  = mpSystem->GetTrackedMapPoints();

        // YOLO detection snapshot
        std::vector<Detection> dets;
        {
            std::lock_guard<std::mutex> lk(mMutexDetections);
            dets = mDetections;
        }

        if (!im_bgr.empty()) {
            // Status text
            if (status == 1)      AddTextToImage("SLAM NOT INITIALIZED", im_bgr, 255, 0, 0);
            else if (status == 2) AddTextToImage("SLAM ON",               im_bgr, 0, 255, 0);
            else if (status == 3) AddTextToImage("SLAM LOST",             im_bgr, 255, 0, 0);

            // Tracked points
            if (!vKeys.empty() && !vMPs.empty())
                DrawTrackedPoints(vKeys, vMPs, im_bgr);

            // YOLO object detection box
            if (!dets.empty())
                DrawBoundingBoxes(im_bgr, dets);

            // Render
            cv::Mat rgba;
            cv::cvtColor(im_bgr, rgba, cv::COLOR_BGR2RGBA);
            glClearColor(0.1f,0.1f,0.1f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            tex.Upload(rgba.data, GL_RGBA, GL_UNSIGNED_BYTE);
            tex.RenderToViewportFlipY();
        }

        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "ViewerAR thread finished.\n";
}

void ViewerAR::DrawBoundingBoxes(cv::Mat& im_bgr, const std::vector<Detection>& dets)
{
    const int W = im_bgr.cols, H = im_bgr.rows;
    for (const auto& d : dets) {
        int x = std::clamp(d.x, 0, W-1);
        int y = std::clamp(d.y, 0, H-1);
        int w = std::max(1, std::min(d.w, W - x));
        int h = std::max(1, std::min(d.h, H - y));
        cv::Rect box(x,y,w,h);

        cv::rectangle(im_bgr, box, {0,0,255}, 2);
        int bl=0; auto sz=cv::getTextSize(d.label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &bl);
        int y0 = std::max(0, y - sz.height - bl);
        int y1 = std::max(0, y);
        cv::rectangle(im_bgr, {x,y0}, {x+sz.width, y1}, {0,0,255}, cv::FILLED);
        cv::putText(im_bgr, d.label, {x, y1-bl}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
    }
}

void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint>& vKeys,
                                 const std::vector<MapPoint*>& vMPs,
                                 cv::Mat& im)
{
    const int N = std::min(vKeys.size(), vMPs.size());
    for (int i = 0; i < N; ++i) {
        if (vMPs[i]) {
            cv::circle(im, vKeys[i].pt, 2, {0,255,0}, -1, cv::LINE_AA);
        }
    }
}

void ViewerAR::AddTextToImage(const std::string& s, cv::Mat& im, int r, int g, int b)
{
    int l = 10;
    cv::putText(im, s, {l, im.rows - l}, cv::FONT_HERSHEY_PLAIN, 1.5, {255,255,255}, 2, 8);
    cv::putText(im, s, {l, im.rows - l}, cv::FONT_HERSHEY_PLAIN, 1.5, {r,g,b}, 2, 8);
}

} // namespace ORB_SLAM3
