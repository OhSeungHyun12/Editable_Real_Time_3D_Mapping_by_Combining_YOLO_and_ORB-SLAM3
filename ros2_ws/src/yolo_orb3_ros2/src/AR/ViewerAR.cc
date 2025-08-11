#include "ViewerAR.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

namespace ORB_SLAM3
{

    void ViewerAR::Run()
    {
        // 1) 첫 유효 프레임 대기(빈 업로드 방지)
        cv::Mat first;
        while (first.empty()) {
            {
                std::lock_guard<std::mutex> lk(mMutexLatestFrame);
                if (!mLatestFrame.empty()) first = mLatestFrame.clone();
            }
            cv::waitKey(1);
        }

        const int w = first.cols;
        const int h = first.rows;

        // 2) Pangolin 최소 텍스처 렌더
        pangolin::CreateWindowAndBind("YOLO-ORB: Current Frame", w, h);
        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);               // 2D 텍스처는 depth 불필요
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // 행 정렬 이슈 방지

        pangolin::DisplayBase().SetBounds(0.0,1.0,0.0,1.0);
        pangolin::GlTexture tex(w, h, GL_RGBA8, false, 0, GL_RGBA, GL_UNSIGNED_BYTE);

        while (!pangolin::ShouldQuit()) {
            // 최신 스냅샷
            cv::Mat im_bgr;
            {
                std::lock_guard<std::mutex> lk(mMutexLatestFrame);
                if (!mLatestFrame.empty()) im_bgr = mLatestFrame.clone();
            }

            std::vector<Detection> dets_snapshot;
            {
                std::lock_guard<std::mutex> lk(mMutexDetections);
                dets_snapshot = mDetections;
            }

            if (!im_bgr.empty()) {
                // (디버그) det=N + 테스트 박스: 파이프 확인
                const std::string dbg = "det=" + std::to_string(dets_snapshot.size());
                cv::putText(im_bgr, dbg, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,0}, 2);
                cv::rectangle(im_bgr, cv::Rect(10, 50, 100, 60), {255,255,255}, 2);

                // YOLO 박스 그리기(좌표 클램프)
                if (!dets_snapshot.empty())
                    DrawBoundingBoxes(im_bgr, dets_snapshot);
                
                std::cout << "[VIEWER] draw dets=" << dets_snapshot.size() << std::endl;

                // BGR -> RGBA (GL_RGBA8 텍스처 업로드)
                cv::Mat im_rgba;
                cv::cvtColor(im_bgr, im_rgba, cv::COLOR_BGR2RGBA);
                if (!im_rgba.isContinuous()) im_rgba = im_rgba.clone();

                glClearColor(0.1f,0.1f,0.1f,1.0f);
                glClear(GL_COLOR_BUFFER_BIT);
                tex.Upload(im_rgba.data, GL_RGBA, GL_UNSIGNED_BYTE);
                tex.RenderToViewportFlipY();
            }

            pangolin::FinishFrame();
            cv::waitKey(1); // 이벤트 펌프 보조
        }

        std::cout << "ViewerAR thread finished.\n";
    }

    void DrawBoundingBoxes(cv::Mat& im_bgr, const std::vector<Detection>& dets)
    {
        const int W = im_bgr.cols, H = im_bgr.rows;

        for (const auto& d : dets) {
            // 좌표 클램프(프레임 바깥/음수 보호)
            int x = std::max(0, std::min(d.x, W - 1));
            int y = std::max(0, std::min(d.y, H - 1));
            int w = std::max(1, std::min(d.w, W - x));
            int h = std::max(1, std::min(d.h, H - y));

            cv::Rect box(x, y, w, h);
            cv::rectangle(im_bgr, box, {0,0,255}, 3);

            int bl = 0;
            auto sz = cv::getTextSize(d.label, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &bl);
            int y0 = std::max(0, y - sz.height - bl);
            int y1 = std::max(0, y);
            cv::rectangle(im_bgr, {x, y0}, {x+sz.width, y1}, {0,0,255}, cv::FILLED);
            cv::putText(im_bgr, d.label, {x, y1 - bl}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2);
        }
    }

} // namespace ORB_SLAM3
