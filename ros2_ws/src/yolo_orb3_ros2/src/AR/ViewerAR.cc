#include "ViewerAR.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3
{

ViewerAR::ViewerAR()
    : mFPS(30.0f), mT(33.3f), fx(0), fy(0), cx(0), cy(0), mStatus(0)
{
}

// 이미지와 포즈 설정
void ViewerAR::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                            const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint *> &vMPs)
{
    std::lock_guard<std::mutex> lock(mMutexPoseImage);
    im.copyTo(mImage);
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;
}

void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                            std::vector<cv::KeyPoint> &vKeys, std::vector<MapPoint *> &vMPs)
{
    std::lock_guard<std::mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
}

// YOLO 감지 결과 그리기
void DrawBoundingBoxes(cv::Mat &im, const std::vector<Detection> &detections)
{
    for (const auto &det : detections)
    {
        cv::Rect box(det.x, det.y, det.w, det.h);
        cv::rectangle(im, box, cv::Scalar(0, 255, 0), 2);

        std::string label_text = det.label + " " + std::to_string(det.confidence).substr(0, 4);
        cv::putText(im, label_text, cv::Point(det.x, det.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }
}

// Pangolin을 이용한 AR 뷰어 실행
void ViewerAR::Run()
{
    pangolin::CreateWindowAndBind("ORB-SLAM3 AR Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, fx, fy, cx, cy, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::GlTexture imageTexture(1024, 768, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        cv::Mat im;
        cv::Mat Tcw;
        int status;
        std::vector<cv::KeyPoint> vKeys;
        std::vector<MapPoint *> vMPs;

        {
            std::lock_guard<std::mutex> lock(mMutexPoseImage);
            if (mImage.empty())
                continue;
            im = mImage.clone();
            Tcw = mTcw.clone();
            status = mStatus;
            vKeys = mvKeys;
            vMPs = mvMPs;
        }

        // YOLO 감지 결과 가져오기
        std::vector<Detection> detections;
        {
            std::lock_guard<std::mutex> detLock(mMutexDetections);
            detections = mDetections;
        }

        // YOLO 결과 그리기
        DrawBoundingBoxes(im, detections);

        // 이미지 텍스처로 Pangolin 뷰어에 표시
        imageTexture.Upload(im.data, GL_BGR, GL_UNSIGNED_BYTE);
        d_cam.Activate(s_cam);
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();

        // OpenCV 디버그 창에도 출력
        cv::imshow("YOLO + ORB-SLAM3", im);
        cv::waitKey(1);
    }
}

} // namespace ORB_SLAM3
