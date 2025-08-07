#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "Usage: ./webcam path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Read settings
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << argv[2] << endl;
        return 1;
    }
    bool bRGB = (int)fsSettings["Camera.RGB"] == 1;

    cv::VideoCapture cap("/dev/video48", cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
    cap.set(cv::CAP_PROP_FPS, 30);
    
    if (!cap.isOpened())
    {
        cerr << "ERROR: Unable to open webcam" << endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cout << endl << "-------" << endl;
    cout << "Start processing webcam stream..." << endl;

    auto initT = chrono::steady_clock::now();

    while (true)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        if (bRGB)
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        if (imageScale != 1.f)
        {
            cv::resize(frame, frame, cv::Size(), imageScale, imageScale);
        }

        auto nowT = chrono::steady_clock::now();
        double timestamp = chrono::duration_cast<chrono::duration<double>>(nowT - initT).count();

        SLAM.TrackMonocular(frame, timestamp);

        // Pangolin 뷰어만 사용하도록 OpenCV UI 제거
        if ((cv::waitKey(1) & 0xFF) == 27) break;
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
