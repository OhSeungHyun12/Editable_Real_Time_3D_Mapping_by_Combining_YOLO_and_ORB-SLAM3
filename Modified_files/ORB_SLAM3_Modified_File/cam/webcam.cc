#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
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

    cv::VideoCapture cap("/dev/video6");  // 또는 숫자로 cap(6)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
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
            int width = frame.cols * imageScale;
            int height = frame.rows * imageScale;
            cv::resize(frame, frame, cv::Size(width, height));
        }

        auto nowT = chrono::steady_clock::now();
        double timestamp = chrono::duration_cast<chrono::duration<double>>(nowT - initT).count();

        SLAM.TrackMonocular(frame, timestamp);

        if (cv::waitKey(1) == 27) break; // ESC to quit
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
