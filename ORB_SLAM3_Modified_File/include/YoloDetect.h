#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>
#include <torch/csrc/jit/passes/tensorexpr_fuser.h>
#include <chrono>  // 추가

using namespace std;

class YoloDetection
{
public:
    YoloDetection();
    ~YoloDetection();

    void GetImage(cv::Mat& RGB);
    void ClearImage();
    bool Detect();
    void ClearArea();
    vector<cv::Rect2i> mvPersonArea = {};
    vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5);

    struct DetectionResult
    {
        cv::Rect2i box;
        std::string label;
        float confidence;
    };

    vector<DetectionResult> mvDetections;

public:
    cv::Mat mRGB;
    torch::jit::script::Module mModule;
    std::vector<std::string> mClassnames;

    vector<string> mvDynamicNames;
    vector<cv::Rect2i> mvDynamicArea;
    map<string, vector<cv::Rect2i>> mmDetectMap;

    torch::Device device;

private:
    std::chrono::steady_clock::time_point lastFrameTime;  // FPS 계산용 변수 추가
};

#endif //YOLO_DETECT_H
