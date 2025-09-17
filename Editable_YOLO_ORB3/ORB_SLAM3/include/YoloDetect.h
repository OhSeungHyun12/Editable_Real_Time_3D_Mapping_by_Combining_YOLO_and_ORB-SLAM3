#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>
#include <torch/csrc/jit/passes/tensorexpr_fuser.h>
#include <chrono> 
#include <cstdint>

struct DetectionResult
{
    cv::Rect2i box;
    std::string label;
    float confidence;
    int class_id;
};

class YoloDetection
{
public:
    YoloDetection();
    ~YoloDetection();

    bool Detect();
    void GetImage(const cv::Mat& im);
    uint32_t GetColorForClass(int class_id);

    // member variables
    const std::vector<DetectionResult>& GetDetections() const {
        return mvDetections;
    }
    const std::vector<cv::Rect>& GetDynamicArea() const {
        return mvDynamicArea;
    }
    const std::map<std::string, std::vector<cv::Rect>>& GetDetectMap() const {
        return mmDetectMap;
    }
    const std::vector<std::string>& GetClassNames() const {
        return mClassnames;
    }

private:
    // member variables
    torch::jit::script::Module mModule;
    torch::Device device;
    cv::Mat mRGB;
    
    std::vector<std::string> mClassnames;                           // List of class names
    std::vector<std::string> mvDynamicNames;                        // Class name to be classified as a dynamic object
    
    std::vector<DetectionResult> mvDetections;                      // Final detection results
    std::vector<cv::Rect> mvDynamicArea;                            // Dynamic object area
    std::map<std::string, std::vector<cv::Rect>> mmDetectMap;       // Detection area by class name
    // std::vector<cv::Rect> mvPersonArea;                            Original code contents: 'person' object area (use if needed)
    
    std::chrono::steady_clock::time_point lastFrameTime;            // FPS

    // Color Palette
    std::vector<uint32_t> mColorPalette;

    // NMS
    std::vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh, float iou_thresh);

    // R,G,B -> uint32_t
    static inline uint32_t rgb2uint32(int r, int g, int b) {
        return (static_cast<uint32_t>(b)) | (static_cast<uint32_t>(g) << 8) | (static_cast<uint32_t>(r) << 16);
    }
};

#endif //YOLO_DETECT_H
