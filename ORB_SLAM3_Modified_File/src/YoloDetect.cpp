#include <YoloDetect.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>

YoloDetection::YoloDetection()
    : device(torch::kCPU)
{
    torch::jit::setTensorExprFuserEnabled(false);

    // 1) Save file path

    std::string model_path = "/home/ruherpan/YOLO_ORB_SLAM3/ORB_SLAM3/yolo11n.torchscript.pt";
    std::string names_path = "/home/ruherpan/YOLO_ORB_SLAM3/ORB_SLAM3/coco.names";

    // 2) YOLOv11 model
    mModule = torch::jit::load(model_path);
    if (torch::cuda::is_available()) {
        device = torch::Device(torch::kCUDA);
        std::cout << "[YOLO] Using GPU for inference" << std::endl;
        mModule.to(device);
    } else {
        device = torch::Device(torch::kCPU);
        std::cout << "[YOLO] Using CPU for inference" << std::endl;
    }

    mModule.eval();

    // 3) Load class name
    std::ifstream f(names_path);
    std::string name;
    while (std::getline(f, name)) {
        mClassnames.push_back(name);
    }

    // Dynamic object class
    mvDynamicNames = {"person", "car", "motorbike", "bus", "train", "truck", "boat", "bird", "cat",
                      "dog", "horse", "sheep", "cow", "bear"};

    lastFrameTime = std::chrono::steady_clock::now();
}

YoloDetection::~YoloDetection() {}

bool YoloDetection::Detect()
{  
    mvDetections.clear();
    mvDynamicArea.clear();
    mmDetectMap.clear();

    if (mRGB.empty()) {
        std::cerr << "[YOLO] Read RGB failed! No image received from topic." << std::endl;
        return false;
    }

    // // FPS measurement
    // auto currentTime = std::chrono::steady_clock::now();
    // float fps = 1000.0f / std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFrameTime).count();
    // lastFrameTime = currentTime;
    // std::cout << "[YOLO] Frame received: " << mRGB.cols << "x" << mRGB.rows << " | FPS: " << fps << std::endl;

    // 1) YOLO input image preprocessing (letterboxing)
    const int IN = 640;
    float gain = std::min(IN / (float)mRGB.cols, IN / (float)mRGB.rows);
    int   new_w = (int)std::round(mRGB.cols * gain);
    int   new_h = (int)std::round(mRGB.rows * gain);
    int   pad_left = (IN - new_w) / 2;
    int   pad_top  = (IN - new_h) / 2;

    cv::Mat img(IN, IN, CV_8UC3, cv::Scalar(114,114,114));
    if (new_w > 0 && new_h > 0) {
        cv::Mat resized;
        cv::resize(mRGB, resized, cv::Size(new_w, new_h));
        resized.copyTo(img(cv::Rect(pad_left, pad_top, new_w, new_h)));
    }
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    

    torch::Tensor imgTensor = torch::from_blob(img.data, {1, img.rows, img.cols, 3}, torch::kByte);
    imgTensor = imgTensor.permute({0, 3, 1, 2}).toType(torch::kFloat) / 255.0;
    imgTensor = imgTensor.to(device);

    // 2) YOLO inference
    torch::IValue output = mModule.forward({imgTensor});
    torch::Tensor preds;

    if (output.isTensor())
        preds = output.toTensor();
    else if (output.isTuple())
        preds = output.toTuple()->elements()[0].toTensor();

    preds = preds.to(torch::kCPU);
    // std::cout << "[YOLO] Output shape: " << preds.sizes() << std::endl;
   
    preds = preds.squeeze(0).transpose(0, 1);                               // YOLOv11 output: [1, 84, 8400] → [8400, 84]

    // 3) NMS
    auto dets = YoloDetection::non_max_suppression(preds, 0.6, 0.5);        // conf=0.6, IoU=0.5

    if (!dets.empty()) {
        for (int i = 0; i < dets[0].sizes()[0]; ++i) {
            // Inverse compensation (using the same integer pad/real gain as preprocessing)
            float left   = (dets[0][i][0].item<float>() - pad_left) / gain;
            float top    = (dets[0][i][1].item<float>() - pad_top ) / gain;
            float right  = (dets[0][i][2].item<float>() - pad_left) / gain;
            float bottom = (dets[0][i][3].item<float>() - pad_top ) / gain;

            // Extract confidence/classID
            float confidence = dets[0][i][4].item<float>();
            int   classID    = dets[0][i][5].item<int>();
            
            int x = std::clamp((int)std::round(left),  0, mRGB.cols - 1);
            int y = std::clamp((int)std::round(top),   0, mRGB.rows - 1);
            int w = std::clamp((int)std::round(right  - left), 1, mRGB.cols - x);
            int h = std::clamp((int)std::round(bottom - top ), 1, mRGB.rows - y);
            cv::Rect2i rect(x, y, w, h);

            if (classID >= 0 && classID < (int)mClassnames.size()) {
                mmDetectMap[mClassnames[classID]].push_back(rect);
                if (std::count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
                mvDynamicArea.push_back(rect);
                
                DetectionResult dr;
                dr.box = rect;
                dr.label = mClassnames[classID];
                dr.confidence = confidence;
                mvDetections.push_back(dr);
            }
        }

        std::cout << "[YOLO] Detection success: " << dets[0].sizes()[0]
                  << " objects detected | mvDetections=" << mvDetections.size() << std::endl;
    } else {
        std::cout << "[YOLO] No objects detected." << std::endl;
    }

    return !mvDetections.empty();
}

std::vector<torch::Tensor> YoloDetection::non_max_suppression(torch::Tensor preds, float score_thresh, float iou_thresh)
{
    std::vector<torch::Tensor> output;    
    torch::Tensor pred = preds;     // [8400, 84]

    const bool has_obj = (pred.size(1) == 85);
    
    torch::Tensor class_conf, max_conf, max_cls, scores;
    if (has_obj) {
        class_conf = pred.slice(1, 5, pred.size(1)).sigmoid();
        std::tie(max_conf, max_cls) = torch::max(class_conf, 1);
        torch::Tensor obj_conf = pred.select(1, 4).sigmoid();
        scores = obj_conf * max_conf;
    } else {
        class_conf = pred.slice(1, 4, pred.size(1)).sigmoid();
        std::tie(max_conf, max_cls) = torch::max(class_conf, 1);
        scores = max_conf;
    }

    std::cout << "[YOLO] Candidates before threshold: " << scores.size(0) << std::endl;
    std::cout << "[YOLO] Max score: " << scores.max().item<float>() << std::endl;


    torch::Tensor mask = scores > score_thresh;
    if (mask.sum().item<int>() == 0) {
        std::cout << "[YOLO] No detections above threshold." << std::endl;
        return output;
    }

    auto selected_idx = torch::nonzero(mask).squeeze();
    pred = pred.index_select(0, selected_idx);
    max_cls = max_cls.index_select(0, selected_idx);
    scores  = scores.index_select(0, selected_idx);

    std::cout << "[YOLO] After score threshold: " << pred.sizes()[0] << " candidates" << std::endl;

    if (pred.sizes()[0] == 0) return output;

    pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
    pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
    pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
    pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);

    pred = torch::cat({pred.slice(1, 0, 4), scores.unsqueeze(1), max_cls.unsqueeze(1)}, 1);

    torch::Tensor areas = (pred.select(1, 2) - pred.select(1, 0)) *
                          (pred.select(1, 3) - pred.select(1, 1));

    auto [_, idxs] = torch::sort(pred.select(1, 4), 0, true);

    std::vector<int64_t> keep;
    while (idxs.size(0) > 0) {
        int64_t i = idxs[0].item<int64_t>();
        keep.push_back(i);

        if (idxs.size(0) == 1) break;

        torch::Tensor rest = idxs.slice(0, 1, idxs.size(0));
        torch::Tensor lefts   = torch::max(pred[i][0], pred.index_select(0, rest).select(1, 0));
        torch::Tensor tops    = torch::max(pred[i][1], pred.index_select(0, rest).select(1, 1));
        torch::Tensor rights  = torch::min(pred[i][2], pred.index_select(0, rest).select(1, 2));
        torch::Tensor bottoms = torch::min(pred[i][3], pred.index_select(0, rest).select(1, 3));

        torch::Tensor widths  = torch::clamp(rights - lefts, 0);
        torch::Tensor heights = torch::clamp(bottoms - tops, 0);
        torch::Tensor overlaps = widths * heights;
        torch::Tensor ious = overlaps / (areas[i] + areas.index_select(0, rest) - overlaps);

        torch::Tensor mask_iou = ious <= iou_thresh;
        idxs = rest.index_select(0, torch::nonzero(mask_iou).squeeze());
    }

    torch::Tensor keep_tensor = torch::from_blob(keep.data(), {(long)keep.size()}, torch::kLong).clone();
    output.push_back(pred.index_select(0, keep_tensor));
    std::cout << "[YOLO] After NMS: " << output[0].sizes()[0] << " final boxes" << std::endl;
    return output;
}

void YoloDetection::GetImage(cv::Mat &im)
{
    if (im.empty()) {
        std::cerr << "[YOLO] GetImage received empty frame!" << std::endl;
        return;
    }
    mRGB = im.clone();
}

void YoloDetection::ClearImage()
{
    mRGB.release();
}

void YoloDetection::ClearArea()
{
    mvPersonArea.clear();
}
