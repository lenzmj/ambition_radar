#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <openvino/openvino.hpp>

struct DetectResult {
    cv::Rect2f box;      
    float score;
    std::vector<cv::Point2f> corners; 
};

class Detector {
public:
    Detector(const std::string& model_path);
    std::vector<DetectResult> run_yolo(cv::Mat& frame);

private:
    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;

    float conf_thres = 0.5f;
    float nms_thres = 0.5f;
    float alpha = 0.77f; // 滤波系数
    DetectResult last_res;      
    bool has_history = false;   
    int lose_cnt = 0;           
};
#endif