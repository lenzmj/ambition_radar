#ifndef DETECTOR_H
#define DETECTOR_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "yolo_infer_backend.h"

struct DetectResult {
    cv::Rect2f box;
    float score;
    std::vector<cv::Point2f> corners;
};

class Detector {
public:
    Detector();
    std::vector<DetectResult> run_yolo(cv::Mat& frame);

private:
    std::unique_ptr<IYoloInferBackend> backend_;

    float conf_thres = 0.5f;
    float nms_thres = 0.5f;
    float alpha = 0.77f;
    DetectResult last_res;
    bool has_history = false;
    int lose_cnt = 0;
};
#endif
