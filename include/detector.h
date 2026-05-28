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
    /** 上一帧 run_yolo 是否为 YOLO 真实检出（非补帧 holdover） */
    bool last_detection_fresh() const { return last_detection_fresh_; }

private:
    std::unique_ptr<IYoloInferBackend> backend_;

    float conf_thres = 0.5f;
    float nms_thres = 0.5f;
    float alpha = 0.77f;
    DetectResult last_res;
    bool has_history = false;
    int lose_cnt = 0;
    bool last_detection_fresh_ = false;
    /** 敌方 YOLO 类别索引：0=blue, 1=red（由 hardware.our_side 推导） */
    int enemy_class_id_ = 0;
};
#endif
