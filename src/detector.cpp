#include "detector.h"
#include "yaml.hpp"
#include <iostream>

using namespace cv;
using namespace std;

Detector::Detector() {
    auto& cfg = ConfigManager::getInstance();
    const string backend = cfg.get<string>("hardware.inference_backend", "openvino");
    const string model_path = cfg.get<string>("hardware.model_path", "");
    try {
        backend_ = create_yolo_infer_backend(backend, model_path);
        cout << "[Detector] 推理后端: " << backend << endl;
    } catch (const exception& e) {
        cerr << "[Detector 错误] 初始化失败: " << e.what() << endl;
    }
}

vector<DetectResult> Detector::run_yolo(Mat& frame) {
    auto& cfg = ConfigManager::getInstance();
    float yaml_conf = cfg.get<float>("params.conf_threshold", 0.6f);
    float yaml_alpha = cfg.get<float>("params.det_alpha", 0.1f);

    vector<DetectResult> final_results;
    if (frame.empty() || !backend_) {
        return final_results;
    }

    Mat blob;
    dnn::blobFromImage(frame, blob, 1.0 / 255.0, Size(640, 640), Scalar(), true, false);

    YoloInferOutput yolo_out = backend_->infer(blob);
    if (!yolo_out.data || yolo_out.dimensions <= 0 || yolo_out.rows <= 0) {
        return final_results;
    }

    const float* data = yolo_out.data;
    const int dimensions = yolo_out.dimensions;
    const int rows = yolo_out.rows;

    float scale_x = static_cast<float>(frame.cols) / 640.0f;
    float scale_y = static_cast<float>(frame.rows) / 640.0f;

    float max_score = -1.0f;
    int best_idx = -1;

    for (int i = 0; i < rows; ++i) {
        float score = data[4 * rows + i];
        if (score > yaml_conf && score > max_score) {
            max_score = score;
            best_idx = i;
        }
    }

    if (best_idx != -1) {
        float cx_net = data[0 * rows + best_idx];
        float cy_net = data[1 * rows + best_idx];
        float w_net = data[2 * rows + best_idx];
        float h_net = data[3 * rows + best_idx];

        float angle_rad = data[(dimensions - 1) * rows + best_idx];
        float angle_deg = (angle_rad * 180.0f / CV_PI);

        RotatedRect rrect_net(Point2f(cx_net, cy_net), Size2f(w_net, h_net), angle_deg);
        Point2f pts_net[4];
        rrect_net.points(pts_net);

        DetectResult current;
        current.corners.clear();

        for (int j = 0; j < 4; j++) {
            Point2f pt_original;
            pt_original.x = pts_net[j].x * scale_x;
            pt_original.y = pts_net[j].y * scale_y;
            current.corners.push_back(pt_original);
        }

        current.box = boundingRect(current.corners);

        if (!has_history) {
            last_res = current;
            has_history = true;
        } else {
            for (int j = 0; j < 4; j++) {
                last_res.corners[j].x =
                    yaml_alpha * current.corners[j].x + (1 - yaml_alpha) * last_res.corners[j].x;
                last_res.corners[j].y =
                    yaml_alpha * current.corners[j].y + (1 - yaml_alpha) * last_res.corners[j].y;
            }
            last_res.box = current.box;
        }
        lose_cnt = 0;
        final_results.push_back(last_res);
    } else if (has_history && lose_cnt < 2) {
        lose_cnt++;
        final_results.push_back(last_res);
    } else {
        has_history = false;
    }

    return final_results;
}
