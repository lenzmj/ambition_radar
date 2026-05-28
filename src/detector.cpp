#include "detector.h"
#include "yaml.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>

using namespace cv;
using namespace std;

namespace {

string to_lower_ascii(string s) {
    transform(s.begin(), s.end(), s.begin(),
              [](unsigned char c) { return static_cast<char>(tolower(c)); });
    return s;
}

/** 数据集 0=blue, 1=red；敌方类别与己方相反 */
int enemy_class_from_our_side(const string& our_side) {
    const string side = to_lower_ascii(our_side);
    if (side == "blue") {
        return 1;
    }
    if (side != "red") {
        cerr << "[Detector 警告] hardware.our_side 应为 red 或 blue，当前为 \"" << our_side
             << "\"，按 red（敌方 blue/0）处理\n";
    }
    return 0;
}

string resolve_model_path(const string& our_side) {
    auto& cfg = ConfigManager::getInstance();
    const string override_path = cfg.get<string>("hardware.model_path", "");
    if (!override_path.empty()) {
        return override_path;
    }

    const string model_dir = cfg.get<string>("hardware.model_dir", "");
    const string side = to_lower_ascii(our_side);
    const string filename = (side == "blue")
                                ? cfg.get<string>("hardware.model_blue", "red.engine")
                                : cfg.get<string>("hardware.model_red", "blue_zmj.engine");
    if (model_dir.empty()) {
        return filename;
    }
    return (std::filesystem::path(model_dir) / filename).string();
}

}  // namespace

Detector::Detector() {
    auto& cfg = ConfigManager::getInstance();
    const string backend = cfg.get<string>("hardware.inference_backend", "openvino");
    const string our_side = cfg.get<string>("hardware.our_side", "red");
    const string model_path = resolve_model_path(our_side);
    enemy_class_id_ = enemy_class_from_our_side(our_side);
    try {
        backend_ = create_yolo_infer_backend(backend, model_path);
        cout << "[Detector] 推理后端: " << backend << endl;
        cout << "[Detector] 我方: " << our_side << "，模型: " << model_path << "，敌方类别索引: "
             << enemy_class_id_ << " (0=blue, 1=red)" << endl;
    } catch (const exception& e) {
        cerr << "[Detector 错误] 初始化失败: " << e.what() << endl;
    }
}

vector<DetectResult> Detector::run_yolo(Mat& frame) {
    auto& cfg = ConfigManager::getInstance();
    float yaml_conf = cfg.get<float>("params.conf_threshold", 0.6f);
    float yaml_alpha = cfg.get<float>("params.det_alpha", 0.1f);

    last_detection_fresh_ = false;
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

    const int score_channel = 4 + enemy_class_id_;
    if (score_channel >= dimensions - 1) {
        cerr << "[Detector 错误] 输出维度 " << dimensions << " 不足以读取敌方类别 " << enemy_class_id_
             << "，请检查 hardware.our_side 与模型类别数\n";
        return final_results;
    }

    float max_score = -1.0f;
    int best_idx = -1;

    for (int i = 0; i < rows; ++i) {
        float score = data[score_channel * rows + i];
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
        current.score = max_score;

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
            last_res.score = max_score;
        }
        lose_cnt = 0;
        last_detection_fresh_ = true;
        final_results.push_back(last_res);
    } else if (has_history && lose_cnt < 2) {
        lose_cnt++;
        final_results.push_back(last_res);
    } else {
        has_history = false;
    }

    return final_results;
}
