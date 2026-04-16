#include "detector.h"
#include "yaml.hpp"
#include <iostream>

using namespace cv;
using namespace std;
using namespace ov;

Detector::Detector(const string& model_path) {
    try {
        shared_ptr<Model> model = core.read_model(model_path);
        compiled_model = core.compile_model(model, "CPU");
        infer_request = compiled_model.create_infer_request();
        cout << "[Detector] 模型加载成功: " << model_path << endl;
    } catch (const exception& e) {
        cerr << "[Detector 错误] 模型加载失败: " << e.what() << endl;
    }
}

vector<DetectResult> Detector::run_yolo(Mat& frame) {
    auto& cfg = ConfigManager::getInstance();
    // 实时读取 YAML 中的配置
  // 必须对应 YAML 里的 params 节点
    float yaml_conf = cfg.get<float>("params.conf_threshold", 0.6f);
    float yaml_alpha = cfg.get<float>("params.det_alpha", 0.1f); 

    vector<DetectResult> final_results;
    if (frame.empty()) return final_results;

    Mat blob;
    dnn::blobFromImage(frame, blob, 1.0/255.0, Size(640, 640), Scalar(), true, false);

    auto input_port = compiled_model.input(0);
    Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.data);
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer(); 

    auto output_tensor = infer_request.get_output_tensor(0);
    auto shape = output_tensor.get_shape();
    float* data = output_tensor.data<float>();
    
    int dimensions = shape[1]; 
    int rows = shape[2];       
    float scale_x = (float)frame.cols / 640.0f;
    float scale_y = (float)frame.rows / 640.0f;

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
        float cx = data[0 * rows + best_idx] * scale_x;
        float cy = data[1 * rows + best_idx] * scale_y;
        float h  = data[2 * rows + best_idx] * scale_x;
        float w  = data[3 * rows + best_idx] * scale_y;

        float angle_rad = data[(dimensions - 1) * rows + best_idx];
        float angle_deg = (angle_rad * 180.0f / CV_PI) - 45.0f; 

        RotatedRect rrect(Point2f(cx, cy), Size2f(w, h), angle_deg);
        Point2f pts[4];
        rrect.points(pts); 

        DetectResult current;
        current.corners.clear();
        for(int j=0; j<4; j++) current.corners.push_back(pts[j]);
        current.box = rrect.boundingRect();

        if (!has_history) {
            last_res = current;
            has_history = true;
        } else {

            for(int j=0; j<4; j++) {
                last_res.corners[j].x = yaml_alpha * current.corners[j].x + (1 - yaml_alpha) * last_res.corners[j].x;
                last_res.corners[j].y = yaml_alpha * current.corners[j].y + (1 - yaml_alpha) * last_res.corners[j].y;
            }
            last_res.box = current.box;
        }
        lose_cnt = 0;
        final_results.push_back(last_res);
    } 
    else if (has_history && lose_cnt < 2) { 
        lose_cnt++;
        final_results.push_back(last_res);
    } else {
        has_history = false;
    }
    
    return final_results;
}
