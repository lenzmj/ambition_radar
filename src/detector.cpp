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
    // 读取 YAML 中的配置
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
       
        // 获取网络在 640x640 尺度下的原始输出
        float cx_net = data[0 * rows + best_idx];
        float cy_net = data[1 * rows + best_idx];
        float w_net  = data[2 * rows + best_idx]; 
        float h_net  = data[3 * rows + best_idx]; 

        float angle_rad = data[(dimensions - 1) * rows + best_idx];
        float angle_deg = (angle_rad * 180.0f / CV_PI); 

        // 在 640x640 的尺度下构建 RotatedRect，并提取 4 个角点
        RotatedRect rrect_net(Point2f(cx_net, cy_net), Size2f(w_net, h_net), angle_deg);
        Point2f pts_net[4];
        rrect_net.points(pts_net); 

        DetectResult current;
        current.corners.clear();
        
        // 对提取出来的 4 个角点分别进行 X 和 Y 的映射缩放
        for(int j = 0; j < 4; j++) {
            Point2f pt_original;
            pt_original.x = pts_net[j].x * scale_x;
            pt_original.y = pts_net[j].y * scale_y;
            current.corners.push_back(pt_original);
        }
        
        //  使用映射后的角点重新计算正交包围框 (用于后续的 box 绘制和判断)
        current.box = boundingRect(current.corners);

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
