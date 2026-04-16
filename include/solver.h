#ifndef SOLVER_H
#define SOLVER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "detector.h"

struct GimbalCmd {
    bool is_locked;
    float target_yaw;
    float target_pitch;
    float p_world_x;
    float p_world_y;
    float p_world_z;
};

class Solver {
public:
    Solver();
    GimbalCmd solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll = 0.0f);
    void reset_filter(); // 目标长时间消失后重置滤波器

private:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Point3f> object_3d_points;
    Eigen::Vector3f cam_offset;
    Eigen::Vector3f ray_offset;

    // --- 滤波相关 ---
    float last_yaw;
    float last_pitch;
    bool is_first_run;
    const float alpha = 0.27f; // 调节此值：0.1很稳但慢，0.5较抖但快
};

#endif