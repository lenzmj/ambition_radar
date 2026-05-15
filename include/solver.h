#ifndef SOLVER_H
#define SOLVER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "detector.h"

struct GimbalCmd {
    bool is_locked;
    float target_yaw;
    float target_pitch;
    float target_roll;
    float p_world_x;
    float p_world_y;
    float p_world_z;
    /** solvePnP 平移 tvec（OpenCV 相机系），供激光十字与目标 3D 对齐投影 */
    float pnp_tx;
    float pnp_ty;
    float pnp_tz;
};

class Solver {
public:

    Solver();
    GimbalCmd solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll = 0.0f);
    void reset_filter(); // 目标长时间消失后重置滤波器

    cv::Mat camera_matrix;
    /** 与 camera_matrix 同源，供 overlay 与 solvePnP 一致的畸变投影 */
    cv::Mat dist_coeffs;
    Eigen::Vector3f cam_offset;
    Eigen::Vector3f ray_offset;
    /** 相机系 -> 激光器系：v_ray = R_cam_to_ray * v_cam（列向量）。由 offset.rpy_cam_to_ray [deg] 构造。 */
    Eigen::Matrix3f R_cam_to_ray;

private:

    std::vector<cv::Point3f> object_3d_points;  

    // --- 滤波相关 ---
    float last_yaw;
    float last_pitch;
    bool is_first_run;
    const float alpha = 0.27f; // 调节此值：0.1很稳但慢，0.5较抖但快
};

#endif

