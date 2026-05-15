#ifndef DRAW_H
#define DRAW_H

#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "detector.h"
#include "solver.h"

// 视觉反馈类：负责所有 UI 绘制逻辑
class Visualizer {
public:
    Visualizer();

    // 绘制识别结果和解算信息（cam_matrix 用于将 PnP 目标中心投影到像素，与 LASER_REF 同一几何基准）
    void draw_results(cv::Mat& frame, const cv::Mat& cam_matrix, const DetectResult& obj, const GimbalCmd& cmd,
                      float real_yaw, float real_pitch, float real_roll);

    /**
     * @brief 绘制激光理论击打点（与 Solver 一致：激光系 +X 为发射轴，rpy_cam_to_ray 定义 R_cam_to_ray）
     * @param frame 图像
     * @param cam_matrix 相机内参
     * @param cam_offset 相机相对于云台轴中心偏移 [x, y, z] (Eigen: 前, 左, 上)
     * @param ray_offset 激光口相对于云台轴中心偏移 [x, y, z] (同上)
     * @param R_cam_to_ray v_ray = R_cam_to_ray * v_cam
     * @param pnp_tx, pnp_ty, pnp_tz solvePnP 的 tvec（OpenCV 相机系），激光射线与过该点的 Z=const 平面求交后投影，与 PnP 目标对齐
     */
    void draw_laser_dot(cv::Mat& frame, const cv::Mat& cam_matrix,
                        const Eigen::Vector3f& cam_offset, const Eigen::Vector3f& ray_offset,
                        const Eigen::Matrix3f& R_cam_to_ray,
                        float pnp_tx, float pnp_ty, float pnp_tz);

    /** 统计并叠加显示刷新帧率（主线程每次 imshow 前调用一次） */
    void draw_display_fps(cv::Mat& frame);

private:
    cv::Scalar locked_color;
    cv::Scalar laser_color;

    std::chrono::steady_clock::time_point fps_tick_;
    int fps_counter_;
    double display_fps_;
};

#endif