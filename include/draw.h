#ifndef DRAW_H
#define DRAW_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "detector.h"
#include "solver.h"

// 视觉反馈类：负责所有 UI 绘制逻辑
class Visualizer {
public:
    Visualizer();

    // 绘制识别结果和解算信息
    void draw_results(cv::Mat& frame, const DetectResult& obj, const GimbalCmd& cmd, float real_yaw, float real_pitch);

    /**
     * @brief 绘制激光理论击打点
     * @param frame 图像
     * @param cam_matrix 相机内参
     * @param cam_offset 相机相对于云台轴中心偏移 [x, y, z] (Eigen坐标系: 深度, 横向, 纵向)
     * @param ray_offset 激光相对于云台轴中心偏移
     */
    void draw_laser_dot(cv::Mat& frame, const cv::Mat& cam_matrix, 
                        const Eigen::Vector3f& cam_offset, const Eigen::Vector3f& ray_offset);

private:
    cv::Scalar locked_color;
    cv::Scalar laser_color;
};

#endif