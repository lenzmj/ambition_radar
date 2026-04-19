#include "solver.h"
#include "yaml.hpp" 
#include <Eigen/Geometry>
#include <cmath>

using namespace cv;
using namespace std;

static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

Solver::Solver() : is_first_run(true), last_yaw(0), last_pitch(0) {
    auto& cfg = ConfigManager::getInstance();

    // 1. 从 YAML 加载相机内参
    double fx = cfg.get<double>("camera.fx", 3200.0);
    double fy = cfg.get<double>("camera.fy", 3200.0);
    double cx = cfg.get<double>("camera.cx", 1536.0);
    double cy = cfg.get<double>("camera.cy", 1024.0);
    camera_matrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    vector<double> dist = cfg.get<vector<double>>("camera.dist_coeffs", {0, 0, 0, 0, 0});
    dist_coeffs = Mat(dist).clone().reshape(1, 1);

    // 2. 从 YAML 加载目标尺寸并建立 3D 模型
    float target_w = cfg.get<float>("target.width", 0.13f);
    float target_h = cfg.get<float>("target.height", 0.13f);
    float r = target_w / 2.0f; 
    
    object_3d_points.clear();
    // Corner 0-3 顺序
    object_3d_points.push_back(Point3f(-r, 0, 0));
    object_3d_points.push_back(Point3f(-r, 0, target_h));
    object_3d_points.push_back(Point3f( r, 0, target_h));
    object_3d_points.push_back(Point3f( r, 0, 0));

    // 3. 加载结构偏移
    vector<float> c_off = cfg.get<vector<float>>("offset.cam_to_gimbal", {0, 0, 0});
    cam_offset << c_off[0], c_off[1], c_off[2];
    
    vector<float> r_off = cfg.get<vector<float>>("offset.ray_to_gimbal", {0, 0, 0});
    ray_offset << r_off[0], r_off[1], r_off[2];
}

GimbalCmd Solver::solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll) {
    auto& cfg = ConfigManager::getInstance();
    GimbalCmd cmd;
    
    // 获取实时参数
    float h = cfg.get<float>("target.height", 0.13f);
    float alpha_val = cfg.get<float>("params.solve_alpha", 0.08f);

    Mat rvec, tvec;
    if (!solvePnP(object_3d_points, target.corners, camera_matrix, dist_coeffs, rvec, tvec, false, SOLVEPNP_EPNP)) {
        cmd.is_locked = false;
        return cmd;
    }

    float tx = (float)tvec.at<double>(0);
    float ty = (float)tvec.at<double>(1);
    float tz = (float)tvec.at<double>(2);
    Eigen::Vector3f P_cam(tz, -tx, -ty); 
    std::cout << "P_cam: x=" << P_cam.x() << ", y=" << P_cam.y() << ", z=" << P_cam.z() << std::endl;

    Eigen::AngleAxisf yaw_rev(curr_yaw * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_rev(curr_pitch * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f R_world = (yaw_rev * pitch_rev).toRotationMatrix();

    Eigen::Vector3f P_world = R_world * (P_cam + cam_offset);
    Eigen::Vector3f aim_vec = P_world - (R_world * ray_offset);

    float raw_yaw = atan2(aim_vec.y(), aim_vec.x()) * 180.0f / M_PI;
    // 补偿：目标中心高度
    float raw_pitch = atan2(aim_vec.z() - (h / 2.0f), aim_vec.norm()) * 180.0f / M_PI;

    if (is_first_run) {
        last_yaw = raw_yaw; last_pitch = raw_pitch; is_first_run = false;
    } else {
        // 使用从 YAML 读取的 alpha_val
        last_yaw = normalize_angle(last_yaw + alpha_val * normalize_angle(raw_yaw - last_yaw));
        last_pitch = last_pitch + alpha_val * (raw_pitch - last_pitch);
    }

    cmd.target_yaw = last_yaw;
    cmd.target_pitch = last_pitch;
    cmd.p_world_x = P_world.x();
    cmd.p_world_y = P_world.y();
    cmd.p_world_z = P_world.z();

    float lock_range = cfg.get<float>("params.lock_range", 5.0f);
    cmd.is_locked = (sqrt(pow(normalize_angle(cmd.target_yaw - curr_yaw), 2) + pow(cmd.target_pitch - curr_pitch, 2)) < lock_range);

    return cmd;
}
