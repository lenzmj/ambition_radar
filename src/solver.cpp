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

    double fx = cfg.get<double>("camera.fx", 2571.7);
    double fy = cfg.get<double>("camera.fy", 2571.2);
    double cx = cfg.get<double>("camera.cx", 1444.7);
    double cy = cfg.get<double>("camera.cy", 1088.5);
    camera_matrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    vector<double> dist = cfg.get<vector<double>>("camera.dist_coeffs", {0, 0, 0, 0, 0});
    dist_coeffs = Mat(dist).clone().reshape(1, 1);

    float w = cfg.get<float>("target.width", 0.060f);
    float h = cfg.get<float>("target.height", 0.065f);
    
    object_3d_points.clear();
    object_3d_points.push_back(Point3f(0, -h/2,  w/2)); 
    object_3d_points.push_back(Point3f(0,  h/2,  w/2)); 
    object_3d_points.push_back(Point3f(0,  h/2, -w/2)); 
    object_3d_points.push_back(Point3f(0, -h/2, -w/2));

    vector<float> c_off = cfg.get<vector<float>>("offset.cam_to_gimbal", {0, 0, 0});
    cam_offset << c_off[0], c_off[1], c_off[2];
    
    vector<float> r_off = cfg.get<vector<float>>("offset.ray_to_gimbal", {0, 0, 0});
    ray_offset << r_off[0], r_off[1], r_off[2];
}

GimbalCmd Solver::solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll) {
    auto& cfg = ConfigManager::getInstance();
    GimbalCmd cmd;
    
    Mat rvec, tvec;
    if (target.corners.size() != 4) return cmd;
    
    bool success = solvePnP(object_3d_points, target.corners, camera_matrix, dist_coeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
    if (!success) return cmd;

    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    if (tz <= 0) return cmd; 

    // 相机系到云台(Eigen)系映射
    Eigen::Vector3f P_cam((float)tz, (float)-tx, (float)-ty);

    // 云台当前姿态旋转矩阵
    Eigen::AngleAxisf yaw_rot(curr_yaw * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
// --- 修改部分 1：修正当前 Pitch 状态的旋转极性 ---
    // 原代码为: Eigen::AngleAxisf pitch_rot(-curr_pitch * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    // 既然云台往反方向跑，说明下位机反馈的 Pitch 极性与代码原本假设的相反。
    // 在这里去掉负号，使得下位机当前的真实姿态能正确且符合右手法则地映射回世界坐标系。
    Eigen::AngleAxisf pitch_rot(curr_pitch * M_PI / 180.0f, Eigen::Vector3f::UnitY());


    Eigen::Matrix3f R_gimbal = (yaw_rot * pitch_rot).toRotationMatrix();

    // 计算世界坐标
    Eigen::Vector3f P_world = R_gimbal * (P_cam + cam_offset);
    Eigen::Vector3f P_laser_origin = R_gimbal * ray_offset;
    Eigen::Vector3f aim_vec = P_world - P_laser_origin;

    // 计算角度
    float raw_yaw = atan2(aim_vec.y(), aim_vec.x()) * 180.0f / M_PI;
    float dist_horiz = sqrt(aim_vec.x()*aim_vec.x() + aim_vec.y()*aim_vec.y());
    float raw_pitch = -atan2(aim_vec.z(), dist_horiz) * 180.0f / M_PI;

    // 滤波处理
    float alpha_val = cfg.get<float>("params.solve_alpha", 0.3f);
    if (is_first_run) {
        last_yaw = raw_yaw; last_pitch = raw_pitch; is_first_run = false;
    } else {
        last_yaw = last_yaw + alpha_val * normalize_angle(raw_yaw - last_yaw);
        last_pitch = last_pitch + alpha_val * (raw_pitch - last_pitch);
    }

    cmd.target_yaw = last_yaw;
    cmd.target_pitch = last_pitch;

    // --- 修改部分：补全世界坐标赋值 ---
    // 之前只写了 p_world_x，现在把 y 和 z 也传回去，解决“y和z一直为0”的问题
    cmd.p_world_x = P_world.x(); // 深度 (前向)
    cmd.p_world_y = P_world.y(); // 横向 (左向)
    cmd.p_world_z = P_world.z(); // 纵向 (上向)
    // --- 修改部分：根据角度偏差决定是否“锁定” ---
    // 1. 获取 YAML 里的锁定阈值（例如 0.5 度）
    float lock_threshold = cfg.get<float>("params.lock_range", 0.3f);

    // 2. 计算当前角度与目标角度的差值 (Yaw 需要做角度归一化)
    float yaw_error = abs(normalize_angle(cmd.target_yaw - curr_yaw));
    float pitch_error = abs(cmd.target_pitch - curr_pitch);

    // 3. 只有当误差小于阈值时，is_locked 才为 true (变绿)
    // 否则为 false (变白)，表示正在跟踪但尚未瞄准
    if (yaw_error < lock_threshold && pitch_error < lock_threshold) {
        cmd.is_locked = true;
    } else {
        cmd.is_locked = false;
    }
    return cmd;
}