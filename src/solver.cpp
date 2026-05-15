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

/** 与 solve 内一致：R_gimbal = Rz(yaw) * Ry(pitch) * Rx(roll)，角度为度（右手系）。 */
static Eigen::Matrix3f R_gimbal_from_deg(float yaw_deg, float pitch_deg, float roll_deg) {
    const float y = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    const float p = pitch_deg * static_cast<float>(M_PI) / 180.0f;
    const float r = roll_deg * static_cast<float>(M_PI) / 180.0f;
    Eigen::AngleAxisf yaw_rot(y, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_rot(p, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf roll_rot(r, Eigen::Vector3f::UnitX());
    return (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
}


/**
 * 在 Rz*Ry 两轴可控、Rx(roll) 由测量固定（无 roll 电机）的模型下，
 * 使激光轴在候选姿态 R(y,p) 下与「该姿态下激光口 → 目标点」方向对齐（最大化点积）。
 * 必须用 P_world - R*ray_offset：随 yaw/pitch 变化，激光口在世界系中平移，仅用当前姿态的弦向会偏差。
 */
static void solve_pt_ik_laser_to_world(const Eigen::Vector3f& P_world,
                                       const Eigen::Vector3f& ray_offset,
                                       const Eigen::Vector3f& d_body_unit,
                                       float seed_yaw_deg, float seed_pitch_deg, float roll_deg_fixed,
                                       float half_deg, float coarse_step_deg,
                                       int fine_passes, float fine_half_deg, float fine_step_deg,
                                       float& out_yaw_deg, float& out_pitch_deg) {
    float best_dot = -2.0f;
    out_yaw_deg = seed_yaw_deg;
    out_pitch_deg = seed_pitch_deg;
    auto score = [&](float y_deg, float p_deg) -> float {
        const Eigen::Matrix3f R = R_gimbal_from_deg(y_deg, p_deg, roll_deg_fixed);
        const Eigen::Vector3f O = R * ray_offset;
        const Eigen::Vector3f to_target = P_world - O;
        const float n = to_target.norm();
        if (n < 1e-4f) return -2.0f;
        const Eigen::Vector3f to_unit = to_target * (1.0f / n);
        return (R * d_body_unit).dot(to_unit);
    };
    for (float dy = -half_deg; dy <= half_deg + 1e-3f; dy += coarse_step_deg)
        for (float dp = -half_deg; dp <= half_deg + 1e-3f; dp += coarse_step_deg) {
            const float y = seed_yaw_deg + dy;
            const float p = seed_pitch_deg + dp;
            const float d = score(y, p);
            if (d > best_dot) {
                best_dot = d;
                out_yaw_deg = y;
                out_pitch_deg = p;
            }
        }
    for (int pass = 0; pass < fine_passes; ++pass) {
        const float cy = out_yaw_deg;
        const float cp = out_pitch_deg;
        for (float dy = -fine_half_deg; dy <= fine_half_deg + 1e-3f; dy += fine_step_deg)
            for (float dp = -fine_half_deg; dp <= fine_half_deg + 1e-3f; dp += fine_step_deg) {
                const float y = cy + dy;
                const float p = cp + dp;
                const float d = score(y, p);
                if (d > best_dot) {
                    best_dot = d;
                    out_yaw_deg = y;
                    out_pitch_deg = p;
                }
            }
    }
    out_yaw_deg = normalize_angle(out_yaw_deg);
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

    // rpy_cam_to_ray [deg]: roll 绕 +X_cam，pitch 绕 +Y_cam'，yaw 绕 +Z_cam''（内旋，右手正角）。
    // R_cam_to_ray 满足 v_ray = R_cam_to_ray * v_cam；激光发射轴取激光系 +X_ray。
    vector<float> rpy_deg = cfg.get<vector<float>>("offset.rpy_cam_to_ray", {0, 0, 0});
    if (rpy_deg.size() < 3) rpy_deg = {0, 0, 0};
    const float rr = rpy_deg[0] * static_cast<float>(M_PI) / 180.0f;
    const float pp = rpy_deg[1] * static_cast<float>(M_PI) / 180.0f;
    const float yy = rpy_deg[2] * static_cast<float>(M_PI) / 180.0f;
    R_cam_to_ray = (Eigen::AngleAxisf(yy, Eigen::Vector3f::UnitZ()) *
                    Eigen::AngleAxisf(pp, Eigen::Vector3f::UnitY()) *
                    Eigen::AngleAxisf(rr, Eigen::Vector3f::UnitX()))
                       .toRotationMatrix();
}

GimbalCmd Solver::solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll) {
    auto& cfg = ConfigManager::getInstance();
    GimbalCmd cmd{};
    
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
    Eigen::AngleAxisf roll_rot(curr_roll * M_PI / 180.0f, Eigen::Vector3f::UnitX());

    Eigen::Matrix3f R_gimbal = (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();

    // 计算世界坐标
    Eigen::Vector3f P_world = R_gimbal * (P_cam + cam_offset);
    Eigen::Vector3f P_laser_origin = R_gimbal * ray_offset;
    Eigen::Vector3f aim_vec = P_world - P_laser_origin;

    // 激光发射轴（与激光系 +X_ray 对齐）在机体/相机系中的方向，再转到世界系。
    // 与 aim_vec 共线时，几何上光束指向目标（用于可选的 lock_beam 判定）。
    Eigen::Vector3f laser_axis_body =
        (R_cam_to_ray.transpose() * Eigen::Vector3f::UnitX()).normalized();
    Eigen::Vector3f laser_axis_world = R_gimbal * laser_axis_body;
    Eigen::Vector3f aim_unit = aim_vec;
    if (aim_unit.norm() > 1e-6f) aim_unit.normalize();

     // 弦方向在世界系的 yaw/pitch，作 2 轴 IK 的初值（roll 固定为 curr_roll 时与网格最优一致）
    float chord_yaw = atan2(aim_vec.y(), aim_vec.x()) * 180.0f / static_cast<float>(M_PI);
    float dist_horiz = sqrtf(aim_vec.x() * aim_vec.x() + aim_vec.y() * aim_vec.y());
    float chord_pitch = -atan2f(aim_vec.z(), dist_horiz) * 180.0f / static_cast<float>(M_PI);

    float raw_yaw = chord_yaw;
    float raw_pitch = chord_pitch;
    if (aim_unit.norm() > 1e-6f) {
        const float half_deg = cfg.get<float>("params.ik_half_deg", 42.0f);
        const float coarse_step = cfg.get<float>("params.ik_coarse_step_deg", 2.0f);
        const int fine_passes = cfg.get<int>("params.ik_fine_passes", 2);
        const float fine_half = cfg.get<float>("params.ik_fine_half_deg", 3.5f);
        const float fine_step = cfg.get<float>("params.ik_fine_step_deg", 0.15f);
        solve_pt_ik_laser_to_world(P_world, ray_offset, laser_axis_body,
                                   chord_yaw, chord_pitch, curr_roll,
                                   half_deg, coarse_step, fine_passes, fine_half, fine_step,
                                   raw_yaw, raw_pitch);
    }

    cmd.pnp_tx = static_cast<float>(tx);
    cmd.pnp_ty = static_cast<float>(ty);
    cmd.pnp_tz = static_cast<float>(tz);

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
    // params.lock_beam_deg >= 0 时，额外要求当前姿态下激光轴与 aim 方向夹角小于该值（度）；默认 -1 关闭。
    float lock_beam_deg = cfg.get<float>("params.lock_beam_deg", -1.0f);
    bool beam_ok = true;
    if (lock_beam_deg >= 0.0f) {
        const float thr = std::cos(lock_beam_deg * static_cast<float>(M_PI) / 180.0f);
        beam_ok = (aim_unit.dot(laser_axis_world) >= thr);
    }

    if (yaw_error < lock_threshold && pitch_error < lock_threshold && beam_ok) {
        cmd.is_locked = true;
    } else {
        cmd.is_locked = false;
    }
    return cmd;
}

void Solver::reset_filter() {
    is_first_run = true;
    last_yaw = 0.0f;
    last_pitch = 0.0f;
}