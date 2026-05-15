#include "rpy_cam_to_ray.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace {

// 与 draw.cpp / Solver 一致：Eigen 机体 (前,左,上) <-> OpenCV 相机 (右,下,前)
inline Eigen::Vector3f eigen_to_cam(const Eigen::Vector3f& v) {
    return Eigen::Vector3f(-v.y(), -v.z(), v.x());
}

/** 单位方向 dir_cam -> 机体系单位向量，满足 eigen_to_cam(body) 与 dir_cam 平行 */
inline Eigen::Vector3f dir_body_from_dir_cam(const Eigen::Vector3f& dir_cam) {
    Eigen::Vector3f d = dir_cam.normalized();
    Eigen::Vector3f b(d.z(), -d.x(), -d.y());
    return b.normalized();
}

/** 像素 -> 归一化平面 (undistortPoints)，再与 Z=plane_z 平面求交（OpenCV 相机系，前向 Z>0） */
bool intersect_ray_plane_z(const cv::Mat& K, const cv::Mat& dist, float u, float v, float plane_z,
                             Eigen::Vector3f& out_cam) {
    if (K.empty() || plane_z <= 1e-6f) return false;
    std::vector<cv::Point2f> src{ cv::Point2f(u, v) }, dst;
    cv::undistortPoints(src, dst, K, dist);
    const float xn = dst[0].x;
    const float yn = dst[0].y;
    // 相机光心出发方向 (xn, yn, 1)，与 Z = plane_z 相交：k * 1 = plane_z
    out_cam = Eigen::Vector3f(xn * plane_z, yn * plane_z, plane_z);
    return std::isfinite(out_cam.x()) && std::isfinite(out_cam.y()) && std::isfinite(out_cam.z());
}

/**
 * v_ray = R * v_cam（与 Solver 一致），激光轴为 laser 系 +X：laser_axis_cam = R^T * e_x == R 的第一行转置。
 * 构造旋转矩阵使 laser_axis_body = d_body（列向量，机体系）。
 */
Eigen::Matrix3f R_cam_to_ray_from_laser_axis_body(const Eigen::Vector3f& laser_axis_body) {
    Eigen::Vector3f r0 = laser_axis_body.normalized();
    Eigen::Vector3f a = Eigen::Vector3f::UnitY();
    if (std::fabs(r0.dot(a)) > 0.9f) a = Eigen::Vector3f::UnitX();
    Eigen::Vector3f r1 = (a - r0 * r0.dot(a)).normalized();
    Eigen::Vector3f r2 = r0.cross(r1);
    Eigen::Matrix3f R;
    R.row(0) = r0.transpose();
    R.row(1) = r1.transpose();
    R.row(2) = r2.transpose();
    if (R.determinant() < 0.0f) {
        R.row(2) = -R.row(2);
    }
    return R;
}

/** R = Rz(yaw)*Ry(pitch)*Rx(roll)，与 solver.cpp 构造一致，输出角度（弧度） */
void matrix_to_rpy_rad(const Eigen::Matrix3f& R, float& roll, float& pitch, float& yaw) {
    const float sinp = -R(2, 0);
    const float cosp = std::sqrt(std::max(0.0f, R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    if (cosp > 1e-6f) {
        yaw = std::atan2(R(1, 0), R(0, 0));
        pitch = std::atan2(sinp, cosp);
        roll = std::atan2(R(2, 1), R(2, 2));
    } else {
        yaw = std::atan2(-R(0, 1), R(1, 1));
        pitch = std::atan2(sinp, cosp);
        roll = 0.0f;
    }
}

struct ClickUi {
    cv::Mat canvas;
    cv::Mat K;
    cv::Mat dist;
    Eigen::Vector3f ap_cam;
    float plane_z{};
    Eigen::Matrix3f R_suggested{};
    int phase{}; // 0 等第一次点击，1 等第二次，2 结束
    cv::Point2f p_sim{};
    cv::Point2f p_act{};
};

void on_mouse(int event, int x, int y, int, void* userdata) {
    auto* st = static_cast<ClickUi*>(userdata);
    if (!st || st->phase >= 2) return;
    if (event != cv::EVENT_LBUTTONDOWN) return;
    if (st->phase == 0) {
        st->p_sim = cv::Point2f((float)x, (float)y);
        st->phase = 1;
        cv::circle(st->canvas, cv::Point(x, y), 6, cv::Scalar(0, 255, 255), 2);
        cv::putText(st->canvas, "1 OK", cv::Point(x + 8, y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                     cv::Scalar(0, 255, 255), 1);
        cv::imshow("RPY_CAM_TO_RAY", st->canvas);
        std::cout << "[rpy_calib] 已记录模拟点 (" << x << ", " << y << ")，请点击实际击打点。\n";
        return;
    }
    if (st->phase == 1) {
        st->p_act = cv::Point2f((float)x, (float)y);

        Eigen::Vector3f hit_cam;
        if (!intersect_ray_plane_z(st->K, st->dist, st->p_act.x, st->p_act.y, st->plane_z, hit_cam)) {
            std::cerr << "[rpy_calib] 反投影失败，检查 K/dist/tz。\n";
            return;
        }
        Eigen::Vector3f chord = hit_cam - st->ap_cam;
        if (chord.norm() < 1e-4f) {
            std::cerr << "[rpy_calib] 击打点与激光口方向退化，请重新点实际击打点。\n";
            return;
        }
        Eigen::Vector3f dir_cam = chord.normalized();
        Eigen::Vector3f axis_body = dir_body_from_dir_cam(dir_cam);
        st->R_suggested = R_cam_to_ray_from_laser_axis_body(axis_body);

        float rr, pp, yy;
        matrix_to_rpy_rad(st->R_suggested, rr, pp, yy);
        const float deg = 180.0f / static_cast<float>(M_PI);

        std::cout << "\n========== rpy_cam_to_ray 标定结果 ==========\n";
        std::cout << "前提：cam_to_gimbal、ray_to_gimbal 正确；击打与 PnP 共面 Z=pnp_tz = " << st->plane_z
                  << " m。\n";
        std::cout << "将以下写入 config 中 offset.rpy_cam_to_ray（单位：度，顺序 [roll, pitch, yaw]）：\n\n";
        std::printf("  rpy_cam_to_ray: [%.6f, %.6f, %.6f]\n\n", rr * deg, pp * deg, yy * deg);
        std::cout << "说明：第一点为记录的模拟十字位置；第二点为实际击打。若旋转模型与 Solver 一致，则此组 rpy "
                     "可使「等距平面」上模拟十字与第二点重合。\n";
        std::cout << "============================================\n\n";

        st->phase = 2;
        cv::circle(st->canvas, cv::Point(x, y), 6, cv::Scalar(0, 0, 255), 2);
        cv::putText(st->canvas, "2 OK", cv::Point(x + 8, y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                     cv::Scalar(0, 0, 255), 1);
        cv::imshow("RPY_CAM_TO_RAY", st->canvas);
    }
}

} // namespace

void run_rpy_calib_two_clicks(const RpyCamToRayInput& in) {
    if (in.frame_bgr.empty() || in.camera_matrix.empty() || in.pnp_tz <= 1e-4f) {
        std::cerr << "[rpy_calib] 无效输入：需要非空帧与有效 pnp_tz。\n";
        return;
    }

    ClickUi st;
    in.frame_bgr.copyTo(st.canvas);
    st.K = in.camera_matrix;
    st.dist = in.dist_coeffs.empty() ? cv::Mat() : in.dist_coeffs;
    st.plane_z = in.pnp_tz;
    st.ap_cam = eigen_to_cam(in.ray_offset - in.cam_offset);
    st.phase = 0;

    cv::namedWindow("RPY_CAM_TO_RAY", cv::WINDOW_NORMAL);
    const char* hint =
        "1: click LASER_REF center  2: click actual hit  ESC: cancel";
    cv::putText(st.canvas, hint, cv::Point(10, st.canvas.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(200, 255, 200), 1);
    cv::imshow("RPY_CAM_TO_RAY", st.canvas);
    cv::setMouseCallback("RPY_CAM_TO_RAY", on_mouse, &st);

    std::cout << "[rpy_calib] 已打开标定窗口：请先点模拟十字中心，再点实际击打点。ESC 取消。\n";

    for (;;) {
        int k = cv::waitKey(20) & 0xFF;
        if (k == 27) {
            std::cout << "[rpy_calib] 已取消。\n";
            break;
        }
        if (st.phase >= 2) {
            cv::waitKey(500);
            break;
        }
    }
    cv::setMouseCallback("RPY_CAM_TO_RAY", nullptr, nullptr);
    cv::destroyWindow("RPY_CAM_TO_RAY");
}
