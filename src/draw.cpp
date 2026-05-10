#include "draw.h"

using namespace cv;
using namespace std;

Visualizer::Visualizer()
    : fps_tick_(std::chrono::steady_clock::now()), fps_counter_(0), display_fps_(0.0)
{
    locked_color = Scalar(0, 255, 0); // 锁定显示绿色
    laser_color = Scalar(0, 0, 255);  // 激光理论点显示红色
}

void Visualizer::draw_display_fps(Mat& frame)
{
    fps_counter_++;
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - fps_tick_).count();
    if (dt >= 0.5) {
        display_fps_ = static_cast<double>(fps_counter_) / dt;
        fps_counter_ = 0;
        fps_tick_ = now;
    }
    string label = format("Display FPS: %.1f", display_fps_);
    int baseline = 0;
    Size sz = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
    Point org(frame.cols - sz.width - 10, 28);
    putText(frame, label, org, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
}

void Visualizer::draw_results(Mat &frame, const DetectResult &obj, const GimbalCmd &cmd, float real_yaw, float real_pitch, float real_roll)
{
    Scalar draw_color = cmd.is_locked ? locked_color : Scalar(255, 255, 255);
    Mat local_frame = frame; 
    // 绘制目标框边线
    for (int j = 0; j < 4; j++)
    {
        line(local_frame, obj.corners[j], obj.corners[(j + 1) % 4], draw_color, 2);
    }

    // 绘制角点序号
    for (int j = 0; j < 4; j++)
    {
        putText(local_frame, to_string(j), obj.corners[j], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
    }

    // 绘制中心点
    circle(local_frame, Point(obj.box.x + obj.box.width / 2, obj.box.y + obj.box.height / 2), 3, Scalar(0, 0, 255), -1);

    // 文本显示
    string l1 = "UAV";
    string l2 = "xyz: (" + to_string(cmd.p_world_x).substr(0, 4) + ", " + to_string(cmd.p_world_y).substr(0, 4) + ", " + to_string(cmd.p_world_z).substr(0, 4) + ")";
    string l3 = "yaw: " + to_string(cmd.target_yaw).substr(0, 5) + ", pitch: " + to_string(cmd.target_pitch).substr(0, 5);
    string l4 = "r_yaw: " + to_string(real_yaw).substr(0, 5) + ", r_pitch: " + to_string(real_pitch).substr(0, 5) + ", r_roll: " + to_string(real_roll).substr(0, 5);

    putText(local_frame, l1, Point(obj.box.x, obj.box.y - 65), FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2);
    putText(local_frame, l2, Point(obj.box.x, obj.box.y - 45), FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 1);
    putText(local_frame, l3, Point(obj.box.x, obj.box.y - 25), FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 1);
    putText(local_frame, l4, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 1);
}

void Visualizer::draw_laser_dot(Mat &frame, const Mat &cam_matrix,
                                const Eigen::Vector3f &cam_offset, const Eigen::Vector3f &ray_offset,float p_world_x)
{
    // 激光器相对于相机的偏移（相机系定义下的位移）
    Eigen::Vector3f rel_offset = ray_offset - cam_offset;

    double fx = cam_matrix.at<double>(0, 0);
    double fy = cam_matrix.at<double>(1, 1);
    double cx = cam_matrix.at<double>(0, 2);
    double cy = cam_matrix.at<double>(1, 2);

    // 设定 15 米参考点
    float dist = p_world_x;

    // --- 修改点：根据 Solver 的映射逻辑反向投影 ---
    // Solver 定义: Eigen_x = Cam_z, Eigen_y = -Cam_x, Eigen_z = -Cam_y
    // 则在相机系下，15米处点的坐标为：
    float z_cam = dist + rel_offset.x();
    float x_cam = -rel_offset.y();
    float y_cam = -rel_offset.z();

    if (z_cam > 0) {
        float u = (float)(fx * (x_cam / z_cam) + cx);
        float v = (float)(fy * (y_cam / z_cam) + cy);

        // 绘制准星（红色）
        line(frame, Point(u - 15, v), Point(u + 15, v), Scalar(0, 0, 255), 1);
        line(frame, Point(u, v - 15), Point(u, v + 15), Scalar(0, 0, 255), 1);
        putText(frame, "LASER_REF_15M", Point(u + 10, v - 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 0.5);
    }
}
