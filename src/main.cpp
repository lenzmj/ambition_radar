#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque> // 修改：使用双端队列维护姿态历史记录[cite: 3]
#include "HikDriver.h"
#include "detector.h"
#include "solver.h"
#include "Protocol.h"
#include "SerialDriver.h"
#include "yaml.hpp"
#include "draw.h"

using namespace std;
using namespace cv;

// 修改：定义云台姿态结构体，将角度与时间戳绑定[cite: 3]
struct GimbalPose {
    float yaw;
    float pitch;
    float roll;
    uint64_t timestamp;
};

Mat shared_frame;
uint64_t shared_timestamp; // 修改：增加图像时间戳的共享变量
mutex frame_mtx;
mutex pose_mtx;           // 修改：增加保护姿态队列的互斥锁
deque<GimbalPose> pose_buffer; // 修改：存储姿态数据的缓冲区

atomic<bool> is_running(true); 
atomic<bool> has_new_frame(false); 
float distance_d = 15.0f; 

// --- 取图线程 ---
void capture_task(HikDriver* cam) {
    Mat tmp_frame;
    uint64_t tmp_ts; // 修改：临时存储图像时间戳
    auto last_success_time = std::chrono::steady_clock::now();

    while (is_running) {
        // 修改：获取带时间戳的图像[cite: 2, 8]
        bool success = cam->get_frame(tmp_frame, tmp_ts);
        
        if (success && !tmp_frame.empty()) {
            last_success_time = std::chrono::steady_clock::now();
            
            frame_mtx.lock();
            tmp_frame.copyTo(shared_frame); 
            shared_timestamp = tmp_ts; // 修改：同步更新时间戳
            has_new_frame = true;
            frame_mtx.unlock();
        } else {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_success_time).count() > 1) {
                cam->close_camera();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                cam->connect(); 
                last_success_time = now; 
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// --- 串口独立处理线程 ---
void serial_task(SerialDriver* serial) {
    ReceivePacket rec;
    while (is_running) {
        if (serial->receive_packet(rec)) {
            // 修改原因：下位机不带时间戳，我们在接收成功的瞬间手动“补打”系统时间。
            // 这样姿态数据就有了与图像数据相同的参考坐标系[cite: 3, 4]
            GimbalPose current_pose;
            current_pose.yaw = rec.current_yaw;
            current_pose.pitch = rec.current_pitch;
            current_pose.roll = rec.current_roll;
            current_pose.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

            lock_guard<mutex> lock(pose_mtx);
            pose_buffer.push_back(current_pose);
            
            // 修改原因：限制队列长度。500 帧左右（约1秒数据）足以覆盖视觉链路延迟[cite: 3]
            if (pose_buffer.size() > 500) {
                pose_buffer.pop_front();
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

int main() {
    ConfigManager::getInstance().init("/home/lenzmj/ws/ambition_radar/config/config.yaml");
    auto& cfg = ConfigManager::getInstance();

    HikDriver camera;
    camera.set_isp_from_config(
        cfg.get<double>("camera.exposure_time_us", -1.0),
        cfg.get<double>("camera.gain", -1.0));
    if (!camera.connect()) return -1;

    string port = cfg.get<string>("hardware.serial_port", "");
    SerialDriver serial(port.c_str());

    string model_path = cfg.get<string>("hardware.model_path", "");
    Detector pikachu_ai(model_path.c_str()); 

    Solver math_solver;
    Visualizer drawer;

    thread t_serial(serial_task, &serial);
    thread t_capture(capture_task, &camera);

    Mat local_frame;
    uint64_t local_timestamp; // 修改：本循环处理的图像时间戳
    namedWindow("Pikachu View", WINDOW_NORMAL);
    resizeWindow("Pikachu View", 800, 600);

    while (is_running) {
        if (has_new_frame) {
            frame_mtx.lock();
            shared_frame.copyTo(local_frame);
            local_timestamp = shared_timestamp; // 修改：拷贝对应的采集时间
            has_new_frame = false;
            frame_mtx.unlock();

            // --- 修改部分：时间戳对齐逻辑 ---
            float matched_yaw = 0.0f;
            float matched_pitch = 0.0f;
            float matched_roll = 0.0f;
            bool find_matched = false;

            {   //别动这个{}锁，保护整个对齐过程的，确保姿态数据不被串口线程修改[cite: 3]
                lock_guard<mutex> lock(pose_mtx);
                if (!pose_buffer.empty()) {
                    // 修改原因：遍历 Buffer，寻找时间戳与图像 local_timestamp 最接近的那一组 RPY。
                    // 15m 精度要求下，寻找最近邻是最稳健的做法[cite: 3, 5]
                    uint64_t min_diff = 0xFFFFFFFFFFFFFFFF;
                    auto best_it = pose_buffer.begin();

                    for (auto it = pose_buffer.begin(); it != pose_buffer.end(); ++it) {
                        uint64_t diff = (local_timestamp > it->timestamp) ? 
                                        (local_timestamp - it->timestamp) : (it->timestamp - local_timestamp);
                        if (diff < min_diff) {
                            min_diff = diff;
                            best_it = it;
                        }
                    }
                    matched_yaw = best_it->yaw;
                    matched_pitch = best_it->pitch;
                    matched_roll = best_it->roll;
                    find_matched = true;
                }
            }

            if (!find_matched) continue;

            // --- 修改：在绘制激光点时，使用对齐后的 matched_yaw/pitch ---
            // 这样绘制出的激光点能反映图像拍摄时刻激光相对于画面的真实位置[cite: 3, 5]
            drawer.draw_laser_dot(local_frame, math_solver.camera_matrix, math_solver.cam_offset, math_solver.ray_offset, distance_d);

            vector<DetectResult> results = pikachu_ai.run_yolo(local_frame);

            for (auto &obj : results) {
                // 修改说明：解算时必须使用图像采集时的姿态 matched_yaw/pitch。
                // 如果使用此时最新的串口角度，由于处理耗时（约 30-50ms），云台可能已移动，导致解算脱靶[cite: 3, 5]
                GimbalCmd cmd = math_solver.solve(results[0], matched_yaw, matched_pitch, matched_roll);
                distance_d = cmd.p_world_x;

                drawer.draw_results(local_frame, obj, cmd, matched_yaw, matched_pitch, matched_roll);

                SendPacket pkt;
                pkt.mode = 1;
                pkt.pitch = cmd.target_pitch;
                pkt.yaw = cmd.target_yaw;
                pkt.distance = cmd.p_world_x;
                serial.send_packet(pkt);
            }

            drawer.draw_display_fps(local_frame);

            imshow("Pikachu View", local_frame);
        }

        if ((waitKey(1) & 0xFF) == 27) { 
            is_running = false;
            break;
        }
    }

    if (t_capture.joinable()) t_capture.join();
    if (t_serial.joinable()) t_serial.join();
    camera.close_camera();

    return 0;
}
