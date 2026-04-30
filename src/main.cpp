#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include "HikDriver.h"
#include "detector.h"
#include "solver.h"
#include "Protocol.h"
#include "SerialDriver.h"
#include "yaml.hpp"
#include "draw.h"

using namespace std;
using namespace cv;


Mat shared_frame;          
mutex frame_mtx;           
atomic<bool> is_running(true); 
atomic<bool> has_new_frame(false); 
atomic<float> latest_real_yaw(0.0f);
atomic<float> latest_real_pitch(0.0f);
float distance_d = 15.0f; // 默认距离






// --- 取图线程 ---
void capture_task(HikDriver* cam) {
    Mat tmp_frame;
    auto last_success_time = std::chrono::steady_clock::now();

    while (is_running) {
        // 增加超时检测机制
        bool success = cam->get_frame(tmp_frame);
        
        if (success && !tmp_frame.empty()) {
            last_success_time = std::chrono::steady_clock::now();
            
            frame_mtx.lock();
            tmp_frame.copyTo(shared_frame); 
            has_new_frame = true;
            frame_mtx.unlock();
        } else {
            // 检测是否卡死（超过 1 秒没收到图）
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_success_time).count() > 1) {
                std::cerr << "[Critical] 相机线程疑似卡死，尝试自动重连..." << std::endl;
                cam->close_camera();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                cam->connect(); // 尝试重连
                last_success_time = now; // 重置计时
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
            // 更新原子变量，保证主循环随时读取到最新值
            latest_real_yaw = rec.current_yaw;
            latest_real_pitch = rec.current_pitch;
        }
        // 极短休眠，防止 CPU 满载
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}


int main() {
    // 1. 先初始化管家，指定你的 config.yaml 绝对路径
    ConfigManager::getInstance().init("/home/lenzmj/ws/ambition_radar/config/config.yaml");
    auto& cfg = ConfigManager::getInstance();

    // 2. 初始化相机
    HikDriver camera;
    if (!camera.connect()) {
        cout << "Camera Connect Failed!" << endl;
        return -1;
    }

    // 3. 初始化串口 (建议串口号也从 YAML 读)
    string port = cfg.get<string>("hardware.serial_port", "");
    SerialDriver serial(port.c_str());
    cout << "[检查] 内存中读取到的串口路径: " << port.c_str()<< endl;

    // 4. 初始化 AI 模型 (路径也可以从 YAML 读)
    string model_path = cfg.get<string>("hardware.model_path", "");
    cout << "[检查] 模型路径: " << model_path << endl;

    Detector pikachu_ai(model_path.c_str()); 

    
    Solver math_solver;
    Visualizer drawer; // 绘图

    // 独立接收串口线程
    thread t_serial(serial_task, &serial);
    // 独立取图线程
    thread t_capture(capture_task, &camera);


    Mat local_frame;
    namedWindow("Pikachu View", WINDOW_NORMAL);
    resizeWindow("Pikachu View", 800, 600);

    //下位机角度
    
    float real_yaw = 0.0f;
    float real_pitch = 0.0f;
    float real_roll = 0.0f; 

    cout << "初始化完毕" << endl;

    while (true)
    {

        float real_yaw = latest_real_yaw.load();
        float real_pitch = latest_real_pitch.load();
        std::cout  <<"----------------------------------------------------------------" <<  std::endl;
        std::cout <<"pitch:" << real_pitch   <<" , yaw:" << real_yaw << std::endl;
        if (has_new_frame)
        {
            frame_mtx.lock();
            shared_frame.copyTo(local_frame);
            has_new_frame = false;
            frame_mtx.unlock();
        
            // --- 无论是否发现目标，都绘制激光理论点 ---
            drawer.draw_laser_dot(local_frame, math_solver.camera_matrix, 
                                 math_solver.cam_offset, math_solver.ray_offset,distance_d);

            vector<DetectResult> results = pikachu_ai.run_yolo(local_frame);
            

            for (auto &obj : results)
            {
                // 1. 解算

                GimbalCmd cmd = math_solver.solve(results[0], real_yaw, real_pitch, 0.0f);
                distance_d = cmd.p_world_x; // 更新全局距离变量，供绘制函数使用

                // 2. 绘制（解算结果 + 视觉反馈）

                drawer.draw_results(local_frame, obj, cmd, real_yaw, real_pitch);

                // 3. 封装并发送
                SendPacket pkt;
                pkt.mode = 1;
                pkt.pitch = cmd.target_pitch;
                pkt.yaw = cmd.target_yaw;
                pkt.distance = cmd.p_world_x;
                serial.send_packet(pkt);
                cout <<  "pitch:" << pkt.pitch  <<  " ,yaw:"  << pkt.yaw << " ,距离:" << pkt.distance <<endl;
                std::cout  <<"----------------------------------------------------------------" <<  std::endl;
            }

            imshow("Pikachu View", local_frame);
        }

        // 按键处理
        int key = waitKey(1) & 0xFF;
        if (key == 27)
        { // ESC 退出
            is_running = false;
            break;
        }
    }

    // 等待子线程结束并释放资源
    if (t_capture.joinable())
        t_capture.join();
    camera.close_camera();

    return 0;
}
