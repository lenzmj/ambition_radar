#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque> // 修改：使用双端队列维护姿态历史记录[cite: 3]
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <algorithm>
#include <cctype>
#include "HikDriver.h"
#include "detector.h"
#include "solver.h"
#include "Protocol.h"
#include "SerialDriver.h"
#include "yaml.hpp"
#include "draw.h"
#include "rpy_cam_to_ray.h"

using namespace std;
using namespace cv;

// 修改：定义云台姿态结构体，将角度与时间戳绑定[cite: 3]
struct GimbalPose {
    float yaw;
    float pitch;
    float roll;
    uint64_t timestamp;
};

Mat shared_frame;       // BGR，供检测/显示/录制
uint64_t shared_timestamp; // 修改：增加图像时间戳的共享变量
mutex frame_mtx;
mutex pose_mtx;           // 修改：增加保护姿态队列的互斥锁
deque<GimbalPose> pose_buffer; // 修改：存储姿态数据的缓冲区

atomic<bool> is_running(true); 
atomic<bool> has_new_frame(false); 

mutex record_mtx;
deque<Mat> record_queue;
condition_variable record_cv;
atomic<uint64_t> record_drops{0};

constexpr size_t kRecordQueueMax = 16;

static void draw_yolo_only_preview(Mat& frame, const DetectResult& obj) {
    for (int j = 0; j < 4; ++j) {
        line(frame, obj.corners[j], obj.corners[(j + 1) % 4], Scalar(0, 255, 0), 2);
    }
    putText(frame, format("score %.2f", obj.score), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7,
            Scalar(0, 255, 0), 2);
}

static bool open_mp4_writer(VideoWriter& writer, const string& path, double fps, const Size& size) {
    const int codecs[] = {
        VideoWriter::fourcc('m', 'p', '4', 'v'),
        VideoWriter::fourcc('a', 'v', 'c', '1'),
        VideoWriter::fourcc('H', '2', '6', '4'),
    };
    for (int fourcc : codecs) {
        if (writer.open(path, fourcc, fps, size, true))
            return true;
    }
    return false;
}

void record_video_writer_task(const string& video_path, double video_fps) {
    VideoWriter writer;
    bool writer_ready = false;
    size_t frames_written = 0;

    while (true) {
        Mat frame;
        {
            unique_lock<mutex> lock(record_mtx);
            record_cv.wait(lock, [] {
                return !record_queue.empty() || !is_running.load();
            });
            if (record_queue.empty() && !is_running.load())
                break;
            if (record_queue.empty())
                continue;
            frame = std::move(record_queue.front());
            record_queue.pop_front();
        }

        if (!writer_ready) {
            if (!open_mp4_writer(writer, video_path, video_fps, frame.size())) {
                std::cerr << "[dataset] 无法创建 MP4: " << video_path << "\n";
                return;
            }
            writer_ready = true;
            std::cout << "[dataset] MP4: " << video_path << " (" << frame.cols << "x" << frame.rows
                      << " @ " << video_fps << " fps)\n";
        }
        writer.write(frame);
        ++frames_written;
    }

    if (writer_ready)
        writer.release();

    const uint64_t drops = record_drops.exchange(0);
    if (drops > 0) {
        std::cerr << "[dataset] 录像队列满，丢弃 " << drops << " 帧\n";
    }
    if (frames_written > 0) {
        std::cout << "[dataset] 录像结束，共 " << frames_written << " 帧 -> " << video_path << "\n";
    } else if (writer_ready) {
        std::filesystem::remove(video_path);
    }
}

static string make_record_video_path(const string& base_dir, const string& stamp) {
    return base_dir + "/record_" + stamp + ".mp4";
}

static string normalize_run_mode(string mode) {
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return mode;
}

void video_playback_task(const string& video_path, bool loop) {
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "[test] 无法打开视频: " << video_path << "\n";
        is_running = false;
        return;
    }
    double fps = cap.get(CAP_PROP_FPS);
    if (fps <= 1.0 || fps > 240.0)
        fps = 30.0;
    const auto frame_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / fps));
    auto next_tick = std::chrono::steady_clock::now();

    std::cout << "[test] 回放 " << video_path << " (" << cap.get(CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(CAP_PROP_FRAME_HEIGHT) << " @ " << fps << " fps";
    if (loop)
        std::cout << ", 循环";
    std::cout << ")\n";

    while (is_running) {
        Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            if (!loop)
                break;
            cap.set(CAP_PROP_POS_FRAMES, 0);
            if (!cap.read(frame) || frame.empty())
                break;
        }

        frame_mtx.lock();
        frame.copyTo(shared_frame);
        shared_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        has_new_frame = true;
        frame_mtx.unlock();

        next_tick += frame_period;
        std::this_thread::sleep_until(next_tick);
    }
}

static string make_record_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

// --- 取图线程 ---
void capture_task(HikDriver* cam,const string& target_sn) {
    Mat tmp_rgb;
    uint64_t tmp_ts; // 修改：临时存储图像时间戳
    auto last_success_time = std::chrono::steady_clock::now();

    while (is_running) {
        // 修改：获取带时间戳的图像[cite: 2, 8]
        bool success = cam->get_frame(tmp_rgb, tmp_ts);
        
        if (success && !tmp_rgb.empty()) {
            last_success_time = std::chrono::steady_clock::now();
            
            frame_mtx.lock();
            tmp_rgb.copyTo(shared_frame);
            shared_timestamp = tmp_ts; // 修改：同步更新时间戳
            has_new_frame = true;
            frame_mtx.unlock();
        } else {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_success_time).count() > 1) {
                cam->close_camera();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                cam->connect(target_sn);
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
            // 下位机不带时间戳，我们在接收成功的瞬间手动“补打”系统时间。
            // 这样姿态数据就有了与图像数据相同的参考坐标系[cite: 3, 4]
            GimbalPose current_pose;
            current_pose.yaw = rec.current_yaw;
            current_pose.pitch = rec.current_pitch;
            current_pose.roll = rec.current_roll;
            current_pose.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

            lock_guard<mutex> lock(pose_mtx);
            pose_buffer.push_back(current_pose);
            
            // 限制队列长度。500 帧左右（约1秒数据）足以覆盖视觉链路延迟[cite: 3]
            if (pose_buffer.size() > 400) {
                pose_buffer.pop_front();
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

int main() {
    ConfigManager::getInstance().init("/home/lenzmj/ws/ambition_radar/config/config.yaml");
    auto& cfg = ConfigManager::getInstance();

    const string run_mode = normalize_run_mode(cfg.get<string>("run.mode", "hik"));
    const bool is_hik = (run_mode == "hik");
    const bool is_test = (run_mode == "test");
    if (!is_hik && !is_test) {
        std::cerr << "[run] 无效 mode=\"" << run_mode << "\"，仅支持 hik | test\n";
        return -1;
    }

    const string test_video = cfg.get<string>("test.video_path", "");
    const bool test_loop = cfg.get<bool>("test.loop", true);
    const bool want_record = cfg.get<bool>("dataset.record_enabled", false);

    HikDriver camera;
    string target_sn = cfg.get<string>("camera.camera_sn", "DA4568803");
    unique_ptr<SerialDriver> serial;

    if (is_hik) {
        string port = cfg.get<string>("hardware.serial_port", "");
        serial = std::make_unique<SerialDriver>(port.c_str());
        camera.set_isp_from_config(cfg.get<double>("camera.exposure_time_us", -1.0),
                                   cfg.get<double>("camera.gain", -1.0));
        if (!camera.connect(target_sn))
            return -1;
        std::cout << "[run] 模式 hik：海康实时 + 串口解算";
        if (want_record)
            std::cout << " + 录制";
        std::cout << "\n";
    } else {
        if (test_video.empty()) {
            std::cerr << "[run] test 模式需设置 test.video_path\n";
            return -1;
        }
        if (!std::filesystem::exists(test_video)) {
            std::cerr << "[run] 视频不存在: " << test_video << "\n";
            return -1;
        }
        if (want_record) {
            std::cerr << "[run] test 模式忽略 dataset.record_enabled（不录制）\n";
        }
        std::cout << "[run] 模式 test：MP4 回放 + YOLO 画框（无相机/串口）\n";
    }

    Detector pikachu_ai;
    Solver math_solver;
    Visualizer drawer;

    thread t_serial;
    thread t_capture;
    if (is_hik)
        t_serial = thread(serial_task, serial.get());
    if (is_hik)
        t_capture = thread(capture_task, &camera, target_sn);
    else
        t_capture = thread(video_playback_task, test_video, test_loop);

    Mat local_frame;
    uint64_t local_timestamp; // 修改：本循环处理的图像时间戳
    Mat last_calib_frame;
    const string record_dir = cfg.get<string>("dataset.record_dir", "dataset/match");
    const bool record_enabled = is_hik && want_record;
    const double record_max_fps = cfg.get<double>("dataset.record_max_fps", 15.0);
    const double record_min_interval_s =
        (record_max_fps > 0.0) ? (1.0 / record_max_fps) : 0.0;
    const double record_fps_meta = (record_max_fps > 0.0) ? record_max_fps : 30.0;
    string record_video_path;
    thread t_record;
    auto last_record_push = std::chrono::steady_clock::time_point{};
    if (record_enabled) {
        std::filesystem::create_directories(record_dir);
        const string stamp = make_record_timestamp();
        record_video_path = make_record_video_path(record_dir, stamp);
        t_record = thread(record_video_writer_task, record_video_path, record_fps_meta);
        std::cout << "[dataset] 比赛录制（YOLO 输入）-> " << record_video_path << "\n";
    }
    bool have_calib_snap = false;
    float calib_pnp_tx = 0.f, calib_pnp_ty = 0.f, calib_pnp_tz = 0.f;
    SendPacket tx_pkt;

    namedWindow("Pikachu View", WINDOW_NORMAL);
    resizeWindow("Pikachu View", 800, 600);

    while (is_running) {
        if (has_new_frame) {
            frame_mtx.lock();
            shared_frame.copyTo(local_frame);
            local_timestamp = shared_timestamp; // 修改：拷贝对应的采集时间
            has_new_frame = false;
            frame_mtx.unlock();

            if (record_enabled) {
                const auto now_tick = std::chrono::steady_clock::now();
                const bool throttle_ok =
                    record_min_interval_s <= 0.0 ||
                    std::chrono::duration<double>(now_tick - last_record_push).count() >=
                        record_min_interval_s;
                if (throttle_ok) {
                    Mat rec_frame;
                    local_frame.copyTo(rec_frame);
                    {
                        lock_guard<mutex> lock(record_mtx);
                        if (record_queue.size() >= kRecordQueueMax) {
                            record_queue.pop_front();
                            record_drops.fetch_add(1);
                        }
                        record_queue.push_back(std::move(rec_frame));
                    }
                    record_cv.notify_one();
                    last_record_push = now_tick;
                }
            }

            // --- 修改部分：时间戳对齐逻辑 ---
            float matched_yaw = 0.000f;
            float matched_pitch = 0.000f;
            float matched_roll = 0.000f;
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

            if (!find_matched && is_hik)
                continue;

            vector<DetectResult> results = pikachu_ai.run_yolo(local_frame);

            if (is_test) {
                if (!results.empty()) {
                    const DetectResult* best = &results[0];
                    for (size_t i = 1; i < results.size(); ++i) {
                        if (results[i].score > best->score)
                            best = &results[i];
                    }
                    draw_yolo_only_preview(local_frame, *best);
                }
            } else if (results.empty()) {
                math_solver.reset_filter();
                have_calib_snap = false;
                tx_pkt.mode = 0;
                if (is_hik)
                    serial->send_packet(tx_pkt);
            } else {
                const DetectResult* best = &results[0];
                for (size_t i = 1; i < results.size(); ++i) {
                    if (results[i].score > best->score)
                        best = &results[i];
                }
                DetectResult track = *best;

                GimbalCmd cmd =
                    math_solver.solve(track, matched_yaw, matched_pitch, matched_roll, local_timestamp);

                drawer.draw_results(local_frame, math_solver.camera_matrix, math_solver.dist_coeffs, track, cmd,
                                    matched_yaw, matched_pitch, matched_roll);
                drawer.draw_laser_dot(local_frame, math_solver.camera_matrix, math_solver.dist_coeffs,
                                      math_solver.cam_offset, math_solver.ray_offset, math_solver.R_cam_to_ray,
                                      cmd.pnp_tx, cmd.pnp_ty, cmd.pnp_tz);

                local_frame.copyTo(last_calib_frame);
                calib_pnp_tx = cmd.pnp_tx;
                calib_pnp_ty = cmd.pnp_ty;
                calib_pnp_tz = cmd.pnp_tz;
                have_calib_snap = true;

                tx_pkt.mode = 1;
                tx_pkt.pitch = cmd.target_pitch;
                tx_pkt.yaw = cmd.target_yaw;
                tx_pkt.distance = cmd.p_world_x;
                if (is_hik)
                    serial->send_packet(tx_pkt);
            }

            drawer.draw_display_fps(local_frame);

            imshow("Pikachu View", local_frame);
        }

        int key = waitKey(1) & 0xFF;
        if (key == 27) {
            is_running = false;
            break;
        }
        // W：相机系→激光系 rpy 两点标定（仅 hik）；结果打印到终端，手动写入 config offset.rpy_cam_to_ray
        if ((key == 'w' || key == 'W') && is_hik) {
            if (!have_calib_snap || calib_pnp_tz <= 1e-4f) {
                std::cerr << "[rpy_calib] 无有效帧：请先稳定跟踪目标（出 LASER_REF）后再按 W。\n";
            } else {
                RpyCamToRayInput cinp;
                last_calib_frame.copyTo(cinp.frame_bgr);  // 带检测叠加的最后一帧，供弹窗点选
                cinp.camera_matrix = math_solver.camera_matrix.clone();
                cinp.dist_coeffs = math_solver.dist_coeffs.clone();
                cinp.cam_offset = math_solver.cam_offset;
                cinp.ray_offset = math_solver.ray_offset;
                cinp.R_cam_to_ray = math_solver.R_cam_to_ray;  // 当前配置，仅作对照
                cinp.pnp_tx = calib_pnp_tx;  // 最近一次有效 PnP，标定假定靶面深度 tz 可信
                cinp.pnp_ty = calib_pnp_ty;
                cinp.pnp_tz = calib_pnp_tz;
                run_rpy_calib_two_clicks(cinp);  // 依次点：模拟激光中心 → 实际击打点
            }
        }
    }

    if (record_enabled) {
        record_cv.notify_all();
        if (t_record.joinable())
            t_record.join();
    }
    if (t_capture.joinable())
        t_capture.join();
    if (t_serial.joinable())
        t_serial.join();
    if (is_hik)
        camera.close_camera();

    return 0;
}
