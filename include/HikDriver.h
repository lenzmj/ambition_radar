#ifndef HIK_DRIVER_H
#define HIK_DRIVER_H

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <chrono> // 修改：引入 chrono 库以支持高精度系统时间

class HikDriver {
public:
    HikDriver();
    ~HikDriver();

    bool connect();

    /** 曝光时间 (µs)、增益 (dB)。值为负数表示不修改该项（保持相机当前/自动模式）。 */
    void set_isp_from_config(double exposure_time_us, double gain_db);
    
    // 修改：增加 uint64_t& timestamp 参数，用于回传图像到达内存的确切时间戳
    bool get_frame(cv::Mat& output_img, uint64_t& timestamp);
    
    void close_camera();

private:
    void apply_isp_settings();
    void* handle;
    bool is_connected;
    double exposure_time_us_;
    double gain_db_;
    int convert_to_mat(MV_FRAME_OUT_INFO_EX* img_info, unsigned char* data_ptr, cv::Mat& dst_img);
};

#endif

