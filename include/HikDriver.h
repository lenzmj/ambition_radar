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
    
    // 修改：增加 uint64_t& timestamp 参数，用于回传图像到达内存的确切时间戳
    bool get_frame(cv::Mat& output_img, uint64_t& timestamp);
    
    void close_camera();

private:
    void* handle;
    bool is_connected;
    int convert_to_mat(MV_FRAME_OUT_INFO_EX* img_info, unsigned char* data_ptr, cv::Mat& dst_img);
};

#endif

