#ifndef HIK_DRIVER_H
#define HIK_DRIVER_H

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

class HikDriver {
public:
    HikDriver();
    ~HikDriver();

    // 连接相机
    bool connect();
    // 获取一帧图片
    bool get_frame(cv::Mat& output_img);
    // 断开
    void close_camera();

private:
    void* handle;           // 相机句柄(指针)
    bool is_connected;      // 是否连接成功的标志
    
    // 把相机数据转成OpenCV格式
    int convert_to_mat(MV_FRAME_OUT_INFO_EX* img_info, unsigned char* data_ptr, cv::Mat& dst_img);
};

#endif
