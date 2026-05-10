#include "HikDriver.h"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

HikDriver::HikDriver() {
    handle = NULL;
    is_connected = false;
    exposure_time_us_ = -1.0;
    gain_db_ = -1.0;
}

HikDriver::~HikDriver() {
    close_camera();
}

bool HikDriver::connect() {
    int res = MV_OK;
    MV_CC_DEVICE_INFO_LIST device_list;
    res = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    if (device_list.nDeviceNum == 0) return false;

    res = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[0]);
    if (res != MV_OK) return false;

    res = MV_CC_OpenDevice(handle);
    if (res != MV_OK) return false;

    MV_CC_StartGrabbing(handle);
    is_connected = true;
    apply_isp_settings();
    return true;
}

void HikDriver::set_isp_from_config(double exposure_time_us, double gain_db) {
    exposure_time_us_ = exposure_time_us;
    gain_db_ = gain_db;
    apply_isp_settings();
}

void HikDriver::apply_isp_settings() {
    if (!is_connected || handle == NULL)
        return;
    if (exposure_time_us_ >= 0.0) {
        int r = MV_CC_SetExposureAutoMode(handle, MV_EXPOSURE_AUTO_MODE_OFF);
        if (r != MV_OK)
            std::cerr << "[HikDriver] MV_CC_SetExposureAutoMode failed: " << r << std::endl;
        r = MV_CC_SetExposureTime(handle, static_cast<float>(exposure_time_us_));
        if (r != MV_OK)
            std::cerr << "[HikDriver] MV_CC_SetExposureTime failed: " << r << std::endl;
    }
    if (gain_db_ >= 0.0) {
        int r = MV_CC_SetGainMode(handle, MV_GAIN_MODE_OFF);
        if (r != MV_OK)
            std::cerr << "[HikDriver] MV_CC_SetGainMode failed: " << r << std::endl;
        r = MV_CC_SetGain(handle, static_cast<float>(gain_db_));
        if (r != MV_OK)
            std::cerr << "[HikDriver] MV_CC_SetGain failed: " << r << std::endl;
    }
}

void HikDriver::close_camera() {
    if (is_connected) {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = NULL;
        is_connected = false;
    }
}

int HikDriver::convert_to_mat(MV_FRAME_OUT_INFO_EX* info, unsigned char* data, Mat& dst) {
    if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
        Mat bayer(info->nHeight, info->nWidth, CV_8UC1, data);
        cvtColor(bayer, dst, COLOR_BayerRG2RGB); 
    }
    return 0;
}

// 修改：重写 get_frame 函数，在图像成功获取的时刻记录系统时间[cite: 2]
bool HikDriver::get_frame(Mat& output_img, uint64_t& timestamp) {
    if (!is_connected) return false;

    MV_FRAME_OUT out_frame = {0};
    // 1000ms超时。这里是图像从相机传输到PC内存的观测点[cite: 2]
    int res = MV_CC_GetImageBuffer(handle, &out_frame, 1000);

    if (res == MV_OK) {
        convert_to_mat(&out_frame.stFrameInfo, out_frame.pBufAddr, output_img);
        
        // 修改原因：获取 steady_clock 的当前毫秒数作为图像的时间坐标。
        // 这代表了这帧画面“被看到”的时刻，用于后续匹配云台角度[cite: 3]
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        MV_CC_FreeImageBuffer(handle, &out_frame);
        return true;
    }
    return false;
}
