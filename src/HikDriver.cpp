#include "HikDriver.h"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

HikDriver::HikDriver() {
    handle = NULL;
    is_connected = false;
}

HikDriver::~HikDriver() {
    close_camera();
}

bool HikDriver::connect() {
    int res = MV_OK;
    MV_CC_DEVICE_INFO_LIST device_list;

    // 枚举所有USB和网口相机
    res = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    if (device_list.nDeviceNum == 0) {
        cout << "[Driver] No camera found." << endl;
        return false;
    }

    // 创建句柄
    res = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[0]);
    if (res != MV_OK) return false;

    // 打开
    res = MV_CC_OpenDevice(handle);
    if (res != MV_OK) return false;

    // 开始取流
    MV_CC_StartGrabbing(handle);
    
    is_connected = true;
    cout << "[Driver] Camera connected." << endl;
    return true;
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
    // 处理 BayerRG8 
    if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
        Mat bayer(info->nHeight, info->nWidth, CV_8UC1, data);
        
        cvtColor(bayer, dst, COLOR_BayerRG2RGB); 
    }

    return 0;
}

bool HikDriver::get_frame(Mat& output_img) {
    if (!is_connected) return false;

    MV_FRAME_OUT out_frame = {0};
    int res = MV_CC_GetImageBuffer(handle, &out_frame, 1000); // 1000ms超时

    if (res == MV_OK) {
        convert_to_mat(&out_frame.stFrameInfo, out_frame.pBufAddr, output_img);
        MV_CC_FreeImageBuffer(handle, &out_frame);
        return true;
    }
    return false;
}
