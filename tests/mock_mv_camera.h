#ifndef MOCK_MV_CAMERA_CONTROL_H
#define MOCK_MV_CAMERA_CONTROL_H

// Mock definitions for the Hikvision MVS SDK types and constants
// used by HikDriver.cpp

#define MV_OK 0
#define MV_USB_DEVICE 0x01
#define MV_GIGE_DEVICE 0x02

// Pixel type enum
enum MvGvspPixelType {
    PixelType_Gvsp_BayerRG8 = 0x01080009
};

// Device info struct (minimal mock)
typedef struct _MV_CC_DEVICE_INFO {
    unsigned int nMajorVer;
    unsigned int nMinorVer;
} MV_CC_DEVICE_INFO;

// Device info list
typedef struct _MV_CC_DEVICE_INFO_LIST {
    unsigned int nDeviceNum;
    MV_CC_DEVICE_INFO* pDeviceInfo[256];
} MV_CC_DEVICE_INFO_LIST;

// Frame output info
typedef struct _MV_FRAME_OUT_INFO_EX {
    unsigned int nWidth;
    unsigned int nHeight;
    MvGvspPixelType enPixelType;
} MV_FRAME_OUT_INFO_EX;

// Frame output
typedef struct _MV_FRAME_OUT {
    MV_FRAME_OUT_INFO_EX stFrameInfo;
    unsigned char* pBufAddr;
} MV_FRAME_OUT;

// --- Mock control interface ---
// These functions allow tests to configure mock behavior before calling HikDriver methods.

// Set how many devices MV_CC_EnumDevices should report (default: 1)
void mock_set_device_count(unsigned int count);

// Set the return value for MV_CC_CreateHandle (default: MV_OK)
void mock_set_create_handle_result(int result);

// Set the return value for MV_CC_OpenDevice (default: MV_OK)
void mock_set_open_device_result(int result);

// Set the return value for MV_CC_GetImageBuffer (default: MV_OK)
void mock_set_get_image_result(int result);

// Set the frame dimensions for the mock image (default: 640x480)
void mock_set_frame_size(unsigned int width, unsigned int height);

// Reset all mock state to defaults
void mock_reset();

// --- SDK function declarations ---
#ifdef __cplusplus
extern "C" {
#endif

int MV_CC_EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList);
int MV_CC_CreateHandle(void** handle, MV_CC_DEVICE_INFO* pstDevInfo);
int MV_CC_OpenDevice(void* handle);
int MV_CC_StartGrabbing(void* handle);
int MV_CC_GetImageBuffer(void* handle, MV_FRAME_OUT* pFrame, unsigned int nMsec);
int MV_CC_FreeImageBuffer(void* handle, MV_FRAME_OUT* pFrame);
int MV_CC_StopGrabbing(void* handle);
int MV_CC_CloseDevice(void* handle);
int MV_CC_DestroyHandle(void* handle);

#ifdef __cplusplus
}
#endif

#endif // MOCK_MV_CAMERA_CONTROL_H
