#include "mock_mv_camera.h"
#include <cstdlib>
#include <cstring>

// --- Internal mock state ---
static unsigned int g_device_count = 1;
static int g_create_handle_result = MV_OK;
static int g_open_device_result = MV_OK;
static int g_get_image_result = MV_OK;
static unsigned int g_frame_width = 640;
static unsigned int g_frame_height = 480;

static MV_CC_DEVICE_INFO g_mock_device_info = {1, 0};
static unsigned char* g_fake_frame_data = nullptr;

// --- Mock control functions ---

void mock_set_device_count(unsigned int count) {
    g_device_count = count;
}

void mock_set_create_handle_result(int result) {
    g_create_handle_result = result;
}

void mock_set_open_device_result(int result) {
    g_open_device_result = result;
}

void mock_set_get_image_result(int result) {
    g_get_image_result = result;
}

void mock_set_frame_size(unsigned int width, unsigned int height) {
    g_frame_width = width;
    g_frame_height = height;
}

void mock_reset() {
    g_device_count = 1;
    g_create_handle_result = MV_OK;
    g_open_device_result = MV_OK;
    g_get_image_result = MV_OK;
    g_frame_width = 640;
    g_frame_height = 480;
    if (g_fake_frame_data) {
        free(g_fake_frame_data);
        g_fake_frame_data = nullptr;
    }
}

// --- Mock SDK function implementations ---

int MV_CC_EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList) {
    (void)nTLayerType;
    pstDevList->nDeviceNum = g_device_count;
    if (g_device_count > 0) {
        pstDevList->pDeviceInfo[0] = &g_mock_device_info;
    }
    return MV_OK;
}

int MV_CC_CreateHandle(void** handle, MV_CC_DEVICE_INFO* pstDevInfo) {
    (void)pstDevInfo;
    if (g_create_handle_result != MV_OK) {
        *handle = nullptr;
        return g_create_handle_result;
    }
    // Allocate a dummy handle (non-null pointer)
    *handle = malloc(1);
    return MV_OK;
}

int MV_CC_OpenDevice(void* handle) {
    (void)handle;
    return g_open_device_result;
}

int MV_CC_StartGrabbing(void* handle) {
    (void)handle;
    return MV_OK;
}

int MV_CC_GetImageBuffer(void* handle, MV_FRAME_OUT* pFrame, unsigned int nMsec) {
    (void)handle;
    (void)nMsec;

    if (g_get_image_result != MV_OK) {
        return g_get_image_result;
    }

    // Allocate a fake BayerRG8 frame (single channel, 8-bit)
    size_t data_size = (size_t)g_frame_width * g_frame_height;
    if (g_fake_frame_data) {
        free(g_fake_frame_data);
    }
    g_fake_frame_data = (unsigned char*)malloc(data_size);

    // Fill with a simple gradient pattern so the converted image is non-trivial
    for (size_t i = 0; i < data_size; i++) {
        g_fake_frame_data[i] = (unsigned char)(i % 256);
    }

    pFrame->stFrameInfo.nWidth = g_frame_width;
    pFrame->stFrameInfo.nHeight = g_frame_height;
    pFrame->stFrameInfo.enPixelType = PixelType_Gvsp_BayerRG8;
    pFrame->pBufAddr = g_fake_frame_data;

    return MV_OK;
}

int MV_CC_FreeImageBuffer(void* handle, MV_FRAME_OUT* pFrame) {
    (void)handle;
    (void)pFrame;
    return MV_OK;
}

int MV_CC_StopGrabbing(void* handle) {
    (void)handle;
    return MV_OK;
}

int MV_CC_CloseDevice(void* handle) {
    (void)handle;
    return MV_OK;
}

int MV_CC_DestroyHandle(void* handle) {
    if (handle) {
        free(handle);
    }
    return MV_OK;
}
