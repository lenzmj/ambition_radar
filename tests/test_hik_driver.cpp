#include <gtest/gtest.h>
#include "HikDriver.h"
#include "mock_mv_camera.h"

// Test fixture that resets mock state before each test
class HikDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_reset();
    }

    void TearDown() override {
        mock_reset();
    }
};

// --- connect() tests ---

TEST_F(HikDriverTest, ConnectSucceedsWithOneDevice) {
    HikDriver driver;
    EXPECT_TRUE(driver.connect());
}

TEST_F(HikDriverTest, ConnectFailsWhenNoDevicesFound) {
    mock_set_device_count(0);
    HikDriver driver;
    EXPECT_FALSE(driver.connect());
}

TEST_F(HikDriverTest, ConnectFailsWhenCreateHandleFails) {
    mock_set_create_handle_result(-1);
    HikDriver driver;
    EXPECT_FALSE(driver.connect());
}

TEST_F(HikDriverTest, ConnectFailsWhenOpenDeviceFails) {
    mock_set_open_device_result(-1);
    HikDriver driver;
    EXPECT_FALSE(driver.connect());
}

// --- get_frame() tests ---

TEST_F(HikDriverTest, GetFrameFailsWhenNotConnected) {
    HikDriver driver;
    cv::Mat frame;
    EXPECT_FALSE(driver.get_frame(frame));
}

TEST_F(HikDriverTest, GetFrameSucceedsAfterConnect) {
    HikDriver driver;
    ASSERT_TRUE(driver.connect());

    cv::Mat frame;
    EXPECT_TRUE(driver.get_frame(frame));
    EXPECT_FALSE(frame.empty());
}

TEST_F(HikDriverTest, GetFrameReturnsCorrectDimensions) {
    mock_set_frame_size(1280, 1024);
    HikDriver driver;
    ASSERT_TRUE(driver.connect());

    cv::Mat frame;
    ASSERT_TRUE(driver.get_frame(frame));
    EXPECT_EQ(frame.rows, 1024);
    EXPECT_EQ(frame.cols, 1280);
}

TEST_F(HikDriverTest, GetFrameReturns3ChannelRGB) {
    HikDriver driver;
    ASSERT_TRUE(driver.connect());

    cv::Mat frame;
    ASSERT_TRUE(driver.get_frame(frame));
    // BayerRG8 is converted to 3-channel BGR/RGB by cvtColor
    EXPECT_EQ(frame.channels(), 3);
    EXPECT_EQ(frame.type(), CV_8UC3);
}

TEST_F(HikDriverTest, GetFrameFailsWhenImageBufferFails) {
    HikDriver driver;
    ASSERT_TRUE(driver.connect());

    mock_set_get_image_result(-1);
    cv::Mat frame;
    EXPECT_FALSE(driver.get_frame(frame));
}

// --- close_camera() tests ---

TEST_F(HikDriverTest, CloseCameraAllowsReconnect) {
    HikDriver driver;
    ASSERT_TRUE(driver.connect());
    driver.close_camera();

    // After closing, get_frame should fail (not connected)
    cv::Mat frame;
    EXPECT_FALSE(driver.get_frame(frame));
}

TEST_F(HikDriverTest, CloseCameraSafeWhenNotConnected) {
    HikDriver driver;
    // Should not crash when closing without connecting
    driver.close_camera();
}

// --- Full workflow regression test ---

TEST_F(HikDriverTest, FullWorkflowConnectGetFrameClose) {
    HikDriver driver;

    // Step 1: Connect
    ASSERT_TRUE(driver.connect());

    // Step 2: Retrieve multiple frames
    for (int i = 0; i < 5; i++) {
        cv::Mat frame;
        ASSERT_TRUE(driver.get_frame(frame));
        ASSERT_FALSE(frame.empty());
        ASSERT_EQ(frame.channels(), 3);
    }

    // Step 3: Close
    driver.close_camera();

    // Step 4: Verify get_frame fails after close
    cv::Mat frame;
    EXPECT_FALSE(driver.get_frame(frame));
}
