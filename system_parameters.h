/*
 * author : Shuichi TAKANO
 * since  : Mon Sep 09 2024 05:50:14
 */
#pragma once

#include <cstdint>
#include <array>
#include "device_def.h"

struct LEDDisplayParameters
{
    std::array<uint16_t, 2> imageSize{320, 240};
    std::array<uint16_t, 2> displayOffset{0, 0};
};

struct VideoCaptureParameters
{
    uint16_t samplingOffset = 0;
    uint16_t samplingWidth = 640;
    uint8_t samplingOffsetFrac = 0;
    uint8_t samplingWidthFrac = 0;
    float fieldInterval = 0.0f;
};

struct VideoStatus
{
    enum class Flags : uint8_t
    {
        SIGNAL_ENABLED = 1 << 0,
        INTERLACED = 1 << 1,
    };

    Flags flags{};
};

struct SystemParameters
{
    LEDDisplayParameters ledDisplay;
    VideoCaptureParameters videoCapture;
    VideoStatus videoStatus;
    device::SignalInput signalInput;
};

// システムパラメータID
// 値を変更しないこと
enum class SystemParmeterID : uint8_t
{
    ledDisplay_imageSize = 0,
    ledDisplay_displayOffset = 1,
    videoCapture_samplingOffset = 2,
    videoCapture_samplingWidth = 3,
    videoCapture_samplingOffsetFrac = 4,
    videoCapture_samplingWidthFrac = 5,
    videoStatus_flags = 6,
    root_signalInput = 7,
};

class SystemParameterHolder
{
public:
private:
    SystemParameters params_;
    SystemParameters prev_{};
};
