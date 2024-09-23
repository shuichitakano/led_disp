/*
 * author : Shuichi TAKANO
 * since  : Sat Sep 14 2024 17:09:10
 */
#pragma once

#include <cstdint>

enum class VideoCaptureRequest : uint8_t
{
    NONE = 0,
    START_CAPTURE = 1,
    STOP_CAPTURE = 2,
    ANALYZE_SIGNAL = 3,
    CAPTURE_FRAME = 4,
    WAIT_CAPTURE = 5,
};
