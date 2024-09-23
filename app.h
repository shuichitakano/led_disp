/*
 * author : Shuichi TAKANO
 * since  : Sun Jun 19 2022 22:05:45
 */
#pragma once

#include <stdint.h>

#if BOARDTYPE_SEPARATE_VIDEO
#define USE_VSYNC_PIN 1
#else
#define USE_VSYNC_PIN 0
#endif

void waitVideoCapture();
void startVideoCapture(uint32_t *dst,
                       uint32_t startCode, uint32_t transferWords);
void enableVideoCaptureIRQ(bool f);
bool getVSync();

#if BOARDTYPE_SEPARATE_VIDEO
static constexpr uint32_t CPU_CLOCK_KHZ = 160000;
#else
static constexpr uint32_t CPU_CLOCK_KHZ = 266000;
#endif
// static constexpr uint32_t CPU_CLOCK_KHZ = 250000;

static constexpr int UNIT_WIDTH = 16;
static constexpr int N_CASCADE = 10; // 16 * 10
static constexpr int N_SCAN_LINES = 60;
static constexpr int BPP = 16;
static constexpr int PWM_PULSE_PER_LINE = 74;

static constexpr int DISPLAY_WIDTH = 320;
static constexpr int DISPLAY_HEIGHT = 240;