/*
 * author : Shuichi TAKANO
 * since  : Sun Jun 19 2022 21:58:07
 */
#pragma once

#include <stdint.h>
#include <pico/stdlib.h>
#include "framebuffer.h"

struct VideoCapture
{
    static constexpr uint32_t UNIT_BUFFER_SIZE = 512;
    static constexpr uint32_t TOTAL_BUFFER_SIZE = UNIT_BUFFER_SIZE * 2;

    uint32_t buffer_[TOTAL_BUFFER_SIZE];

    bool signalDetected_ = false;
    int activeWidth_ = 0;
    int dataWidthInWords_ = 0;
    int hBlankSizeInBytes_ = 0;
    int activeLines_ = 0;
    int vBlankLines_ = 0;
    bool interlace_ = false;
    uint32_t vSyncIntervalCycles_ = 0;
    int activeLineOfs_ = 0;
    int leftMargin_ = 40;
    int rightMargin_ = 40;

    int currentField_ = 0;

    static constexpr uint32_t SAV_ACTIVE_F0 = 0x800000ff;
    static constexpr uint32_t SAV_ACTIVE_F1 = 0xc70000ff;
    static constexpr uint32_t SAV_VSYNC_F0 = 0xab0000ff;
    static constexpr uint32_t SAV_VSYNC_F1 = 0xec0000ff;

    static constexpr uint32_t EAV_ACTIVE_F0 = 0x9d0000ff;
    static constexpr uint32_t EAV_ACTIVE_F1 = 0xda0000ff;
    static constexpr uint32_t EAV_VSYNC_F0 = 0xb60000ff;
    static constexpr uint32_t EAV_VSYNC_F1 = 0xf10000ff;

    static constexpr uint32_t MAX_ACTIVE_LINES = 720;
    static constexpr uint32_t MAX_VBLANK_LINES = 304;

public:
    bool __not_in_flash_func(tick)(graphics::FrameBuffer &frameBuffer);
    void __not_in_flash_func(captureFrame)(graphics::FrameBuffer &frameBuffer);

    uint32_t *__not_in_flash_func(getBuffer)(int dbid)
    {
        return &buffer_[dbid * UNIT_BUFFER_SIZE];
    }

    bool __not_in_flash_func(analyzeSignal)();

    void simpleCaptureTest();
    void imageConvertTest();
};
