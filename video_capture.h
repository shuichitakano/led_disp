/*
 * author : Shuichi TAKANO
 * since  : Sun Jun 19 2022 21:58:07
 */
#pragma once

#include <stdint.h>
#include <pico/stdlib.h>
#include <array>
#include "framebuffer.h"
#include "video_stream.h"
#include "video_stream_buffer.h"
#include "adv7181.h"

namespace video
{
    class VideoCapture
    {
        VideoStreamBuffer buffer_;

        struct FieldInfo
        {
            int preVSyncLines = 0;
            int activeLines = 0;
            int activeLines2 = 0;
            int postVSyncLines = 0;
        };

        bool signalDetected_ = false;
        int activeWidth_ = 0;
        int dataWidthInWords_ = 0;
        int hBlankSizeInBytes_ = 0;
        std::array<FieldInfo, 2> fieldInfo_;
        uint32_t vSyncIntervalCycles_ = 0;

        uint32_t lineIntervalCycles128_ = 0;
        uint32_t frameIntervalCycles_ = 0;
        uint32_t frameIntervalLines_ = 0;

        int activeLineOfs_ = 0; // V先頭からの表示開始ライン数
        int startLine_ = 0;     // 表示開始ライン
        int activeLines_ = 0;   // 表示ライン数

        int16_t leftMargin_ = 0;
        int16_t rightMargin_ = 0;
        int16_t topBound_ = 0;
        int16_t bottomBound_ = 0;

        int currentField_ = 0;
        int deviatedFrames_ = 0;    // V同期しなかった連続フレーム数
        int baseCycleCounter_ = -1; // V頭のサイクルカウンタ

        uint32_t baseTime_ = 0;

        uint8_t blackLevel_ = 0x60; // これ以下で黒
        uint8_t isCSync_ = false;

        static constexpr uint32_t SAV_ACTIVE_F0 = 0x800000ff;
        static constexpr uint32_t SAV_ACTIVE_F1 = 0xc70000ff;
        static constexpr uint32_t SAV_VSYNC_F0 = 0xab0000ff;
        static constexpr uint32_t SAV_VSYNC_F1 = 0xec0000ff;

        static constexpr uint32_t EAV_ACTIVE_F0 = 0x9d0000ff;
        static constexpr uint32_t EAV_ACTIVE_F1 = 0xda0000ff;
        static constexpr uint32_t EAV_VSYNC_F0 = 0xb60000ff;
        static constexpr uint32_t EAV_VSYNC_F1 = 0xf10000ff;

        static constexpr uint32_t MAX_ACTIVE_LINES = 400;
        static constexpr uint32_t MAX_VBLANK_LINES = 100;

        volatile bool breakIRQ_ = false;
        volatile bool runningIRQ_ = false;

        int cursorLine_ = -1;
        int dumpLineReq_ = -1;
        bool logReq_ = false;

        static inline constexpr int DEFAULT_HDIV = 858;
        int hdiv_ = DEFAULT_HDIV;

        device::ADV7181::STDIState currentSTDIState_{};

    public:
        bool __not_in_flash_func(tick)(graphics::FrameBuffer &frameBuffer,
                                       device::ADV7181 &adv7181);
        void __not_in_flash_func(captureFrame)(graphics::FrameBuffer &frameBuffer);
        void __not_in_flash_func(captureFrame2)(graphics::FrameBuffer &frameBuffer);

        void resetSignalDetection()
        {
            signalDetected_ = false;
        }

        bool __not_in_flash_func(analyzeSignal)();
        void __not_in_flash_func(measureInterval)();

        void simpleCaptureTest();
        void imageConvertTest();

        bool __not_in_flash_func(irq)(uint32_t time);

        void requestDumpLine(int i) { dumpLineReq_ = i; }
        void requestLog() { logReq_ = true; }

        void setCursorLine(int i) { cursorLine_ = i; }
        int getHDiv() const { return hdiv_; }
        void setHDiv(int v, device::ADV7181 &adv7181);

        uint32_t getFieldIntervalCycles() const { return frameIntervalCycles_ >> 1; }

        int getLeftMargin() const { return leftMargin_; }
        int getRightMargin() const { return rightMargin_; }
        int getTopBound() const { return topBound_; }
        int getBottomBound() const { return bottomBound_; }

    private:
        void __not_in_flash_func(startCaptureLine)(BT656TimingCode sav, uint32_t time);
        void startBGCapture();
        void stopBGCapture();

        void updateBound(const uint32_t *data, int line);
    };

}