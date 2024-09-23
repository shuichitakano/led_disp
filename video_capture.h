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

class MasterParameterCommunicator;

namespace ui
{
    class Menu;
}

namespace graphics
{
    class TextPlane;
}

namespace video
{
    class VideoCaptureHandler
    {
    public:
        virtual ~VideoCaptureHandler() = default;

        virtual void startBGCapture() = 0;
        virtual void stopBGCapture() = 0;
        virtual bool analyzeSignal() = 0;
        virtual void startCaptureFrame() = 0;
        virtual void endCaptureFrame() = 0;
    };

    class VideoCaptureController
    {
        device::ADV7181 *adv7181_{};
        VideoCaptureHandler *handler_{};

        bool signalDetected_ = false;
        device::ADV7181::STDIState currentSTDIState_{};

        bool isCSync_ = false;
        // bool isCSync_ = true;

        bool captureBegin_ = false;

        int id_ = 0;

    public:
        void setADV7181(device::ADV7181 *p) { adv7181_ = p; }
        void setHandler(VideoCaptureHandler *p) { handler_ = p; }
        void setID(int id) { id_ = id; }

        bool tick();
        void wait();
        void resetSignalDetection();
    };

    class VideoCapture
    {
        VideoStreamBuffer buffer_;

        struct FieldInfo
        {
            int preVSyncLines = 0;
            int activeLines = 0;
            int activeLines2 = 0;
            int activeLines3 = 0;
            int postVSyncLines = 0;
            int postVSyncLines2 = 0;

            int getTotalActiveLines() const { return activeLines + activeLines2 + activeLines3; }
        };

        int activeWidth_ = 0;
        int dataWidthInWords_ = 0;
        int hBlankSizeInBytes_ = 0;
        std::array<FieldInfo, 2> fieldInfo_;
        uint32_t vSyncIntervalCycles_ = 0;

        bool isFreeRun_ = false;

        uint32_t lineIntervalCycles128_ = 1;
        uint32_t frameIntervalCycles_ = 1;
        uint32_t frameIntervalLines_ = 1;

        int activeLineOfs_ = 0; // V先頭からの表示開始ライン数
        int startLine_ = 0;     // 表示開始ライン
        int activeLines_ = 0;   // 表示ライン数

        int16_t leftMargin_ = 0;
        int16_t rightMargin_ = 0;
        int16_t topBound_ = 0;
        int16_t bottomBound_ = 0;

        int currentField_ = 0;
        int deviatedFrames_ = 0; // V同期しなかった連続フレーム数

        volatile int currentCaptureLine_ = 0;
        uint32_t latestVSyncFallTime_ = 0;
        uint32_t latestVSyncRiseTime_ = 0;
        uint32_t latestVInterval_ = 0;

        uint32_t baseTime_ = 0;

        uint8_t blackLevel_ = 0x60; // これ以下で黒
        uint8_t isCSync_ = false;
        // uint8_t isCSync_ = true;

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

        bool bgCaptureActive_ = false;

        int cursorLine_ = -1;
        int dumpLineReq_ = -1;
        bool logReq_ = false;

        static inline constexpr int DEFAULT_HDIV = 858;
        int hdiv_ = DEFAULT_HDIV; // PLL サンプル数
        int resampleWidth_ = 640; // 画面幅にリサンプルする範囲
        int resampleWidthFrac_ = 0;
        int resampleOfs_ = 40;    // リサンプルする時のサンプルオフセット
        int resampleOfsFrac_ = 0; // リサンプル時のサブオフセット
        int vOfs_ = 0;            // V方向オフセット

        // device::ADV7181 *adv7181_{};
        // device::ADV7181::STDIState currentSTDIState_{};

        graphics::FrameBuffer *frameBuffer_{};
        graphics::TextPlane *textPlane_{};

    public:
        void setFrameBuffer(graphics::FrameBuffer *p) { frameBuffer_ = p; }
        void setTextPlane(graphics::TextPlane *p) { textPlane_ = p; }

        void __not_in_flash_func(captureFrame)();

        void resetSignalDetection();

        bool analyzeSignal();
        void __not_in_flash_func(measureInterval)();

        void simpleCaptureTest();
        void imageConvertTest();

        bool __not_in_flash_func(irq)(uint32_t time);
        void __not_in_flash_func(vsyncIRQ)(bool rise, uint32_t time);

        void requestDumpLine(int i) { dumpLineReq_ = i; }
        void requestLog() { logReq_ = true; }

        void setCursorLine(int i) { cursorLine_ = i; }
        int getHDiv() const { return hdiv_; }
        // void setHDiv(int v);

        uint32_t getFieldIntervalCycles() const { return frameIntervalCycles_ >> 1; }

        int getLeftMargin() const { return leftMargin_; }
        int getRightMargin() const { return rightMargin_; }
        int getTopBound() const { return topBound_; }
        int getBottomBound() const { return bottomBound_; }

        void initMenu(ui::Menu &menu);

        void startBGCapture();
        void stopBGCapture();

        void setResampleParams(int w, int wf, int ofs, int of)
        {
            resampleWidth_ = w;
            resampleWidthFrac_ = wf;
            resampleOfs_ = ofs;
            resampleOfsFrac_ = of;
        }

    private:
        bool __not_in_flash_func(analyzeSignalVC)();
        bool __not_in_flash_func(analyzeSignalHSVS)();

        void __not_in_flash_func(startCaptureLine)(BT656TimingCode sav, uint32_t time);
        void __not_in_flash_func(waitVSync)() const;

        void __not_in_flash_func(captureFrameHSVS)(graphics::FrameBuffer &frameBuffer);
        void __not_in_flash_func(captureFrameVC)(graphics::FrameBuffer &frameBuffer);

        void __not_in_flash_func(updateBound)(const uint32_t *data, int line);
    };

    class MasterVideoCaptureHandler : public VideoCaptureHandler
    {
        VideoCapture *capture_;

    public:
        MasterVideoCaptureHandler(VideoCapture *p) : capture_(p) {}

        void startBGCapture() override { capture_->startBGCapture(); }
        void stopBGCapture() override { capture_->stopBGCapture(); }
        bool analyzeSignal() override { return capture_->analyzeSignal(); }
        void startCaptureFrame() override {}
        void endCaptureFrame() override { capture_->captureFrame(); }
    };

    class RemoteVideoCaptureHandler : public VideoCaptureHandler
    {
        MasterParameterCommunicator *com_{};

    public:
        RemoteVideoCaptureHandler(MasterParameterCommunicator *com) : com_(com) {}

        void startBGCapture() override;
        void stopBGCapture() override;
        bool analyzeSignal() override;
        void startCaptureFrame() override;
        void endCaptureFrame() override;
    };
}