/*
 * author : Shuichi TAKANO
 * since  : Sun Jun 19 2022 22:11:16
 */

#include "video_capture.h"

#include <hardware/divider.h>
#include <stdio.h>
#include <string.h>
#include <tuple>
#include <limits>
#include <utility>
#include <cmath>
#include "image_proc.h"
#include "util.h"
#include "menu.h"
#include "nv_settings.h"
#include "app.h"
#include "parameter_communicator.h"
#include "textplane.h"

#define ENABLE_CURSOR 0

namespace video
{

    bool
    VideoCaptureController::tick()
    {
        assert(adv7181_);
        assert(handler_);

        if (signalDetected_ &&
            (adv7181_->isSDPMode() ||
             test(currentSTDIState_, adv7181_->getSTDIState(), 10)))
        {
            handler_->startCaptureFrame();
            // adv7181_->setChGain(true, 200, 200, 200);
            // adv7181_->setChOffset(0, 0, 0);
            captureBegin_ = true;
            return true;
        }
        else
        {
            handler_->stopBGCapture();
            DBGPRINT("%d: detecting signal...\n", id_);

            auto waitForCounterStable = [&]
            {
                constexpr int timeout = 100; // ms
                if (adv7181_->getCurrentInput() == device::SignalInput::RGB21)
                {
                    // for (auto csync : {true, false})
                    auto csync = isCSync_;
                    {
                        adv7181_->setRGBSyncModeCSync(csync);
                        if (adv7181_->waitForCounterStable(timeout))
                        {
                            DBGPRINT("%d: csync = %d\n", id_, csync);
                            // adv7181_->setChGain(true, 200, 200, 200);
                            // adv7181_->setChOffset(0, 0, 0);

                            return true;
                        }
                    }
                }
                else
                {
                    return adv7181_->waitForCounterStable(timeout);
                }
                return false;
            };

            if (adv7181_->isSDPMode() || waitForCounterStable())
            {
                currentSTDIState_ = adv7181_->getSTDIState();
                currentSTDIState_.dump();

                if (currentSTDIState_.enabled)
                {
                    adv7181_->setLineLength(currentSTDIState_.blockSize / 8);
                }

                // sleep_ms(500);

                if (handler_->analyzeSignal())
                {
                    signalDetected_ = true;
#if !USE_VSYNC_PIN
                    handler_->measureInterval();
#endif

                    /*
                                        if (auto *p = NVSettings::instance().findCaptureSettings(currentSTDIState_))
                                        {
                                            DBGPRINT("setting found! %p\n", p);
                                            resampleWidth_ = p->resampleWidth;
                                            resampleOfs_ = p->resampleOfs;
                                            vOfs_ = p->vOfs;
                                        }
                                        else
                                        {
                                            DBGPRINT("setting not found.\n");
                                            resampleWidth_ = 640;
                                            resampleOfs_ = 40;
                                            vOfs_ = 0;
                                        }
                    */
                    handler_->startBGCapture();
                }
                else
                {
                    isCSync_ ^= true;
                }
            }
            else
            {
#if !USE_VSYNC_PIN
                isCSync_ ^= true;
#endif
                DBGPRINT("%d: video signal is not stabled.\n", id_);
            }
        }
        handler_->startCaptureFrame();
        return false;
    }

    void
    VideoCaptureController::wait()
    {
        // if (captureBegin_)
        {
            handler_->endCaptureFrame();
            adv7181_->updateStatus();
            captureBegin_ = false;
        }
    }

    void
    VideoCaptureController::resetSignalDetection()
    {
        signalDetected_ = false;
        adv7181_->clearStatusCache();
    }

    ////

#if 0
    void
    VideoCapture::setHDiv(int v)
    {
        hdiv_ = v;

        auto hfreq = (CPU_CLOCK_KHZ * 1000) / (lineIntervalCycles128_ >> 7);
        adv7181_->setPLL(true, false, v, hfreq);
    }
#endif

    void
    VideoCapture::startCaptureLine(BT656TimingCode sav, uint32_t time)
    {
        auto [line, linfo] = buffer_.getCurrentWriteLine();
        auto t = (baseTime_ - time) & 0xffffff;

        int captureSize = dataWidthInWords_;

#if USE_VSYNC_PIN
        linfo.time = t;
        linfo.lineIdx_x2 = currentCaptureLine_ << 1;
        linfo.error = false;

        if (currentCaptureLine_ > 240)
        {
            //            captureSize = 1;
        }

        ++currentCaptureLine_;
#else
        int l2 = hw_divider_u32_quotient_inlined((t << 8) + (lineIntervalCycles128_ >> 1),
                                                 lineIntervalCycles128_);
        // DBGPRINT("t %d, l2 %d\n", t, l2);

        linfo.code = makeBT656TimingCodeByte(sav);
        linfo.time = t;
        linfo.lineIdx_x2 = l2;
        linfo.error = false;

        int d2 = l2 - (frameIntervalLines_ << 1);
        if (d2 >= 0)
        {
            baseTime_ = time - d2;
        }
#endif

        startVideoCapture(line.data(), asUInt(sav), captureSize);
    }

    bool
    VideoCapture::irq(uint32_t time)
    {
        assert(runningIRQ_);
#if !USE_VSYNC_PIN
        auto [line, linfo] = buffer_.getCurrentWriteLine();
        auto [nextEAV, error] = video::findNextEAV(line.data(), activeWidth_ >> 1);
        linfo.error = error;
#endif

        buffer_.nextWrite();

        if (breakIRQ_)
        {
            runningIRQ_ = false;
            __sev();
            return false; // break
        }
        else
        {
#if USE_VSYNC_PIN
            startCaptureLine(BT656TimingCode::SAV_ACTIVE_F0, time);
#else
            auto nextSAV = video::getSAVCorrespondingToEAV(nextEAV);
            if (nextSAV == BT656TimingCode::INVALID)
            {
                nextSAV = BT656TimingCode::SAV_ACTIVE_F0;
            }
            startCaptureLine(nextSAV, time);
#endif
            // DBGPRINT("%p, l%d, %x, %x, %d\n", line.data(), linfo.lineIdx_x2, asUInt(nextEAV), asUInt(nextSAV), error);
        }
        return true; // continue
    }

    void
    VideoCapture::vsyncIRQ(bool rise, uint32_t time)
    {
        gpio_put(28, rise);

        if (rise)
        {
            latestVSyncRiseTime_ = time;
            currentCaptureLine_ = 0;
        }
        else
        {
            latestVInterval_ = (latestVSyncFallTime_ - time) & 0xffffff;
            latestVSyncFallTime_ = time;
            // printf("%d %d\n", latestVInterval_, currentCaptureLine_);
        }
    }

    void
    VideoCapture::stopBGCapture()
    {
        if (!runningIRQ_)
        {
            return;
        }

        breakIRQ_ = true;
        while (runningIRQ_)
        {
            __wfe();
        }

        bgCaptureActive_ = false;
    }

    void
    VideoCapture::waitVSync() const
    {
#if USE_VSYNC_PIN
        while (!getVSync())
        {
            tight_loop_contents();
        }
        while (getVSync())
        {
            tight_loop_contents();
        }
#else
        // auto *tmp = buffer_.getRawBuffer().data();
        uint32_t tmp;
        startVideoCapture(&tmp, asUInt(video::BT656TimingCode::EAV_VSYNC_F0), 1);
        waitVideoCapture();
        startVideoCapture(&tmp, asUInt(video::BT656TimingCode::EAV_ACTIVE_F0), 1);
        waitVideoCapture();
        // ACTIVE_F0 の先頭を基準とする
#endif
    }

    void
    VideoCapture::startBGCapture()
    {
        stopBGCapture();

        assert(!runningIRQ_);

        waitVSync();

        auto t = util::getSysTickCounter24();
        baseTime_ = t;

        runningIRQ_ = true;
        breakIRQ_ = false;
        enableVideoCaptureIRQ(true);

        bgCaptureActive_ = true;
        startCaptureLine(video::BT656TimingCode::SAV_ACTIVE_F0, t);
    }

    void
    VideoCapture::updateBound(const uint32_t *data, int line)
    {
        const auto *p = reinterpret_cast<const uint8_t *>(data);
        auto isBlack = [this](int v)
        {
            return v <= blackLevel_;
        };

        auto getLuminance = [&](int x)
        {
            return p[x * 2 + 1];
        };

        // if (line > topBound_ && line < bottomBound_)
        if (line > 64 && line < 176)
        {

            if (leftMargin_ > 0 && !isBlack(getLuminance(leftMargin_ - 1)))
            {
                --leftMargin_;
            }
            if (rightMargin_ > 0 && !isBlack(getLuminance(720 - rightMargin_)))
            {
                --rightMargin_;
            }
        }

        auto hasVideo = !isBlack(getLuminance(360));
        if (line < topBound_ && hasVideo)
        {
            --topBound_;
        }
        if (line > bottomBound_ && hasVideo)
        {
            ++bottomBound_;
        }
        // todo: limitつけたほうがいい
    }

    void
    VideoCapture::captureFrame()
    {
        assert(frameBuffer_);
        if (!bgCaptureActive_)
        {
            frameBuffer_->fillBlackPlane();
            return;
        }

#if USE_VSYNC_PIN
        captureFrameHSVS(*frameBuffer_);
#else
        captureFrameVC(*frameBuffer_);
#endif
    }

    void
    VideoCapture::captureFrameHSVS(graphics::FrameBuffer &frameBuffer)
    {
        int activeLines = std::min(DISPLAY_HEIGHT, activeLines_);
        int y = 0;

        // DBGPRINT("w %d\n", frameBuffer.getWritePlaneID());

        auto sampleStep16 = ((resampleWidth_ << 16) + (resampleWidthFrac_ << 12)) * 2 / 320;
        auto sampleOfs16 = resampleOfs_ * sampleStep16 + (resampleOfsFrac_ * sampleStep16 >> 4);

        int tt0,
            tt1, tt2, tt3;
        while (y < activeLines)
        {
            auto t00 = util::getSysTickCounter24();
            buffer_.waitForNextLine();
            auto t22 = util::getSysTickCounter24();

            auto [line, linfo] = buffer_.getCurrentReadLine();

            const auto lx2 = linfo.lineIdx_x2;
            const auto l = (lx2 >> 1) - 10;
            // const auto l = (lx2 >> 1);
            //            if (l < y)
            if (l != y)
            {
                continue;
            }

            if (y < activeLines)
            {
                auto freeLines = frameBuffer.getFreeLineCount();
                int dstLineID = frameBuffer.allocateLine();
                auto dstBuffer = frameBuffer.getLineBuffer(dstLineID);

                uint32_t resized[DISPLAY_WIDTH]; // 1280byte stack size注意
                auto t0 = util::getSysTickCounter24();
                graphics::resizeSimple(resized, line.data(), DISPLAY_WIDTH, sampleStep16, sampleOfs16);
                // graphics::resizeSimple(resized, line.data(), DISPLAY_WIDTH, int(4.0f * 65536), 0);
                auto t1 = util::getSysTickCounter24();
                graphics::convertXRGB8888toRGB565(dstBuffer, resized, DISPLAY_WIDTH);
                auto t2 = util::getSysTickCounter24();
                // memset(dstBuffer, 255, DISPLAY_WIDTH * 2);
                if (textPlane_)
                {
                    textPlane_->setupComposite();
                    textPlane_->composite(dstBuffer, y);
                }
                auto t3 = util::getSysTickCounter24();

                tt0 = (t0 - t1) & 0xffffff;
                tt1 = (t1 - t2) & 0xffffff;
                tt2 = (t2 - t3) & 0xffffff;

                frameBuffer.commitNextLine(dstLineID);
                ++y;
            }

            auto t11 = util::getSysTickCounter24();
            tt3 = (t00 - t22) & 0xffffff;
        }

        for (auto ct = DISPLAY_HEIGHT - y; ct > 0; --ct)
        {
            int dstLineID = frameBuffer.allocateLine();
            auto p = frameBuffer.getLineBuffer(dstLineID);
            memset(p, 0, DISPLAY_WIDTH * 2);
            frameBuffer.commitNextLine(dstLineID);
            ++y;
        }

        assert(y == DISPLAY_HEIGHT);

        // DBGPRINT("%d %d %d: %d\n", tt0, tt1, tt2, tt3);
    }

    void
    VideoCapture::captureFrameVC(graphics::FrameBuffer &frameBuffer)
    {
        // buffer_.reset();

        int halfFrameInterval_x2 = frameIntervalLines_;
        int frameInterval_x2 = frameIntervalLines_ << 1;
        int baseOfs_x2 =
            -(activeLineOfs_ << 1) + (3 - currentField_) * halfFrameInterval_x2 + vOfs_ * 2;

        // (0-5-242.5*1+484*1+242.5)%485-242.5

        auto adjustLine = [&](int line_x2) -> int
        {
            return hw_divider_u32_remainder_inlined(line_x2 + baseOfs_x2, frameInterval_x2) - halfFrameInterval_x2;
        };

        auto nSrcPixels = std::min(720 - resampleOfs_, resampleWidth_);
        graphics::setupResizeYCbCr420Config(DISPLAY_WIDTH, nSrcPixels, resampleOfs_, resampleOfsFrac_ * 16);

        struct Log
        {
            uint16_t line;
            uint16_t tWait;
            uint32_t tRecover;
            uint32_t tAlloc;
            uint16_t tProc;
            uint8_t freeLines;
            uint8_t freeStreamBuffers;
        };
        static Log logs[240];
        int ilog = 0;

        int activeLines = std::min(DISPLAY_HEIGHT, activeLines_);
        int y = 0;
        while (y < activeLines)
        {
            auto t0 = util::getSysTickCounter24();

            buffer_.waitForNextLine();
            auto t1 = util::getSysTickCounter24();

            auto [line, linfo] = buffer_.getCurrentReadLine();

            const auto lx2 = adjustLine(linfo.lineIdx_x2);
            const auto l = lx2 >> 1;
            updateBound(line.data(), l + activeLineOfs_);
            if (l < y)
            {
                continue;
            }

            auto &log = logs[ilog++];

#ifdef NDEBUG
            constexpr int errorColor = 0;
#else
            constexpr int errorColor = 0x2080;
#endif

            while (l != y && y < activeLines)
            {
                int dstLineID = frameBuffer.allocateLine();
                auto p = frameBuffer.getLineBuffer(dstLineID);
                memset(p, errorColor, DISPLAY_WIDTH * 2);
                frameBuffer.commitNextLine(dstLineID);
                ++y;
            }
            auto t2 = util::getSysTickCounter24();

            if (dumpLineReq_ == y)
            {
                util::dump(line.begin(), line.end());
                dumpLineReq_ = -1;
            }

            if (y < activeLines)
            {
                auto freeLines = frameBuffer.getFreeLineCount();
                int dstLineID = frameBuffer.allocateLine();
                auto dstBuffer = frameBuffer.getLineBuffer(dstLineID);
                auto t3 = util::getSysTickCounter24();

                if (linfo.error)
                // if (0)
                {
                    //                    memset(dstBuffer, 0, DISPLAY_WIDTH * 2);
                    memset(dstBuffer, 0x4020, DISPLAY_WIDTH * 2);
                }
                else
                {
                    uint32_t resized[DISPLAY_WIDTH]; // 1280byte stack size注意
                    // graphics::resizeYCbCr420(resized, DISPLAY_WIDTH, line.data(), nSrcPixels, leftMargin_);
                    graphics::resizeYCbCr420PreConfig(resized, DISPLAY_WIDTH, line.data());
                    graphics::convertYCbCr2RGB565(dstBuffer, resized, DISPLAY_WIDTH);

#if ENABLE_CURSOR
                    if (cursorLine_ == y)
                    {
                        dstBuffer[DISPLAY_WIDTH - 32] = 0xaa55;
                    }
#endif
                }

                frameBuffer.commitNextLine(dstLineID);
                ++y;

                auto t4 = util::getSysTickCounter24();
                log.line = l;
                log.tWait = (t0 - t1) & 0xffffff;
                log.tRecover = (t1 - t2) & 0xffffff;
                log.tAlloc = (t2 - t3) & 0xffffff;
                log.tProc = (t3 - t4) & 0xffffff;
                log.freeLines = freeLines;
                log.freeStreamBuffers = buffer_.getFreeBufferSize();
            }
        }

        for (auto ct = DISPLAY_HEIGHT - y; ct > 0; --ct)
        {
            int dstLineID = frameBuffer.allocateLine();
            auto p = frameBuffer.getLineBuffer(dstLineID);
            memset(p, 0, DISPLAY_WIDTH * 2);
            frameBuffer.commitNextLine(dstLineID);
            ++y;
        }

        assert(y == DISPLAY_HEIGHT);

#if 1
        if (logReq_)
        {
            for (int i = 0; i < ilog; ++i)
            {
                auto &l = logs[i];
                printf("%d: l %d, tw %d, tr %d, ta %d, tp %d, fl %d, fb %d\n",
                       i, l.line, l.tWait, l.tRecover, l.tAlloc, l.tProc, l.freeLines, l.freeStreamBuffers);
            }

            logReq_ = false;
        }
#endif
        currentField_ ^= 1;
    }

    bool
    VideoCapture::analyzeSignal()
    {
#if USE_VSYNC_PIN
        return analyzeSignalHSVS();
#else
        return analyzeSignalVC();
#endif
    }

    bool
    VideoCapture::analyzeSignalHSVS()
    {
        auto &buffer = buffer_.getRawBuffer();

        auto waitLine = [&](int n)
        {
            for (int i = 0; i < n; ++i)
            {
                startVideoCapture(buffer.data(), EAV_VSYNC_F0, 1);
                waitVideoCapture();
            }
        };

        waitVSync();

        waitLine(20);

        startVideoCapture(buffer.data(), EAV_VSYNC_F0, buffer.size());
        waitVideoCapture();

        auto findHS = [&](bool active, int ofs) -> int
        {
            for (auto i = ofs; i < buffer.size(); ++i)
            {
                bool hs = buffer[i] & 0x1000000;
                if (hs == active)
                {
                    return i;
                }
            }
            return -1;
        };

        auto activeEnd = findHS(false, 0);
        auto nextLine = activeEnd > 0 ? findHS(true, activeEnd) : -1;
        if (activeEnd < 0 || nextLine < 0)
        {
            return false;
        }

        activeWidth_ = std::min<int>(activeEnd, VideoStreamBuffer::UNIT_BUFFER_SIZE);
        dataWidthInWords_ = activeWidth_;

        waitVSync();
        auto t0 = util::getSysTickCounter24();

        waitLine(20);

        auto ht0 = util::getSysTickCounter24();

        startVideoCapture(buffer.data(), EAV_VSYNC_F0, 1);
        waitVideoCapture();
        auto ht1 = util::getSysTickCounter24();

        waitVSync();
        auto t1 = util::getSysTickCounter24();
        waitVSync();
        auto t2 = util::getSysTickCounter24();

        auto VInterval0 = (t0 - t1) & 0xffffff;
        auto VInterval1 = (t1 - t2) & 0xffffff;
        auto HInterval = (ht0 - ht1) & 0xffffff;

        auto activeLines2 = (VInterval0 + VInterval1 + HInterval / 2) / HInterval;
        activeLines_ = activeLines2 >> 1;

        DBGPRINT("active width: %d (%d)\n", activeWidth_, activeEnd);
        DBGPRINT("line width: %d\n", nextLine);
        DBGPRINT("active lines: %d\n", activeLines_);
        DBGPRINT("V interval: %d, %d\n", VInterval0, VInterval1);
        DBGPRINT("H interval: %d\n", HInterval);

        return true;
    }

    bool
    VideoCapture::analyzeSignalVC()
    {
        auto findTimingCode = [&](const uint8_t *p, size_t size) -> std::pair<int, int>
        {
            for (auto i = 0u; i < size - 3; ++i)
            {
                if (p[i] == 0xff)
                {
                    if (p[i + 1] == 0 && p[i + 2] == 0)
                    {
                        return {p[i + 3], i};
                    }
                }
            }
            return {-1, -1};
        };

        constexpr auto getCodeByte = [](uint32_t codeWord)
        {
            return codeWord >> 24;
        };

        auto &buffer = buffer_.getRawBuffer();

        // とりあえず映像探す
        startVideoCapture(buffer.data(), SAV_ACTIVE_F0, buffer.size());
        waitVideoCapture();

        auto bytebuf = reinterpret_cast<uint8_t *>(buffer.data());

        auto [nextEAV0, activeSize] = findTimingCode(bytebuf, buffer.size() * 4 - 4);
        if (nextEAV0 < 0)
        {
            printf("no timing code : random line\n");
            util::dump(buffer.begin(), buffer.end());
            return false;
        }

        auto [nextSAV0, hBlankSize] = findTimingCode(bytebuf + activeSize + 4,
                                                     buffer.size() * 4 - activeSize - 4);
        if (nextSAV0 < 0)
        {
            printf("no next SAV\n");
            util::dump(buffer.begin(), buffer.end());
            return false;
        }

        // ラインは EAV_hoge, hblank, SAV_hoge, data ... みたいな順番
        // フレームは V0.., A0.., V0.., V1.., A1.., V1.., ... みたいなシーケンスになってる
        // 最初のV0が0個のこともあるかもしれないのでA0区間から計数を始める

        // V0.. A0.. A1.. V1.. A1.. A0.. V0.. みたいなパターンがある…

        // VSync
        startVideoCapture(buffer.data(), EAV_VSYNC_F0, 1);
        waitVideoCapture();

        uint8_t lineCodeLog[(MAX_ACTIVE_LINES + MAX_VBLANK_LINES) * 2];
        int analyzeLine = 0;

        std::vector<uint32_t> lineCt((MAX_ACTIVE_LINES + MAX_VBLANK_LINES) * 2);

        // Active 期間のライン数を数える
        startVideoCapture(buffer.data(), EAV_ACTIVE_F0, 1);
        int nextEAVbyte = getCodeByte(EAV_ACTIVE_F0);
        waitVideoCapture();

        auto t0 = util::getSysTickCounter24();
        auto pt = t0;

        const int readSizeInWord = ((activeSize + 3) >> 2) + 1 /* eav */ + 8 /*test*/;

        auto getEAVCode = [&]() -> int
        {
            auto v = buffer[activeSize >> 2];
            if ((v & 0xff) != 0xff)
            {
                return -1;
            }
            return v >> 24;
        };

        auto countLines = [&](const uint32_t targetSAV, const uint32_t targetEAV,
                              int curEAVbyte, int maxLines, const char *name)
            -> std::tuple<int, int>
        {
            auto targetEAVbyte = getCodeByte(targetEAV);
            if (targetEAVbyte != curEAVbyte)
            {
                return {curEAVbyte, 0};
            }

            int count = 0;
            while (1)
            {
                startVideoCapture(buffer.data(), targetSAV, readSizeInWord);
                ++count;
                waitVideoCapture();

                auto t = util::getSysTickCounter24();
                auto d = (pt - t) & 0xffffff;
                lineCodeLog[analyzeLine] = targetEAVbyte;
                lineCt[analyzeLine] = t;
                ++analyzeLine;
                pt = t;

                int nextEAVbyte = getEAVCode();
                if (nextEAVbyte < 0)
                {
                    printf("%s: error: no timing code, count = %d\n", name, count);
#ifndef NDEBUG
                    util::dump(buffer.begin(), buffer.begin() + readSizeInWord);
#endif
                    return {-1, 0};
                }
                if (nextEAVbyte != targetEAVbyte)
                {
                    return {nextEAVbyte, count};
                }

                if (count > maxLines)
                {
                    printf("%s: error: line over. (%d > %d)\n", name, count, maxLines);
                    util::dump(buffer.begin(), buffer.begin() + readSizeInWord);
                    return {-1, 0};
                }
            }
        };

        bool forceF0 = false;
        std::array<FieldInfo, 2> fields;
        for (int ii = 0; ii < 2; ++ii)
        {
            int i = forceF0 || ii == 0 ? 0 : 1;

            // Active
            const auto savActive0 = i == 0 ? SAV_ACTIVE_F0 : SAV_ACTIVE_F1;
            const auto eavActive0 = i == 0 ? EAV_ACTIVE_F0 : EAV_ACTIVE_F1;
            std::tie(nextEAVbyte, fields[ii].activeLines) = countLines(savActive0, eavActive0,
                                                                       nextEAVbyte, MAX_ACTIVE_LINES,
                                                                       i == 0 ? "F0" : "F1");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            const auto savActive1 = i == 0 ? SAV_ACTIVE_F1 : SAV_ACTIVE_F0;
            const auto eavActive1 = i == 0 ? EAV_ACTIVE_F1 : EAV_ACTIVE_F0;
            std::tie(nextEAVbyte, fields[ii].activeLines2) = countLines(savActive1, eavActive1,
                                                                        nextEAVbyte, MAX_ACTIVE_LINES,
                                                                        i == 0 ? "F1" : "F0");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            std::tie(nextEAVbyte, fields[ii].activeLines3) = countLines(savActive0, eavActive0,
                                                                        nextEAVbyte, MAX_ACTIVE_LINES,
                                                                        i == 0 ? "F0" : "F1");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            // Post VSync
            const auto savVsync0 = i == 0 ? SAV_VSYNC_F0 : SAV_VSYNC_F1;
            const auto eavVsync0 = i == 0 ? EAV_VSYNC_F0 : EAV_VSYNC_F1;
            std::tie(nextEAVbyte, fields[ii].postVSyncLines) = countLines(savVsync0, eavVsync0,
                                                                          nextEAVbyte, MAX_VBLANK_LINES / 2,
                                                                          i == 0 ? "V0(post)" : "V1(post)");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            // Pre VSync
            const auto savVsync1 = i == 0 ? SAV_VSYNC_F1 : SAV_VSYNC_F0;
            const auto eavVsync1 = i == 0 ? EAV_VSYNC_F1 : EAV_VSYNC_F0;
            std::tie(nextEAVbyte, fields[ii ^ 1].preVSyncLines) = countLines(savVsync1, eavVsync1,
                                                                             nextEAVbyte, MAX_VBLANK_LINES / 2,
                                                                             i == 0 ? "V1(pre)" : "V0(pre)");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            std::tie(nextEAVbyte, fields[ii].postVSyncLines2) = countLines(savVsync0, eavVsync0,
                                                                           nextEAVbyte, MAX_VBLANK_LINES / 2,
                                                                           i == 0 ? "V0(post)" : "V1(post)");
            if (nextEAVbyte < 0)
            {
                return false;
            }
            if (fields[0].postVSyncLines2)
            {
                forceF0 = true;
                // どっちのフィールドもF0が来ることがある
            }
        }

        auto t1 = util::getSysTickCounter24();

#if 1
        pt = t0;
        for (int i = 0; i < analyzeLine; ++i)
        {
            auto v = lineCodeLog[i];
            auto dt = (pt - lineCt[i]) & 0xffffff;
            auto l = ((t0 - lineCt[i]) & 0xffffff) / 16645.468750f;
            pt = lineCt[i];
            auto code = [&]
            {
                switch (v)
                {
                case getCodeByte(EAV_ACTIVE_F0):
                    return "ACTIVE_F0";
                case getCodeByte(EAV_ACTIVE_F1):
                    return "ACTIVE_F1";
                case getCodeByte(EAV_VSYNC_F0):
                    return "VSYNC_F0";
                case getCodeByte(EAV_VSYNC_F1):
                    return "VSYNC_F1";
                };
                return "unknown";
            }();
            // DBGPRINT("%2d: %d: %02x (%s) \n", i, dt, v, code);
            DBGPRINT("%2d: %d: %f:  %02x (%s) \n", i, dt, l, v, code);
        }
#endif

        // 次のactive区間
        if (nextEAVbyte != getCodeByte(EAV_ACTIVE_F0))
        {
            printf("error: unkown frame format: last EAV %02x\n", nextEAVbyte);
            return false;
        }

        if (fields[0].activeLines == 0 || fields[1].activeLines == 0)
        {
            printf("error: no active lines.\n");
            return false;
        }

        auto minV = std::min(fields[0].preVSyncLines, fields[1].preVSyncLines);
        auto maxV = std::max(fields[0].preVSyncLines, fields[1].preVSyncLines);

        activeWidth_ = activeSize >> 1;
        dataWidthInWords_ = readSizeInWord;
        hBlankSizeInBytes_ = hBlankSize;
        fieldInfo_ = fields;
        vSyncIntervalCycles_ = ((t0 - t1) & 0xffffff) / 2;
        // activeLineOfs_ = std::min(fields[0].preVSyncLines, fields[1].preVSyncLines);
        // activeLineOfs_ = minV - fields[0].preVSyncLines;
        // activeLineOfs_ = minV - maxV;
        activeLineOfs_ = fields[0].preVSyncLines > fields[1].preVSyncLines ? 0 : minV - maxV;
        // activeLines_ = std::max(fields[0].activeLines, fields[1].activeLines);
        activeLines_ = std::max(fields[0].getTotalActiveLines(),
                                fields[1].getTotalActiveLines()) -
                       activeLineOfs_;
        //  activeLines_ = fields[0].activeLines;
        startLine_ = 0;
        currentField_ = 0;

        constexpr int defaultLRMargin = (720 - 512) / 2;
        leftMargin_ = defaultLRMargin;
        rightMargin_ = defaultLRMargin;

        constexpr int defaultTBMargin = 40;
        topBound_ = defaultTBMargin;
        bottomBound_ = 240 - defaultTBMargin;

        //    activeLines_ /= 2;

        DBGPRINT("Data width  : %d.\n", activeWidth_);
        DBGPRINT("H blank     : %d.%d.\n", hBlankSize >> 1, hBlankSize & 1 ? 5 : 0);
        for (int i = 0; i < 2; ++i)
        {
            DBGPRINT("Field%d      : V(pre) %d, Active %d+%d+%d, V(post) %d\n",
                     i, fields[i].preVSyncLines,
                     fields[i].activeLines, fields[i].activeLines2, fields[i].activeLines3,
                     fields[i].postVSyncLines);
        }
        DBGPRINT("V sync cycle: %d\n", vSyncIntervalCycles_);
        DBGPRINT("FPS         : %f\n", (float)CPU_CLOCK_KHZ * 1000 / vSyncIntervalCycles_);

        return true;
    }

    void VideoCapture::measureInterval()
    {
        auto *tmpBuffer = buffer_.getRawBuffer().data();
        {
            uint16_t diff[MAX_ACTIVE_LINES];
            auto *dp = diff;

            int ct = fieldInfo_[0].activeLines * 3 >> 2;

            startVideoCapture(tmpBuffer, EAV_VSYNC_F0, 1);
            waitVideoCapture();
            startVideoCapture(tmpBuffer, EAV_ACTIVE_F0, 1);
            waitVideoCapture();

            auto pt = util::getSysTickCounter24();
            auto t0 = pt;

            for (int i = 0; i < ct; ++i)
            {
                startVideoCapture(tmpBuffer, EAV_ACTIVE_F0, 1);
                waitVideoCapture();

                auto t = util::getSysTickCounter24();
                *dp++ = (pt - t) & 0xffffff;
                pt = t;
            }

            auto ave128 = (((t0 - pt) & 0xffffff) << 7) / ct;

            int d2 = 0;
            for (int i = 0; i < ct; ++i)
            {
                auto d = diff[i];
                auto dd = (d << 7) - ave128;
                d2 += dd * dd;
            }
            d2 /= ct;

            lineIntervalCycles128_ = ave128;
            DBGPRINT("line interval: ave %f, sigma %f\n",
                     ave128 / 128.0f,
                     sqrt(d2 / ct / 128.0f));
        }
        //
        {
            startVideoCapture(tmpBuffer, EAV_VSYNC_F0, 1);
            waitVideoCapture();
            startVideoCapture(tmpBuffer, EAV_ACTIVE_F0, 1);
            waitVideoCapture();

            auto pt = util::getSysTickCounter24();
            auto t0 = pt;

            constexpr auto N_FRAMES = 8;
            uint32_t frameDiff[N_FRAMES];
            uint32_t totalDiff = 0;

            for (int i = 0; i < N_FRAMES; ++i)
            {
                startVideoCapture(tmpBuffer, EAV_VSYNC_F1, 1);
                waitVideoCapture();
                startVideoCapture(tmpBuffer, EAV_ACTIVE_F1, 1);
                waitVideoCapture();
                startVideoCapture(tmpBuffer, EAV_VSYNC_F0, 1);
                waitVideoCapture();
                startVideoCapture(tmpBuffer, EAV_ACTIVE_F0, 1);
                waitVideoCapture();

                auto t = util::getSysTickCounter24();
                auto d = (pt - t) & 0xffffff;
                frameDiff[i] = d;
                totalDiff += d;
                pt = t;
            }

            for (int i = 0; i < N_FRAMES; ++i)
            {
                DBGPRINT("f%d: %d\n", i, frameDiff[i]);
            }
            auto aveFrame = totalDiff / N_FRAMES;
            frameIntervalCycles_ = aveFrame;
            frameIntervalLines_ = ((aveFrame << 7) + (lineIntervalCycles128_ >> 1)) / lineIntervalCycles128_;
            DBGPRINT("Frame interval: %d, %d lines (%f)\n",
                     aveFrame,
                     frameIntervalLines_,
                     aveFrame * 128.0f / lineIntervalCycles128_);

            // 266000000/50*2 = 10640000
            // 50fpsくらいで24bit必要
        }
    }

    void VideoCapture::simpleCaptureTest()
    {
#if 1
        auto &buffer = buffer_.getRawBuffer();
        startVideoCapture(buffer.data(), EAV_VSYNC_F0, 1);
        waitVideoCapture();

        startVideoCapture(buffer.data(), EAV_ACTIVE_F0, buffer.size());
        waitVideoCapture();
        DBGPRINT("Active line:\n");
        util::dump(buffer.begin(), buffer.end());

        startVideoCapture(buffer.data(), EAV_ACTIVE_F0, 1);
        waitVideoCapture();
        startVideoCapture(buffer.data(), EAV_VSYNC_F0, buffer.size());
        waitVideoCapture();
        DBGPRINT("VSync line:\n");
        util::dump(buffer.begin(), buffer.end());
#else
        DBGPRINT("Test\n");
        auto &buffer = buffer_.getRawBuffer();
        // startVideoCapture(buffer.data(), EAV_VSYNC_F0, buffer.size());
        // startVideoCapture(buffer.data(), 0x09d0f00f, buffer.size());
        startVideoCapture(buffer.data(), 0xec00f000, buffer.size());
        waitVideoCapture();

        // graphics::fixBitOrderV4(buffer.data(), buffer.size());
        // graphics::convertXRGB8888toRGB565((uint16_t *)buffer.data(), buffer.data(), buffer.size());
        // graphics::convertBGRB888toBGR565((uint16_t *)buffer.data(), (uint8_t *)buffer.data(), buffer.size());

        util::dump(buffer.begin(), buffer.end());
#endif
    }

    void VideoCapture::imageConvertTest()
    {
        auto &buffer = buffer_.getRawBuffer();

        for (int i = 0; i < 720 / 2; ++i)
            //            buffer_[i] = 0x01900080 + (0x02020202 * i);
            buffer[i] = 0x01800080 + (0x02000200 * i);

        uint32_t tmp[320];
        graphics::resizeYCbCr420(tmp, 320, buffer.data(), 720);

        DBGPRINT("src:\n");
        util::dump((uint16_t *)buffer.data(), (uint16_t *)buffer.data() + 720, "%04x ");
        DBGPRINT("scaled:\n");
        util::dump(tmp, tmp + 320);

        uint16_t tmp2[320];
        graphics::convertYCbCr2RGB565(tmp2, tmp, 320);

        DBGPRINT("rgb:\n");
        util::dumpF(tmp2, tmp2 + 320, [](int v)
                    { DBGPRINT("(%2d, %2d, %2d) ", (v >> 11) & 31, (v >> 5) & 63, v & 31); });
    }

    void
    VideoCapture::initMenu(ui::Menu &menu)
    {
        menu.appendItem(&resampleWidth_, {512, 720}, "SAMPLE W", "Reset", {}, [&]
                        { resampleWidth_ = 640; });

        menu.appendItem(&resampleWidthFrac_, {0, 15}, "WIDTH FRAC", "Reset", {}, [&]
                        { resampleWidthFrac_ = 0; });

        menu.appendItem(&resampleOfs_, {0, 255}, "SAMPLE OFS", "Reset", {}, [&]
                        { resampleOfs_ = 40; });

        menu.appendItem(&resampleOfsFrac_, {0, 15}, "OFS FRAC", "Reset", {}, [&]
                        { resampleOfsFrac_ = 0; });

        menu.appendItem(&vOfs_, {-50, 50}, "V OFFSET", "Reset", {}, [&]
                        { vOfs_ = 0; });

#ifndef NDEBUG
        // menu.appendItem(
        //         &hdiv_, {848, 868}, "PLL H DIV", "Reset",
        //         [&]
        //         { setHDiv(hdiv_); },
        //         [&]
        //         { setHDiv(858); })
        //     .setInsensitiveFunc([this]
        //                         { return adv7181_->isSDPMode(); });

        // menu.appendItem("AUTO ADJ.", "Adjust Params",
        //                 [&]
        //                 {
        //                     resampleOfs_ = leftMargin_;
        //                     resampleWidth_ = 720 - leftMargin_ - rightMargin_;
        //                 });
        menu.appendItem("DB CAPTIME", "Dump Cap Timing",
                        [&]
                        {
                            requestLog();
                        });
#endif
    }

#if 0
    void
    VideoCapture::onMenuClose()
    {
        if (signalDetected_)
        {
            NVSettings::CaptureSettings s;
            s.stdiState = currentSTDIState_;
            s.resampleWidth = resampleWidth_;
            s.resampleOfs = resampleOfs_;
            s.resamplePhaseOfs = resamplePhaseOfs_;
            s.vOfs = vOfs_;

            NVSettings::instance().setCaptureSetting(s);
        }
    }
#endif

    ///////////////
    void RemoteVideoCaptureHandler::startBGCapture()
    {
        com_->sendRequest(VideoCaptureRequest::START_CAPTURE);
    }

    void RemoteVideoCaptureHandler::stopBGCapture()
    {
        com_->sendRequest(VideoCaptureRequest::STOP_CAPTURE);
    }

    bool RemoteVideoCaptureHandler::analyzeSignal()
    {
        return com_->sendRequest(VideoCaptureRequest::ANALYZE_SIGNAL);
    }

    void RemoteVideoCaptureHandler::startCaptureFrame()
    {
        com_->sendRequest(VideoCaptureRequest::CAPTURE_FRAME);
    }

    void RemoteVideoCaptureHandler::endCaptureFrame()
    {
        com_->sendRequest(VideoCaptureRequest::WAIT_CAPTURE);
    }

}