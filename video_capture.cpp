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
#include "app.h"

namespace video
{

    bool VideoCapture::tick(graphics::FrameBuffer &frameBuffer,
                            device::ADV7181 &adv7181)
    {
        if (signalDetected_ && test(currentSTDIState_, adv7181.getSTDIState(), 10))
        {
            captureFrame2(frameBuffer);
            return true;
        }
        else
        {
            stopBGCapture();

            auto waitForCounterStable = [&]
            {
                constexpr int timeout = 100; // ms
                if (adv7181.getCurrentInput() == device::SignalInput::RGB21)
                {
                    for (auto csync : {false, true})
                    {
                        adv7181.setRGBSyncModeCSync(csync);
                        if (adv7181.waitForCounterStable(timeout))
                        {
                            printf("csync = %d\n", csync);
                            return true;
                        }
                    }
                }
                else
                {
                    return adv7181.waitForCounterStable(timeout);
                }
                return false;
            };

            if (waitForCounterStable())
            {
                currentSTDIState_ = adv7181.getSTDIState();
                currentSTDIState_.dump();

                adv7181.setLineLength(currentSTDIState_.blockSize / 8);

                if (analyzeSignal())
                {
                    measureInterval();
                    baseCycleCounter_ = -1;

                    startBGCapture();
                }
            }
            else
            {
                printf("video signal is not stabled.\n");
            }
        }
        return false;
    }

    ////

    void
    VideoCapture::startCaptureLine(BT656TimingCode sav, uint32_t time)
    {
        auto [line, linfo] = buffer_.getCurrentWriteLine();

        auto t = (baseTime_ - time) & 0xffffff;
        int l2 = hw_divider_u32_quotient_inlined((t << 8) + (lineIntervalCycles128_ >> 1),
                                                 lineIntervalCycles128_);
        // printf("t %d, l2 %d\n", t, l2);

        linfo.code = makeBT656TimingCodeByte(sav);
        linfo.time = t;
        linfo.lineIdx_x2 = l2;
        linfo.error = false;

        int d2 = l2 - (frameIntervalLines_ << 1);
        if (d2 >= 0)
        {
            auto overTime = d2 * lineIntervalCycles128_ >> 8;
            baseTime_ = time - overTime;
        }

        startVideoCapture(line.data(), asUInt(sav), dataWidthInWords_);
    }

    bool
    VideoCapture::irq(uint32_t time)
    {
        assert(runningIRQ_);
        auto [line, linfo] = buffer_.getCurrentWriteLine();
        auto [nextEAV, error] = video::findNextEAV(line.data(), activeWidth_ >> 1);
        linfo.error = error;

        buffer_.nextWrite();

        if (breakIRQ_)
        {
            runningIRQ_ = false;
            __sev();
            return false; // break
        }
        else
        {
            auto nextSAV = video::getSAVCorrespondingToEAV(nextEAV);
            if (nextSAV == BT656TimingCode::INVALID)
            {
                nextSAV = BT656TimingCode::SAV_ACTIVE_F0;
            }
            startCaptureLine(nextSAV, time);
            // printf("%p, l%d, %x, %x, %d\n", line.data(), linfo.lineIdx_x2, asUInt(nextEAV), asUInt(nextSAV), error);
        }
        return true; // continue
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
    }

    void
    VideoCapture::startBGCapture()
    {
        stopBGCapture();

        assert(!runningIRQ_);

        auto *tmp = buffer_.getRawBuffer().data();
        startVideoCapture(tmp, asUInt(video::BT656TimingCode::EAV_VSYNC_F0), 1);
        waitVideoCapture();
        startVideoCapture(tmp, asUInt(video::BT656TimingCode::EAV_ACTIVE_F0), 1);
        waitVideoCapture();
        // ACTIVE_F0 の先頭を基準とする

        auto t = util::getSysTickCounter24();
        baseTime_ = t;

        runningIRQ_ = true;
        breakIRQ_ = false;
        enableVideoCaptureIRQ(true);

        startCaptureLine(video::BT656TimingCode::SAV_ACTIVE_F0, t);
    }

    void
    VideoCapture::captureFrame2(graphics::FrameBuffer &frameBuffer)
    {
        // buffer_.reset();

        int halfFrameInterval_x2 = frameIntervalLines_;
        int frameInterval_x2 = frameIntervalLines_ << 1;
        int baseOfs_x2 =
            -(activeLineOfs_ << 1) + (3 - currentField_) * halfFrameInterval_x2;

        // (0-5-242.5*1+484*1+242.5)%485-242.5

        auto adjustLine = [&](int line_x2) -> int
        {
            return hw_divider_u32_remainder_inlined(line_x2 + baseOfs_x2, frameInterval_x2) - halfFrameInterval_x2;
        };

        auto nSrcPixels = (dataWidthInWords_ << 1) - leftMargin_ - rightMargin_;

        graphics::setupResizeYCbCr420Config(DISPLAY_WIDTH, nSrcPixels, leftMargin_);

        struct Log
        {
            uint16_t line;
            uint16_t tWait;
            uint16_t tAlloc;
            uint16_t tProc;
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
            auto l = adjustLine(linfo.lineIdx_x2) >> 1;
            // printf("l = %d, %d\n", l, linfo.lineIdx_x2);
            if (l < y)
            {
                continue;
            }

            auto &log = logs[ilog++];

            while (l != y && y < activeLines)
            {
                int dstLineID = frameBuffer.allocateLine();
                auto p = frameBuffer.getLineBuffer(dstLineID);
                memset(p, 0, DISPLAY_WIDTH * 2);
                frameBuffer.commitNextLine(dstLineID);
                ++y;
            }

            if (dumpLineReq_ == y)
            {
                util::dump(line.begin(), line.end());
                dumpLineReq_ = -1;
            }

            if (y < activeLines)
            {
                int dstLineID = frameBuffer.allocateLine();
                auto dstBuffer = frameBuffer.getLineBuffer(dstLineID);
                auto t2 = util::getSysTickCounter24();

                if (linfo.error)
                // if (0)
                {
                    memset(dstBuffer, 0, DISPLAY_WIDTH * 2);
                }
                else
                {
                    uint32_t resized[DISPLAY_WIDTH]; // 1280byte stack size注意
                    // graphics::resizeYCbCr420(resized, DISPLAY_WIDTH, line.data(), nSrcPixels, leftMargin_);
                    graphics::resizeYCbCr420PreConfig(resized, DISPLAY_WIDTH, line.data());
                    graphics::convertYCbCr2RGB565(dstBuffer, resized, DISPLAY_WIDTH);

                    if (cursorLine_ == y)
                    {
                        dstBuffer[DISPLAY_WIDTH - 32] = 0xaa55;
                    }
                }

                frameBuffer.commitNextLine(dstLineID);
                ++y;

                auto t3 = util::getSysTickCounter24();
                log.line = l;
                log.tWait = (t0 - t1) & 0xffffff;
                log.tAlloc = (t1 - t2) & 0xffffff;
                log.tProc = (t2 - t3) & 0xffffff;
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

#if 0
        for (int i = 0; i < ilog; ++i)
        {
            auto &l = logs[i];
            printf("%d: l %d, tw %d, ta %d, tp %d\n", i, l.line, l.tWait, l.tAlloc, l.tProc);
        }
#endif

        currentField_ ^= 1;
    }

    namespace
    {
#define ENABLE_LOG 1
#if ENABLE_LOG
        struct Timing
        {
            const char *name;
            uint32_t counter;
        };

        Timing timingLog_[4096];
        Timing *curTimingLog_ = timingLog_;

        void resetMark()
        {
            curTimingLog_ = timingLog_;
        }

        void mark(const char *name)
        {
            curTimingLog_->counter = util::getSysTickCounter24();
            curTimingLog_->name = name;
            ++curTimingLog_;
        }

        void dumpTiming()
        {
            auto *p = timingLog_;
            auto *prev = p;
            ++p;
            while (p != curTimingLog_)
            {
                printf("%s: %d\n", prev->name, (prev->counter - p->counter) & 0xffffff);
                prev = p++;
            }

            resetMark();
        }
#else
        void resetMark()
        {
        }

        void mark(const char *)
        {
        }

        void dumpTiming()
        {
        }
#endif
    }

    void VideoCapture::captureFrame(graphics::FrameBuffer &frameBuffer)
    {
        resetMark();
        mark(currentField_ == 0 ? "wait V0" : "wait V1");

        auto activeLineOfs = activeLineOfs_;

        // よくわからん調整…
        int baseOfs = fieldInfo_[currentField_].activeLines2 ? 1 : 0;

#if 0
    activeLineOfs += currentField_ != 0 ? 15 : 0;
#endif

        int y = 0;
        if (startLine_)
        {
            mark("fill pre active line");
            for (auto ct = startLine_; ct; --ct)
            {
                int lineID = frameBuffer.allocateLine();
                auto p = frameBuffer.getLineBuffer(lineID);
                memset(p, 0, DISPLAY_WIDTH * 2);
                frameBuffer.commitNextLine(lineID);
                ++y;
            }
        }

        auto srcPixels = (dataWidthInWords_ << 1) - leftMargin_ - rightMargin_;

        // VSync確認
        auto eav = currentField_ == 0 ? EAV_VSYNC_F0 : EAV_VSYNC_F1;
        startVideoCapture(buffer_.getRawBuffer().data(), eav, 1);
        waitVideoCapture();

        auto ctv0 = util::getSysTickCounter24() & 0xffffff;

        auto vsyncEAV = eav;
        auto vsyncEAVCount = 1;

        int activeLine = 0;

        auto getBuffer = [&](int dbid)
        {
            return buffer_.getBuffer(dbid).data();
        };
        auto captureLine = getBuffer(0);

        auto getNextSAV = [](uint32_t eav)
        {
            return (uint32_t)getSAVCorrespondingToEAV((BT656TimingCode)eav);
        };

        mark("start capture active line");
        auto sav = getNextSAV(eav);
        startVideoCapture(captureLine, sav, dataWidthInWords_);

        int aaa = 0;
        if (baseCycleCounter_ >= 0)
        {
            int df = (baseCycleCounter_ - ctv0) & 0xffffff;
            int fieldLines = hw_divider_u32_quotient_inlined(
                (df << 7) + (lineIntervalCycles128_ >> 1),
                lineIntervalCycles128_);
            int adj = (frameIntervalLines_ >> 1) - fieldLines + baseOfs;
            // aaa = adj;
            aaa = fieldLines;
            if (adj == 0 || (++deviatedFrames_ == 30))
            {
                deviatedFrames_ = 0;
            }
            else
            {
                // 基準調整
                // ctv0 -= (adj * lineIntervalCycles256_ >> 8);
            }
            // activeLineOfs += adj + baseOfs;
            activeLineOfs += baseOfs;
        }
        baseCycleCounter_ = ctv0;

        while (y < DISPLAY_HEIGHT && activeLine < activeLines_)
        {
            bool skip = activeLine < activeLineOfs;
            ++activeLine;

            auto video = captureLine;
            captureLine = getBuffer(activeLine & 1);

            waitVideoCapture();

            bool error = false;
            if (activeLine < activeLines_)
            {
                auto ofs = activeWidth_ >> 1;
                eav = video[ofs];
                // マーカーがずれていそうならエラーとしつつリカバリーを試みる
                if ((eav & 0xff) != 0xff)
                {
                    error = true;
                    auto pv = video[ofs + 1];
                    do
                    {
                        eav = (eav >> 8) | ((pv << 24) & 0xff000000);
                        pv >>= 8;
                    } while (pv && (eav & 0xff) != 0xff);
                }

                sav = getNextSAV(eav);
                if (!sav)
                {
                    printf("error eav = %08x, y %d, al %d\n", eav, y, activeLine);
                    util::dump(video, video + dataWidthInWords_);
                    break;
                    return;
                }
                startVideoCapture(captureLine, sav, dataWidthInWords_);
            }

            if (eav == vsyncEAV)
                ++vsyncEAVCount;

            if (!skip)
            {
                mark("proc active line");
                int lineID = frameBuffer.allocateLine();
                auto dstBuffer = frameBuffer.getLineBuffer(lineID);

                if (error)
                {
                    memset(dstBuffer, 0, DISPLAY_WIDTH * 2);
                }
                else
                {
                    uint32_t resized[DISPLAY_WIDTH]; // 1280byte のこりstack注意
                    graphics::resizeYCbCr420(resized, DISPLAY_WIDTH, video, srcPixels, leftMargin_);
                    graphics::convertYCbCr2RGB565(dstBuffer, resized, DISPLAY_WIDTH);
                }
                frameBuffer.commitNextLine(lineID);
                ++y;
            }
            else
            {
                mark("skip line");
            }
        }

        mark("post active");
        for (auto ct = DISPLAY_HEIGHT - y; ct > 0; --ct)
        {
            int lineID = frameBuffer.allocateLine();
            auto p = frameBuffer.getLineBuffer(lineID);
            memset(p, 0, DISPLAY_WIDTH * 2);
            frameBuffer.commitNextLine(lineID);
            ++y;
        }

        // int d = fieldInfo_[currentField_].preVSyncLines - vsyncEAVCount;
        // if (d)
        //     printf("%d:%d, %d\n", currentField_, d, vsyncEAVCount);

        // printf("%d: %d\n", currentField_, aaa);
        currentField_ ^= 1;

        mark("end");

        // dumpTiming();
    }

    bool
    VideoCapture::analyzeSignal()
    {
        signalDetected_ = false;

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

        static uint32_t lineCt[(MAX_ACTIVE_LINES + MAX_VBLANK_LINES) * 2];

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
                    util::dump(buffer.begin(), buffer.begin() + readSizeInWord);
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

        std::array<FieldInfo, 2> fields;
        for (int i = 0; i < 2; ++i)
        {
            // Active
            const auto savActive0 = i == 0 ? SAV_ACTIVE_F0 : SAV_ACTIVE_F1;
            const auto eavActive0 = i == 0 ? EAV_ACTIVE_F0 : EAV_ACTIVE_F1;
            std::tie(nextEAVbyte, fields[i].activeLines) = countLines(savActive0, eavActive0,
                                                                      nextEAVbyte, MAX_ACTIVE_LINES,
                                                                      i == 0 ? "F0" : "F1");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            const auto savActive1 = i == 0 ? SAV_ACTIVE_F1 : SAV_ACTIVE_F0;
            const auto eavActive1 = i == 0 ? EAV_ACTIVE_F1 : EAV_ACTIVE_F0;
            std::tie(nextEAVbyte, fields[i].activeLines2) = countLines(savActive1, eavActive1,
                                                                       nextEAVbyte, MAX_ACTIVE_LINES,
                                                                       i == 0 ? "F1" : "F0");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            // Post VSync
            const auto savVsync0 = i == 0 ? SAV_VSYNC_F0 : SAV_VSYNC_F1;
            const auto eavVsync0 = i == 0 ? EAV_VSYNC_F0 : EAV_VSYNC_F1;
            std::tie(nextEAVbyte, fields[i].postVSyncLines) = countLines(savVsync0, eavVsync0,
                                                                         nextEAVbyte, MAX_VBLANK_LINES / 2,
                                                                         i == 0 ? "V0(post)" : "V1(post)");
            if (nextEAVbyte < 0)
            {
                return false;
            }

            // Pre VSync
            const auto savVsync1 = i == 0 ? SAV_VSYNC_F1 : SAV_VSYNC_F0;
            const auto eavVsync1 = i == 0 ? EAV_VSYNC_F1 : EAV_VSYNC_F0;
            std::tie(nextEAVbyte, fields[i ^ 1].preVSyncLines) = countLines(savVsync1, eavVsync1,
                                                                            nextEAVbyte, MAX_VBLANK_LINES / 2,
                                                                            i == 0 ? "V1(pre)" : "V0(pre)");
            if (nextEAVbyte < 0)
            {
                return false;
            }
        }

        auto t1 = util::getSysTickCounter24();

#if 0
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
            // printf("%2d: %d: %02x (%s) \n", i, dt, v, code);
            printf("%2d: %d: %f:  %02x (%s) \n", i, dt, l, v, code);
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
        signalDetected_ = true;
        // activeLineOfs_ = std::min(fields[0].preVSyncLines, fields[1].preVSyncLines);
        // activeLineOfs_ = minV - fields[0].preVSyncLines;
        // activeLineOfs_ = minV - maxV;
        activeLineOfs_ = fields[0].preVSyncLines > fields[1].preVSyncLines ? 0 : minV - maxV;
        // activeLines_ = std::max(fields[0].activeLines, fields[1].activeLines);
        activeLines_ = std::max(fields[0].activeLines, fields[1].activeLines) - activeLineOfs_;
        //  activeLines_ = fields[0].activeLines;
        startLine_ = 0;
        currentField_ = 0;

        //    activeLines_ /= 2;

        printf("Data width  : %d.\n", activeWidth_);
        printf("H blank     : %d.%d.\n", hBlankSize >> 1, hBlankSize & 1 ? 5 : 0);
        for (int i = 0; i < 2; ++i)
        {
            printf("Field%d      : V(pre) %d, Active %d+%d, V(post) %d\n",
                   i, fields[i].preVSyncLines,
                   fields[i].activeLines, fields[i].activeLines2,
                   fields[i].postVSyncLines);
        }
        printf("V sync cycle: %d\n", vSyncIntervalCycles_);
        printf("FPS         : %f\n", (float)CPU_CLOCK_KHZ * 1000 / vSyncIntervalCycles_);

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
            printf("line interval: ave %f, sigma %f\n",
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
                printf("f%d: %d\n", i, frameDiff[i]);
            }
            auto aveFrame = totalDiff / N_FRAMES;
            frameIntervalCycles_ = aveFrame;
            frameIntervalLines_ = ((aveFrame << 7) + (lineIntervalCycles128_ >> 1)) / lineIntervalCycles128_;
            printf("Frame interval: %d, %d lines (%f)\n",
                   aveFrame,
                   frameIntervalLines_,
                   aveFrame * 128.0f / lineIntervalCycles128_);

            // 266000000/50*2 = 10640000
            // 50fpsくらいで24bit必要
        }
    }

    void VideoCapture::simpleCaptureTest()
    {
        auto &buffer = buffer_.getRawBuffer();
        startVideoCapture(buffer.data(), EAV_VSYNC_F0, 1);
        waitVideoCapture();

        startVideoCapture(buffer.data(), EAV_ACTIVE_F0, buffer.size());
        waitVideoCapture();
        printf("Active line:\n");
        util::dump(buffer.begin(), buffer.end());

        startVideoCapture(buffer.data(), EAV_ACTIVE_F0, 1);
        waitVideoCapture();
        startVideoCapture(buffer.data(), EAV_VSYNC_F0, buffer.size());
        waitVideoCapture();
        printf("VSync line:\n");
        util::dump(buffer.begin(), buffer.end());
    }

    void VideoCapture::imageConvertTest()
    {
        auto &buffer = buffer_.getRawBuffer();

        for (int i = 0; i < 720 / 2; ++i)
            //            buffer_[i] = 0x01900080 + (0x02020202 * i);
            buffer[i] = 0x01800080 + (0x02000200 * i);

        uint32_t tmp[320];
        graphics::resizeYCbCr420(tmp, 320, buffer.data(), 720);

        printf("src:\n");
        util::dump((uint16_t *)buffer.data(), (uint16_t *)buffer.data() + 720, "%04x ");
        printf("scaled:\n");
        util::dump(tmp, tmp + 320);

        uint16_t tmp2[320];
        graphics::convertYCbCr2RGB565(tmp2, tmp, 320);

        printf("rgb:\n");
        util::dumpF(tmp2, tmp2 + 320, [](int v)
                    { printf("(%2d, %2d, %2d) ", (v >> 11) & 31, (v >> 5) & 63, v & 31); });
    }

}