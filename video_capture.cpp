/*
 * author : Shuichi TAKANO
 * since  : Sun Jun 19 2022 22:11:16
 */

#include "video_capture.h"

#include <stdio.h>
#include <string.h>
#include <tuple>
#include "image_proc.h"
#include "util.h"
#include "app.h"

bool VideoCapture::tick(graphics::FrameBuffer &frameBuffer)
{
    if (signalDetected_)
    {
        captureFrame(frameBuffer);
        return true;
    }
    else
    {
        analyzeSignal();
    }
    return false;
}

namespace
{
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
}

void VideoCapture::captureFrame(graphics::FrameBuffer &frameBuffer)
{
    resetMark();
    mark(currentField_ == 0 ? "wait V0" : "wait V1");

    auto activeLineOfs = activeLineOfs_;

#if 0
    if (currentField_ == 0)
    {
        activeLineOfs += 2;
    }
#endif

    int y = 0;
    if (activeLineOfs < 0)
    {
        mark("fill pre active line");
        for (auto ct = -activeLineOfs; ct; --ct)
        {
            int lineID = frameBuffer.allocateLine();
            auto p = frameBuffer.getLineBuffer(lineID);
            memset(p, 0, DISPLAY_WIDTH * 2);
            frameBuffer.commitNextLine(lineID);
            ++y;
        }
    }

    // VSync確認
    auto eav = currentField_ == 0 ? EAV_VSYNC_F0 : EAV_VSYNC_F1;
    startVideoCapture(buffer_, eav, 1);

    auto getNextSAV = [&](uint32_t eav, uint32_t prevSAV) -> uint32_t
    {
        switch (eav)
        {
        case EAV_ACTIVE_F0:
            return SAV_ACTIVE_F0;

        case EAV_ACTIVE_F1:
            return SAV_ACTIVE_F1;

            // case 0x000000ff:
            //     return prevSAV; /// fallback
        }
        return 0;
    };

    auto sav = currentField_ == 0 ? SAV_ACTIVE_F0 : SAV_ACTIVE_F1;
    auto srcPixels = (dataWidthInWords_ << 1) - leftMargin_ - rightMargin_;
    int activeLine = 0;
    auto captureLine = getBuffer(0);
    waitVideoCapture();

    mark("start capture active line");
    startVideoCapture(captureLine, sav, dataWidthInWords_);

    while (y < DISPLAY_HEIGHT && activeLine < activeLines_)
    {
        bool skip = activeLine < activeLineOfs;

        ++activeLine;

        auto video = captureLine;
        captureLine = getBuffer(activeLine & 1);

        waitVideoCapture();
        if (activeLine < activeLines_)
        {
            eav = video[activeWidth_ >> 1];
            sav = getNextSAV(eav, sav);
            if (!sav)
            {
                break;
                printf("error eav = %08x, y %d, al %d\n", eav, y, activeLine);
                util::dump(video, video + dataWidthInWords_);
                return;
            }
            startVideoCapture(captureLine, sav, dataWidthInWords_);
        }
        mark("proc active line");

        if (!skip)
        {
            int lineID = frameBuffer.allocateLine();
            auto dstBuffer = frameBuffer.getLineBuffer(lineID);

            uint32_t resized[DISPLAY_WIDTH];
            graphics::resizeYCbCr420(resized, DISPLAY_WIDTH, video, srcPixels, leftMargin_);
            graphics::convertYCbCr2RGB565(dstBuffer, resized, DISPLAY_WIDTH);

            frameBuffer.commitNextLine(lineID);
            ++y;
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

    currentField_ ^= 1;

    mark("end");
    // dumpTiming();
}

bool VideoCapture::analyzeSignal()
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

    // とりあえず映像探す
    startVideoCapture(buffer_, SAV_ACTIVE_F0, TOTAL_BUFFER_SIZE);
    waitVideoCapture();

    auto bytebuf = reinterpret_cast<uint8_t *>(buffer_);

    int eavActive, activeSize;
    std::tie(eavActive, activeSize) = findTimingCode(bytebuf, TOTAL_BUFFER_SIZE * 4 - 4);
    if (eavActive < 0)
    {
        printf("no timing code : random line\n");
        util::dump(std::begin(buffer_), std::end(buffer_));
        return false;
    }

    constexpr auto getCodeByte = [](uint32_t codeWord)
    {
        return codeWord >> 24;
    };

    int nextCode, hBlankSize;
    std::tie(nextCode, hBlankSize) = findTimingCode(bytebuf + activeSize + 4,
                                                    TOTAL_BUFFER_SIZE * 4 - activeSize - 4);
    if (nextCode < 0)
    {
        printf("no next SAV\n");
        return false;
    }

    // EAV_hoge, hblank, SAV_hoge, data ... みたいな順番

    // VSync
    startVideoCapture(buffer_, EAV_VSYNC_F0, 1);
    waitVideoCapture();

    uint8_t lineCodeLog[(MAX_ACTIVE_LINES + MAX_VBLANK_LINES) * 1];
    int analyzeLine = 0;

    // Active 期間のライン数を数える
    nextCode = getCodeByte(EAV_ACTIVE_F0);
    startVideoCapture(buffer_, EAV_ACTIVE_F0, 1);
    waitVideoCapture();

    auto t0 = util::getSysTickCounter24();

    const int readSizeInWord = ((activeSize + 3) >> 2) + 1 /* eav */;

    auto getEAVCode = [&]() -> int
    {
        auto v = buffer_[activeSize >> 2];
        if ((v & 0xff) != 0xff)
        {
            return -1;
        }
        return v >> 24;
    };

    int activeLinesF0 = 0;
    while (1)
    {
        auto getNextSAV = [&]() -> int
        {
            switch (nextCode)
            {
            case getCodeByte(EAV_ACTIVE_F0):
                return SAV_ACTIVE_F0;

            case getCodeByte(EAV_ACTIVE_F1):
                return SAV_ACTIVE_F1;
            };
            return 0;
        };

        auto sav = getNextSAV();
        if (!sav)
        {
            break;
        }

        lineCodeLog[analyzeLine++] = nextCode;

        startVideoCapture(buffer_, sav, readSizeInWord);
        waitVideoCapture();

        nextCode = getEAVCode();
        if (nextCode < 0)
        {
            printf("error: no timing code (active F0).\n");
            return false;
        }

        ++activeLinesF0;

        if (activeLinesF0 > MAX_ACTIVE_LINES)
        {
            printf("error: active line over.\n");
            util::dump(buffer_, buffer_ + readSizeInWord);
            return false;
        }
    }

    // VBlank のラインを数える
    int vblankLines = 0;
    while (1)
    {
        auto getNextSAV = [&]() -> int
        {
            switch (nextCode)
            {
            case getCodeByte(EAV_VSYNC_F0):
                return SAV_VSYNC_F0;

            case getCodeByte(EAV_VSYNC_F1):
                return SAV_VSYNC_F1;
            };
            return 0;
        };

        auto sav = getNextSAV();
        if (!sav)
        {
            break;
        }

        lineCodeLog[analyzeLine++] = nextCode;

        startVideoCapture(buffer_, sav, readSizeInWord);
        waitVideoCapture();

        nextCode = getEAVCode();
        if (nextCode < 0)
        {
            printf("error: no timing code (vsync).\n");
            return false;
        }

        ++vblankLines;

        if (vblankLines > MAX_VBLANK_LINES)
        {
            printf("error: vblank line over.\n");
            return false;
        }
    }
    auto t1 = util::getSysTickCounter24();

    lineCodeLog[analyzeLine++] = nextCode;

    // 次のactive区間
    bool interlaced = false;
    if (nextCode == getCodeByte(EAV_ACTIVE_F1))
    {
        interlaced = true;
    }
    else if (nextCode == getCodeByte(EAV_ACTIVE_F0))
    {
        interlaced = false;
    }
    else
    {
        printf("error: unkown frame format: EAV %02x, %d, %d.\n", nextCode, activeLinesF0, vblankLines);
        return false;
    }

    activeWidth_ = activeSize >> 1;
    dataWidthInWords_ = readSizeInWord;
    hBlankSizeInBytes_ = hBlankSize;
    activeLines_ = activeLinesF0;
    vBlankLines_ = vblankLines;
    interlace_ = interlaced;
    vSyncIntervalCycles_ = (t0 - t1) & 0xffffff;
    signalDetected_ = true;

    // activeLineOfs_ = (activeLines_ - DISPLAY_HEIGHT) >> 1;
    activeLineOfs_ = 0;
    currentField_ = 0;

    printf("Data width  : %d.\n", activeWidth_);
    printf("H blank     : %d.%d.\n", hBlankSize >> 1, hBlankSize & 1 ? 5 : 0);
    printf("Active line : %d\n", activeLinesF0);
    printf("V blank line: %d\n", vblankLines);
    printf("Interlace   : %s\n", interlaced ? "true" : "false");
    printf("V sync cycle: %d\n", vSyncIntervalCycles_);
    printf("FPS         : %f\n", (float)CPU_CLOCK_KHZ * 1000 / vSyncIntervalCycles_);

    for (int i = 0; i < analyzeLine; ++i)
    {
        printf("%2d: %02x\n", i, lineCodeLog[i]);
    }
    return true;
}

void VideoCapture::simpleCaptureTest()
{
    startVideoCapture(buffer_, EAV_VSYNC_F0, 1);
    waitVideoCapture();

    startVideoCapture(buffer_, EAV_ACTIVE_F0, TOTAL_BUFFER_SIZE);
    waitVideoCapture();
    printf("Active line:\n");
    util::dump(std::begin(buffer_), std::end(buffer_));

    startVideoCapture(buffer_, EAV_ACTIVE_F0, 1);
    waitVideoCapture();
    startVideoCapture(buffer_, EAV_VSYNC_F0, TOTAL_BUFFER_SIZE);
    waitVideoCapture();
    printf("VSync line:\n");
    util::dump(std::begin(buffer_), std::end(buffer_));
}

void VideoCapture::imageConvertTest()
{
    for (int i = 0; i < 720 / 2; ++i)
        //            buffer_[i] = 0x01900080 + (0x02020202 * i);
        buffer_[i] = 0x01800080 + (0x02000200 * i);

    uint32_t tmp[320];
    graphics::resizeYCbCr420(tmp, 320, buffer_, 720);

    printf("src:\n");
    util::dump((uint16_t *)buffer_, (uint16_t *)buffer_ + 720, "%04x ");
    printf("scaled:\n");
    util::dump(tmp, tmp + 320);

    uint16_t tmp2[320];
    graphics::convertYCbCr2RGB565(tmp2, tmp, 320);

    printf("rgb:\n");
    util::dumpF(tmp2, tmp2 + 320, [](int v)
                { printf("(%2d, %2d, %2d) ", (v >> 11) & 31, (v >> 5) & 63, v & 31); });
}
