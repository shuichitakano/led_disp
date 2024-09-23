/*
 * author : Shuichi TAKANO
 * since  : Wed Aug 28 2024 03:00:57
 */
#pragma once

#include "framebuffer.h"
#include <hardware/pio.h>
#include <vector>

class VideoSerializer
{
    PIO pio_{};
    int sm_{};
    int pinSync_{};

    uint programOfs_{};

public:
    void init(PIO pio, int sm, int pinData0, int pinClk, int pinSync);
    void __not_in_flash_func(sendFrame)(graphics::FrameBuffer &fb);

protected:
    void __not_in_flash_func(sendData)(const uint32_t *data, size_t size) const;
};

void initializeDeserializerProgram(PIO pio);

class VideoDeserializerBase
{
    int sm_{};
    int dmaCh_{};
    int dmaCtrlCh_{};

    int dmaID_{};

    void *nextTransferAddr_{};

    int currentY_{};
    int dbid_ = 0;

    std::vector<uint32_t> receiveBuffer_;
    graphics::FrameBuffer *fb_{};

public:
    virtual ~VideoDeserializerBase() = default;

    void init(int sm, int pin0, int dmaID = 0);
    void __not_in_flash_func(startReceive)(graphics::FrameBuffer *fb);
    void __not_in_flash_func(waitTransfer)();

    void __not_in_flash_func(_finishCurrentLine)();
    void dump() const;

protected:
    virtual void __not_in_flash_func(startFrame)() = 0;
    virtual void __not_in_flash_func(processLine)(uint16_t *line) = 0;
    virtual void __not_in_flash_func(endFrame)() = 0;

    void __not_in_flash_func(setupNextLine)();
    uint32_t *getReceiveBuffer(int dbid)
    {
        return receiveBuffer_.data() + dbid * (receiveBuffer_.size() >> 1);
    }

    graphics::FrameBuffer *getFrameBuffer() { return fb_; }
};

class VideoDeserializer : public VideoDeserializerBase
{
public:
    void setOffset(int offset) { offset_ = offset; }

protected:
    void __not_in_flash_func(startFrame)() override;
    void __not_in_flash_func(processLine)(uint16_t *line) override;
    void __not_in_flash_func(endFrame)() override;

private:
    int committed_{};
    int offset_ = 0;
};

class LineCompositor
{
public:
    LineCompositor(graphics::FrameBuffer *fb) : fb_(fb)
    {
        lineQueue_.reserve(fb->getHeight());
    }
    void __not_in_flash_func(commit)(bool right, uint16_t *data);

    void setLineInfo(bool right, int srcOfs, int dstOfs, int w)
    {
        auto &li = lineInfo_[right];
        li.srcOfs_ = srcOfs;
        li.dstOfs_ = dstOfs;
        li.w_ = w;
    }

    void clear() { committed_ = 0; }
    void finish();

protected:
    graphics::FrameBuffer *getFrameBuffer() { return fb_; }

private:
    graphics::FrameBuffer *fb_{};
    std::vector<int> lineQueue_;
    bool frontRight_{};

    struct LineInfo
    {
        int srcOfs_{};
        int dstOfs_{};
        int w_{};
    };
    LineInfo lineInfo_[2]; // l, r

    int committed_{};
};

class VideoDeserializer2 : public VideoDeserializerBase
{
public:
    VideoDeserializer2(LineCompositor *lineCompositor, bool right)
        : lineCompositor_(lineCompositor), right_(right) {}

protected:
    void __not_in_flash_func(startFrame)() override;
    void __not_in_flash_func(processLine)(uint16_t *line) override;
    void __not_in_flash_func(endFrame)() override;

private:
    LineCompositor *lineCompositor_{};
    bool right_ = false;
};

void serializerDebug();
