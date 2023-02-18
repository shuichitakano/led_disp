/*
 * author : Shuichi TAKANO
 * since  : Fri Feb 10 2023 03:04:23
 */
#pragma once

#include <stdint.h>
#include <array>
#include <tuple>
#include "spinlock.h"

namespace video
{
    class VideoStreamBuffer
    {
    public:
        static constexpr uint32_t UNIT_BUFFER_SIZE = 512;
        static constexpr uint32_t WRITE_RING_SIZE = 8; // 2のベキ
        static constexpr uint32_t N_BUFFERS = 8;
        static constexpr uint32_t TOTAL_BUFFER_SIZE = UNIT_BUFFER_SIZE * N_BUFFERS;

        static_assert(N_BUFFERS <= WRITE_RING_SIZE);
        static_assert((WRITE_RING_SIZE & (WRITE_RING_SIZE - 1)) == 0);

        using RawBuffer = std::array<uint32_t, TOTAL_BUFFER_SIZE>;
        using Buffer = std::array<uint32_t, UNIT_BUFFER_SIZE>;

        struct LineInfo
        {
            uint32_t time;
            uint16_t lineIdx_x2;
            uint8_t code;
            uint8_t error;
        };

    public:
        VideoStreamBuffer()
        {
            reset();
        }

        RawBuffer &getRawBuffer()
        {
            return buffer_;
        }

        const Buffer &getBuffer(int id) const
        {
            return *reinterpret_cast<const Buffer *>(&buffer_[UNIT_BUFFER_SIZE * id]);
        }
        Buffer &getBuffer(int id)
        {
            return *reinterpret_cast<Buffer *>(&buffer_[UNIT_BUFFER_SIZE * id]);
        }

        const LineInfo &getLineInfo(int id) const
        {
            return lineInfos_[id];
        }
        LineInfo &getLineInfo(int id)
        {
            return lineInfos_[id];
        }

        bool __not_in_flash_func(nextRead)();
        void __not_in_flash_func(waitForNextLine)();
        void __not_in_flash_func(nextWrite)();

        std::tuple<const Buffer &, const LineInfo &>
        getCurrentReadLine() const
        {
            return {getBuffer(currentReadIdx_), getLineInfo(currentReadIdx_)};
        }

        std::tuple<Buffer &, LineInfo &>
        getCurrentWriteLine()
        {
            return {getBuffer(currentWriteIdx_), getLineInfo(currentWriteIdx_)};
        }

        void reset();
        bool hasData() const
        {
            return writeIdx0_ != writeIdx1_;
        }

    private:
        RawBuffer buffer_;
        LineInfo lineInfos_[N_BUFFERS];

        std::array<uint8_t, WRITE_RING_SIZE> writtenBufferIdxs_;
        std::array<uint8_t, N_BUFFERS - 2> freeBufferIdxs_;

        volatile int writeIdx0_ = 0; // old
        volatile int writeIdx1_ = 0; // new
        int nFreeBuffers_ = 0;

        int currentReadIdx_ = 0;
        int currentWriteIdx_ = 1;

        // util::SpinLock lock_;
    };
}
