/*
 * author : Shuichi TAKANO
 * since  : Fri Feb 10 2023 04:04:20
 */

#include "video_stream_buffer.h"
#include <mutex>
#include <numeric>
#include <hardware/divider.h>

namespace video
{
    void
    VideoStreamBuffer::reset()
    {
        std::iota(freeBufferIdxs_.begin(), freeBufferIdxs_.end(), 0);
        nFreeBuffers_ = freeBufferIdxs_.size();

        writeIdx0_ = 0;
        writeIdx1_ = 0;

        currentWriteIdx_ = N_BUFFERS - 2;
        currentReadIdx_ = N_BUFFERS - 1;
    }

    bool
    VideoStreamBuffer::nextRead()
    {
        // std::lock_guard lock(lock_);
        if (hasData())
        {
            auto save = save_and_disable_interrupts();

            freeBufferIdxs_[nFreeBuffers_++] = currentReadIdx_;
            assert(nFreeBuffers_ <= N_BUFFERS - 2);

            currentReadIdx_ = writtenBufferIdxs_[writeIdx0_];
            writeIdx0_ = (writeIdx0_ + 1) & (WRITE_RING_SIZE - 1);

            restore_interrupts(save);
            return true;
        }
        return false;
    }

    void
    VideoStreamBuffer::waitForNextLine()
    {
        while (!nextRead())
        {
            //__wfe();
        }
    }

    void
    VideoStreamBuffer::nextWrite()
    {
        {
            // std::lock_guard lock(lock_);

            writtenBufferIdxs_[writeIdx1_] = currentWriteIdx_;
            writeIdx1_ = (writeIdx1_ + 1) & (WRITE_RING_SIZE - 1);

            if (nFreeBuffers_ > 0)
            {
                currentWriteIdx_ = freeBufferIdxs_[--nFreeBuffers_];
            }
            else
            {
                currentWriteIdx_ = writtenBufferIdxs_[writeIdx0_];
                writeIdx0_ = (writeIdx0_ + 1) & (WRITE_RING_SIZE - 1);
            }
        }
        //__sev();
    }
}
