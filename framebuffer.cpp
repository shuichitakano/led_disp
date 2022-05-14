/*
 * author : Shuichi TAKANO
 * since  : Sun May 01 2022 04:20:04
 */

#include "framebuffer.h"
#include <hardware/sync.h>
#include <mutex>
#include <assert.h>
#include <numeric>

namespace graphics
{
    void
    FrameBuffer::initialize(int w, int h, int margin)
    {
        nLines_ = h + margin;
        width_ = w;
        height_ = h;

        pixelBuffer_.resize(w * nLines_);
        freeLines_.resize(nLines_);
        std::iota(freeLines_.begin(), freeLines_.end(), 0);

        for (auto &v : planes_)
        {
            v.reserve(h);
        }
    }

    uint16_t *
    FrameBuffer::getLineBuffer(int lineID)
    {
        assert(lineID < nLines_);
        return pixelBuffer_.data() + width_ * lineID;
    }

    int
    FrameBuffer::allocateLine()
    {
        while (1)
        {
            {
                std::lock_guard lock(freeLock_);
                if (!freeLines_.empty())
                {
                    int r = freeLines_.back();
                    freeLines_.pop_back();
                    return r;
                }
            }
            __wfe();
        }
    }

    void
    FrameBuffer::freeLine(const std::initializer_list<int> &indices)
    {
        {
            std::lock_guard lock(freeLock_);
            freeLines_.insert(freeLines_.begin(),
                              indices.begin(), indices.end());
        }
        __sev();
    }

    void
    FrameBuffer::commitNextLine(int lineID)
    {
        {
            std::lock_guard lock(planeLock_);
            // printf("commit %d, %zd\n", lineID, getWritePlane().size());
            getWritePlane().push_back(lineID);
        }
        __sev();
    }

    // 指定ラインを取り出し
    int
    FrameBuffer::moveCurrentPlaneLine(int y)
    {
        while (1)
        {
            {
                std::lock_guard lock(planeLock_);
                auto &plane = getReadPlane();
                if (y < plane.size())
                {
                    auto r = plane[y];
                    plane[y] = -1;
                    // printf("move %d\n", y);
                    assert(r >= 0);
                    return r;
                }
            }
            __wfe();
        }
    }

    void
    FrameBuffer::finishPlane()
    {
        {
            std::lock_guard lock(planeLock_);
            if (++writePlaneID_ == N_PLANE_BUFFERS)
            {
                writePlaneID_ = 0;
            }
            // printf("finishPlane: w%d, r%d, %zd\n", writePlaneID_, readPlaneID_, getWritePlane().size());
            // Planeを初期化するのは読み取り側の責任
            assert(getWritePlane().empty());
        }
        __sev();
    }

    void
    FrameBuffer::flipReadPlane()
    {
        std::lock_guard lock(planeLock_);
        auto &plane = getReadPlane();
        {
            std::lock_guard lock(freeLock_);
            for (auto i : plane)
            {
                if (i >= 0)
                {
                    freeLines_.push_back(i);
                }
            }
            plane.clear();
        }
        __sev();

        if (++readPlaneID_ == N_PLANE_BUFFERS)
        {
            readPlaneID_ = 0;
        }
        // printf("flipReadPlane: w%d, r%d\n", writePlaneID_, readPlaneID_);
    }
}
