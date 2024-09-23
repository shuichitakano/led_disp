/*
 * author : Shuichi TAKANO
 * since  : Sun May 01 2022 04:20:04
 */

#include "framebuffer.h"

#include <hardware/sync.h>
#include <mutex>
#include <assert.h>
#include <numeric>
#include <cstring>

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
        assert(lineID >= 0 && lineID < nLines_);
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
                    assert(r >= 0);
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
            freeLines_.insert(freeLines_.end(),
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

    void
    FrameBuffer::fillBlackPlane()
    {
        auto &wp = getWritePlane();
        while (wp.size() < height_)
        {
            int lineID = allocateLine();
            auto p = getLineBuffer(lineID);
            memset(p, 0, width_ * 2);
            commitNextLine(lineID);
        }
    }

    uint16_t *
    FrameBuffer::getWritePlaneLineUnsafe(int y)
    {
        assert(y < getWritePlane().size());
        auto i = getWritePlane()[y];
        return i < 0 ? nullptr : getLineBuffer(i);
    }

    std::tuple<uint16_t *, bool>
    FrameBuffer::peekCurrentPlaneLineUnsafe(int y)
    {
        std::lock_guard lock(planeLock_);
        auto &plane = getReadPlane();
        if (y < plane.size())
        {
            int idx = plane[y];
            return {idx >= 0 ? getLineBuffer(idx) : nullptr, true};
        }
        else
        {
            return {nullptr, false};
        }
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
    FrameBuffer::waitForCurrentPlaneLineReady(int y)
    {
        while (1)
        {
            {
                std::lock_guard lock(planeLock_);
                auto &plane = getReadPlane();
                if (y < plane.size())
                {
                    return;
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

            if (dumpReq_)
            {
                dumpState();
                dumpReq_ = false;
            }
        }
        __sev();

        if (++readPlaneID_ == N_PLANE_BUFFERS)
        {
            readPlaneID_ = 0;
        }
        // printf("flipReadPlane: w%d, r%d\n", writePlaneID_, readPlaneID_);
    }

    void
    FrameBuffer::dumpState() const
    {
        printf("FrameBuffer: %d x %d %dlines, free %d\n", width_, height_, nLines_, freeLines_.size());
        printf("  read plane: %d, write plane; %d\n", readPlaneID_, writePlaneID_);
        int n = 0;
        int i = 0;
        for (auto &pl : planes_)
        {
            printf(" plane %d:\n", i);
            int j = 0;
            for (auto v : pl)
            {
                printf("   %d: %d\n", j, v);
                ++j;
                if (v >= 0)
                    ++n;
            }
            ++i;
        }
        printf(" %d used, total %d\n", n, n + freeLines_.size());
    }
}
