/*
 * author : Shuichi TAKANO
 * since  : Sun May 01 2022 03:30:34
 */
#pragma once

#include <stdint.h>
#include <vector>
#include <initializer_list>
#include "spinlock.h"

namespace graphics
{
    class FrameBuffer
    {
        using Plane = std::vector<int16_t>;

    public:
        void initialize(int w, int h, int margin);

        uint16_t *getLineBuffer(int lineID);

        int allocateLine();
        void commitNextLine(int lineID);
        void finishPlane();

        int moveCurrentPlaneLine(int y);
        void freeLine(const std::initializer_list<int> &indices);

        void flipReadPlane();

        uint16_t *getWritePlaneLineUnsafe(int y);

    protected:
        Plane &getWritePlane() { return planes_[writePlaneID_]; }
        Plane &getReadPlane() { return planes_[readPlaneID_]; }

    private:
        std::vector<uint16_t> pixelBuffer_; // BGR565
        int width_ = 0;
        int height_ = 0;
        size_t nLines_ = 0;

        static constexpr size_t N_PLANE_BUFFERS = 3;
        Plane planes_[N_PLANE_BUFFERS];

        int writePlaneID_ = 0;
        int readPlaneID_ = 0;
        // 更新タイミングがずれることがある
        // 1/4フレーム先行して始められる

        // dmaをirqで止める?
        // 止まってる間クロックを出す

        std::vector<int16_t> freeLines_;

        util::SpinLock freeLock_;
        util::SpinLock planeLock_;
    };
}
