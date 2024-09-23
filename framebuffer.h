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

        uint16_t *__not_in_flash_func(getLineBuffer)(int lineID);

        int __not_in_flash_func(allocateLine)();
        void __not_in_flash_func(commitNextLine)(int lineID);
        void fillBlackPlane();

        void finishPlane();

        void waitForCurrentPlaneLineReady(int y);
        int __not_in_flash_func(moveCurrentPlaneLine)(int y);
        void __not_in_flash_func(freeLine)(const std::initializer_list<int> &indices);

        std::tuple<uint16_t *, bool> peekCurrentPlaneLineUnsafe(int y);
        uint16_t *getWritePlaneLineUnsafe(int y);

        void flipReadPlane();

        size_t getFreeLineCount() const { return freeLines_.size(); }

        void dumpState() const;
        void requestDump() { dumpReq_ = true; }

        int getWidth() const { return width_; }
        int getHeight() const { return height_; }

        int getWritePlaneID() const { return writePlaneID_; }
        int getReadPlaneID() const { return readPlaneID_; }

    protected:
        Plane &getWritePlane() { return planes_[writePlaneID_]; }
        Plane &getReadPlane() { return planes_[readPlaneID_]; }
        const Plane &getReadPlane() const { return planes_[readPlaneID_]; }

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

        bool dumpReq_ = false;
    };
}
