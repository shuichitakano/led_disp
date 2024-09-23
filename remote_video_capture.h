/*
 * author : Shuichi TAKANO
 * since  : Sun Sep 15 2024 01:39:57
 */
#pragma once

#include "video_capture_request.h"
#include <functional>

namespace video
{
    class VideoCapture;

    class RemoteVideoCaptureManager
    {
    public:
        RemoteVideoCaptureManager(VideoCapture *videoCapture)
            : videoCapture_(videoCapture)
        {
        }

        void setAfterCaptureFunc(std::function<void()> f) { afterCaptureFunc_ = std::move(f); }

        void loop();

        bool request(VideoCaptureRequest req);
        bool reset();

        VideoCapture *getVideoCapture() { return videoCapture_; }
        bool isBusy() const { return req_ != VideoCaptureRequest::NONE; }

    private:
        VideoCapture *videoCapture_{};
        volatile VideoCaptureRequest req_{};

        std::function<void()> afterCaptureFunc_{};
    };
}
