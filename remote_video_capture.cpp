/*
 * author : Shuichi TAKANO
 * since  : Sun Sep 15 2024 01:41:54
 */

#include "remote_video_capture.h"

#include "video_capture.h"
#include "util.h"
#include <cassert>

namespace video
{

    void
    RemoteVideoCaptureManager::loop()
    {
        DBGPRINT("RemoteVideoCaptureManager: enter loop\n");
        bool led = isBusy();
        while (1)
        {
            auto req = req_;

            if (req == VideoCaptureRequest::NONE)
            {
                continue;
            }

            bool doAfterCapture = false;

            switch (req)
            {
            case VideoCaptureRequest::CAPTURE_FRAME:
                led ^= true;
                gpio_put(PICO_DEFAULT_LED_PIN, led);

                videoCapture_->captureFrame();
                doAfterCapture = true;
                break;

            case VideoCaptureRequest::WAIT_CAPTURE:
                // ここに来れたときは終わっている
                break;

            default:
                DBGPRINT("invalid request: %d\n", static_cast<int>(req));
                assert(0);
                break;
            }
            req_ = VideoCaptureRequest::NONE;

            if (doAfterCapture)
            {
                if (afterCaptureFunc_)
                {
                    afterCaptureFunc_();
                }
            }
        }
    }

    bool
    RemoteVideoCaptureManager::request(VideoCaptureRequest req)
    {
        if (isBusy())
        {
            return false;
        }
        req_ = req;
        return true;
    }

    bool
    RemoteVideoCaptureManager::reset()
    {
        if (isBusy())
        {
            return false;
        }
        videoCapture_->stopBGCapture();
        return true;
    }
}
