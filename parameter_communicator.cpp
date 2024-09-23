/*
 * author : Shuichi TAKANO
 * since  : Mon Sep 09 2024 00:02:53
 */

#include "parameter_communicator.h"

#include <hardware/gpio.h>
#include "util.h"
#include "remote_video_capture.h"
#include "video_capture.h"
#include <cassert>

namespace
{

#define PARAM_ID(m1, m2) m1##_##m2
#define MEMBER(m1, m2) m1.m2
#define EVAL(f, m1, m2) f(PARAM_ID(m1, m2), MEMBER(m1, m2))

#define EVAL_SYSPARAMS(f)                      \
    EVAL(f, ledDisplay, imageSize);            \
    EVAL(f, ledDisplay, displayOffset);        \
    EVAL(f, videoCapture, samplingOffset);     \
    EVAL(f, videoCapture, samplingWidth);      \
    EVAL(f, videoCapture, samplingOffsetFrac); \
    EVAL(f, videoCapture, samplingWidthFrac);  \
    EVAL(f, videoStatus, flags);               \
    EVAL(f, root, signalInput)

    inline constexpr int CODEOFS_VIDEOCAPTURE = 0xc0;
    inline constexpr int COMMAND_RESET = 0xff;

    union ResultByte
    {
        uint8_t byte;

        struct
        {
            uint8_t data : 7;
            uint8_t busy : 1;
        };
    };
}

int getI2CSlaveAddress(I2CTargetType type)
{
    switch (type)
    {
    case I2CTargetType::VIDEO_BOARD_MASTER:
        assert(0);
        return 0;

    case I2CTargetType::VIDEO_BOARD_SLAVE:
        return 0x11;

    case I2CTargetType::LED_PANEL_DRIVER_LEFT:
        return 0x14;

    case I2CTargetType::LED_PANEL_DRIVER_RIGHT:
        return 0x15;
    }
    return -1;
}

// ADV7181: (0x20), 0x21
// PCA9554C: (0x38), 0x3c
// () が slave

/////////////////////////////////////

namespace
{
    SlaveParameterCommunicator *slaveParameterCommunicator_{};

    void i2cSlaveHandler(i2c_inst_t *i2c,
                         i2c_slave_event_t event)
    {
        slaveParameterCommunicator_->onI2CEvent(event);
    }
}

void SlaveParameterCommunicator::init(i2c_inst_t *i2c, int slaveAddr,
                                      int pinSDA, int pinSCL)
{
    i2c_ = i2c;
    slaveAddr_ = slaveAddr;

    gpio_init(pinSDA);
    gpio_set_function(pinSDA, GPIO_FUNC_I2C);
    gpio_pull_up(pinSDA);

    gpio_init(pinSCL);
    gpio_set_function(pinSCL, GPIO_FUNC_I2C);
    gpio_pull_up(pinSCL);

    i2c_init(i2c, 400000);

    slaveParameterCommunicator_ = this;
    i2c_slave_init(i2c, slaveAddr, &i2cSlaveHandler);
}

void SlaveParameterCommunicator::onI2CEvent(i2c_slave_event_t event)
{
    switch (event)
    {
    case I2C_SLAVE_RECEIVE:
    {
        int data = i2c_read_byte_raw(i2c_);
        if (commandReceived_)
        {
            assert(command_ < CODEOFS_VIDEOCAPTURE);
            getBuffer()[receivePointer_++] = data;
        }
        else
        {
            receivePointer_ = 0;
            command_ = data;
            commandReceived_ = true;
        }
    }
    break;

    case I2C_SLAVE_REQUEST:
    {
        ResultByte result{};
        result.busy = 0;

        if (command_ == COMMAND_RESET)
        {
            DBGPRINT("Reset.\n");
            if (remoteVideoCaptureManager_)
            {
                remoteVideoCaptureManager_->reset();
            }
            i2c_write_byte_raw(i2c_, result.byte);
        }
        else if (command_ >= CODEOFS_VIDEOCAPTURE)
        {
            if (!remoteVideoCaptureManager_ ||
                remoteVideoCaptureManager_->isBusy())
            {
                // DBGPRINT("busy\n");
                result.busy = 1;
            }
            else
            {
                auto capture = remoteVideoCaptureManager_->getVideoCapture();
                assert(capture);

                result.data = 1;

                auto req = static_cast<VideoCaptureRequest>(command_ - CODEOFS_VIDEOCAPTURE);
                switch (req)
                {
                case VideoCaptureRequest::START_CAPTURE:
                    capture->startBGCapture();
                    break;

                case VideoCaptureRequest::STOP_CAPTURE:
                    capture->stopBGCapture();
                    break;

                case VideoCaptureRequest::ANALYZE_SIGNAL:
                    result.data = capture->analyzeSignal();
                    break;

                default:
                    // DBGPRINT("request: %d\n", (int)req);
                    remoteVideoCaptureManager_->request(req);
                    break;
                }
            }
            i2c_write_byte_raw(i2c_, result.byte);
        }
        else
        {
            assert(0);
        }
    }
    break;

    case I2C_SLAVE_FINISH:
        onParameterReceived();
        commandReceived_ = false;
        break;
    }
}

void SlaveParameterCommunicator::onParameterReceived()
{
    size_t size = receivePointer_;
    // DBGPRINT("command: %d, size: %d\n", command_, size);
    switch (command_)
    {
    };
}

/////////////////////////////////////

// Slave 側が reset されたことを検知できるといいよね…
// (Device を再初期化しないといけない)

void MasterParameterCommunicator::send(int slaveAddr, uint8_t code, const void *data, int sizeInBytes)
{
    i2c_write_blocking(i2c_, slaveAddr, &code, 1, true);
    i2c_write_blocking(i2c_, slaveAddr, reinterpret_cast<const uint8_t *>(data), sizeInBytes, false);
}

int MasterParameterCommunicator::sendCommand(int slaveAddr, uint8_t code) const
{
    ResultByte result{};
    do
    {
        i2c_write_blocking(i2c_, slaveAddr, &code, 1, true);
        i2c_read_blocking(i2c_, slaveAddr, &result.byte, 1, false);
        if (!result.busy)
        {
            break;
        }
        DBGPRINT("slave is busy (code = %d). retry.\n", (int)code);
    } while (1);
    return result.data;
}

int MasterParameterCommunicator::sendRequest(VideoCaptureRequest req) const
{
    uint8_t code = CODEOFS_VIDEOCAPTURE + static_cast<int>(req);
    return sendCommand(getI2CSlaveAddress(I2CTargetType::VIDEO_BOARD_SLAVE), code);
}

void MasterParameterCommunicator::requestReset(I2CTargetType target) const
{
    sendCommand(getI2CSlaveAddress(target), COMMAND_RESET);
}
