/*
 * author : Shuichi TAKANO
 * since  : Sun Sep 08 2024 23:39:56
 */
#pragma once

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <stdint.h>
#include "system_parameters.h"
#include "video_capture_request.h"

namespace video
{
    class RemoteVideoCaptureManager;
}

enum class I2CTargetType
{
    VIDEO_BOARD_MASTER,
    VIDEO_BOARD_SLAVE,
    LED_PANEL_DRIVER_LEFT,
    LED_PANEL_DRIVER_RIGHT,
};

int getI2CSlaveAddress(I2CTargetType type);

class SlaveParameterCommunicator
{
public:
    void init(i2c_inst_t *i2c, int slaveAddr,
              int pinSDA, int pinSCL);

    void setRemoteVideoCaptureManager(video::RemoteVideoCaptureManager *remoteVideoCaptureManager)
    {
        remoteVideoCaptureManager_ = remoteVideoCaptureManager;
    }

public:
    void onI2CEvent(i2c_slave_event_t event);

protected:
    uint8_t *getBuffer() { return reinterpret_cast<uint8_t *>(buffer_); }
    void onParameterReceived();

private:
    i2c_inst_t *i2c_{};
    uint8_t slaveAddr_{};

    uint8_t receivePointer_{};
    uint8_t command_ = 0;
    bool commandReceived_{};
    uint32_t buffer_[256 / 4]{};

    video::RemoteVideoCaptureManager *remoteVideoCaptureManager_{};
};

class MasterParameterCommunicator
{
public:
    void init(i2c_inst_t *i2c)
    {
        i2c_ = i2c;
    }

    void send(int slaveAddr, uint8_t code, const void *data, int sizeInBytes);
    int sendRequest(VideoCaptureRequest req) const;
    void requestReset(I2CTargetType target) const;

protected:
    int sendCommand(int slaveAddr, uint8_t code) const;

private:
    i2c_inst_t *i2c_{};
};
