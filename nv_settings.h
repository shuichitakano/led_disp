/*
 * author : Shuichi TAKANO
 * since  : Mon Oct 09 2023 23:50:46
 */

#pragma once

#include <array>
#include "adv7181.h"
#include "device_def.h"
#include "spinlock.h"
#include <hardware/flash.h>

class NVSettings
{
public:
    struct CaptureSettings
    {
        device::ADV7181::STDIState stdiState; // 10byte

        uint16_t resampleWidth;
        int16_t resampleOfs;
        int8_t vOfs;
        uint8_t resamplePhaseOfs;

    public:
        friend bool operator==(const CaptureSettings &, const CaptureSettings &);
    };
    static_assert(sizeof(CaptureSettings) == 16);
    inline static constexpr int MAX_CAPTURE_SETTINGS = 1024 - FLASH_SECTOR_SIZE / sizeof(CaptureSettings);

    struct State
    {
        inline static constexpr uint32_t MAGIC = 'C' | ('A' << 8) | ('P' << 16) | ('0' << 24);
        inline static constexpr uint32_t CURRENT_VER = 0;
        uint32_t magic = MAGIC;
        uint32_t version = CURRENT_VER;
        uint32_t sum = 0;

        uint16_t nCaptureSettings = 0;
        device::SignalInput latestInput = device::SignalInput::COMPOSITE;
        uint8_t pad1;

        int8_t speakerGain = -28;
        uint8_t infoMode = 0;
    };

    inline static constexpr size_t STATE_SIZE = FLASH_SECTOR_SIZE;
    static_assert(sizeof(State) <= STATE_SIZE);

public:
    const State &getState() const { return state_; }
    const CaptureSettings *findCaptureSettings(const device::ADV7181::STDIState &s);

    void setLatestInput(device::SignalInput v);
    void setSpeakerGain(int v);
    void setInfoMode(int v);
    void setCaptureSetting(const CaptureSettings &v);

    void load();
    void __not_in_flash_func(flash)();
    // void flash();
    void tick();
    bool isDirty() const { return dirty_; }

    static NVSettings &instance();

private:
    State state_;
    bool dirty_ = false;

    CaptureSettings latestCaptureSetting_;
    int latestCaptureSettingIdx_ = -1;

    volatile bool waiting_ = false;
};
