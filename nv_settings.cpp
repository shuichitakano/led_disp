/*
 * author : Shuichi TAKANO
 * since  : Mon Oct 09 2023 23:55:24
 */

#include "nv_settings.h"

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <numeric>

namespace
{
    constexpr int sumOfs = 3; // 古い値を忘れたい時にincrement

    constexpr size_t TOTAL_SIZE = (NVSettings::STATE_SIZE +
                                   sizeof(NVSettings::CaptureSettings) * NVSettings::MAX_CAPTURE_SETTINGS);

    constexpr uintptr_t getFlashAddr()
    {
        constexpr int ofs = 2 * 1024 * 1024 - TOTAL_SIZE;
        return XIP_BASE + ofs;
    }

    constexpr uintptr_t getCaptureSettingAddr()
    {
        return getFlashAddr() + NVSettings::STATE_SIZE;
    }

    const NVSettings::State &getNVState()
    {
        return *reinterpret_cast<NVSettings::State *>(getFlashAddr());
    }

    const NVSettings::CaptureSettings *getNVCaptureSettingsBegin()
    {
        return reinterpret_cast<NVSettings::CaptureSettings *>(getCaptureSettingAddr());
    }

    template <class T>
    uint32_t computeSum(const T &v)
    {
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
        return std::reduce(p, p + sizeof(T), 0u);
    }
}

const NVSettings::CaptureSettings *
NVSettings::findCaptureSettings(const device::ADV7181::STDIState &s)
{
    auto *p = getNVCaptureSettingsBegin();
    for (auto i = 0u; i < state_.nCaptureSettings; ++i)
    {
        if (p->stdiState == s)
        {
            return p;
        }
        ++p;
    }
    return nullptr;
}

void NVSettings::setLatestInput(device::SignalInput v)
{
    if (state_.latestInput != v)
    {
        state_.latestInput = v;
        dirty_ = true;
    }
}

void NVSettings::setSpeakerGain(int v)
{
    if (state_.speakerGain != v)
    {
        state_.speakerGain = v;
        dirty_ = true;
    }
}

void NVSettings::setInfoMode(int v)
{
    if (state_.infoMode != v)
    {
        state_.infoMode = v;
        dirty_ = true;
    }
}

void NVSettings::setCaptureSetting(const CaptureSettings &v)
{
    latestCaptureSetting_ = v;
    latestCaptureSettingIdx_ = -1;

    if (auto *p = findCaptureSettings(v.stdiState))
    {
        if (*p == v)
        {
            return;
        }
        latestCaptureSettingIdx_ = p - getNVCaptureSettingsBegin();
        dirty_ = true;
        return;
    }

    latestCaptureSettingIdx_ = state_.nCaptureSettings;
    // nCaptureSettings を増やすのは書き込む時

    dirty_ = true;
}

void NVSettings::load()
{
    auto nvs = getNVState();
    auto sum = nvs.sum;
    nvs.sum = sumOfs;
    if (nvs.magic != State::MAGIC || sum != computeSum(nvs))
    {
        printf("Invalid NVState.\n");
        return;
    }

    state_ = nvs;
}

void NVSettings::flash()
{
    if (!dirty_)
    {
        return;
    }

    auto save = save_and_disable_interrupts();

    assert(!waiting_);
    waiting_ = true;

    do
    {
        __wfe();
    } while (waiting_);

    restore_interrupts(save);
    dirty_ = false;
}

void NVSettings::tick()
{
    if (waiting_)
    {
        auto save = save_and_disable_interrupts();

        if (latestCaptureSettingIdx_ >= 0)
        {
            std::vector<uint8_t> tmp(FLASH_SECTOR_SIZE);

            size_t ofs = latestCaptureSettingIdx_ * sizeof(CaptureSettings);
            size_t sector = ofs / FLASH_SECTOR_SIZE;

            auto dstAddr = getCaptureSettingAddr() + sector * FLASH_SECTOR_SIZE;

            printf("write capture setting. idx %d, sector %d, addr %x\n",
                   latestCaptureSettingIdx_, sector, dstAddr);

            memcpy(tmp.data(), reinterpret_cast<const void *>(dstAddr), FLASH_SECTOR_SIZE);
            memcpy(tmp.data() + (ofs & (FLASH_SECTOR_SIZE - 1)), &latestCaptureSetting_, sizeof(CaptureSettings));

            flash_range_erase(dstAddr - XIP_BASE, FLASH_SECTOR_SIZE);
            flash_range_program(dstAddr - XIP_BASE, tmp.data(), FLASH_SECTOR_SIZE);

            if (state_.nCaptureSettings <= latestCaptureSettingIdx_)
            {
                state_.nCaptureSettings = latestCaptureSettingIdx_ + 1;
            }

            latestCaptureSettingIdx_ = -1;
        }

        // ヘッダは必ず書く
        printf("write header. %x\n", getFlashAddr());
        state_.sum = sumOfs;
        state_.sum = computeSum(state_);
        flash_range_erase(getFlashAddr() - XIP_BASE, STATE_SIZE);
        flash_range_program(getFlashAddr() - XIP_BASE, reinterpret_cast<const uint8_t *>(&state_), STATE_SIZE);

        printf("done.\n%d capture settings.\n", state_.nCaptureSettings);

        restore_interrupts(save);
        waiting_ = false;
    }
    __sev();
}

NVSettings &NVSettings::instance()
{
    static NVSettings inst;
    return inst;
}

///
bool operator==(const NVSettings::CaptureSettings &a, const NVSettings::CaptureSettings &b)
{
    return a.stdiState == b.stdiState &&
           a.resampleWidth == b.resampleWidth &&
           a.resampleOfs == b.resampleOfs &&
           a.vOfs == b.vOfs &&
           a.resamplePhaseOfs == b.resamplePhaseOfs;
}
