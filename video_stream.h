/*
 * author : Shuichi TAKANO
 * since  : Sat Feb 11 2023 13:45:54
 */

#include <stdint.h>
#include <tuple>
#pragma once

namespace video
{
    enum class BT656TimingCode : uint32_t
    {
        INVALID = 0,

        SAV_ACTIVE_F0 = 0x800000ff,
        SAV_ACTIVE_F1 = 0xc70000ff,
        SAV_VSYNC_F0 = 0xab0000ff,
        SAV_VSYNC_F1 = 0xec0000ff,

        EAV_ACTIVE_F0 = 0x9d0000ff,
        EAV_ACTIVE_F1 = 0xda0000ff,
        EAV_VSYNC_F0 = 0xb60000ff,
        EAV_VSYNC_F1 = 0xf10000ff,
    };

    constexpr uint32_t asUInt(BT656TimingCode c) { return static_cast<uint32_t>(c); }
    constexpr uint8_t makeBT656TimingCodeByte(BT656TimingCode c) { return asUInt(c) >> 24; }

    std::tuple<BT656TimingCode, bool>
    findNextEAV(const uint32_t *buffer, uint32_t activeWidthInWords);

    BT656TimingCode getSAVCorrespondingToEAV(BT656TimingCode eav);

}
