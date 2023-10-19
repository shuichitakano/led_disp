/*
 * author : Shuichi TAKANO
 * since  : Fri Sep 29 2023 10:46:39
 */

#include "tpa2016.h"

#include <algorithm>
#include <cstdio>

namespace device
{

    namespace
    {
        constexpr int TPA2016_ADDR = 0xb0 >> 1;
    }

    void
    TPA2016::init(i2c_inst_t *i2c)
    {
        i2c_ = i2c;

        uint8_t data;
        auto r = i2c_read_timeout_us(i2c_, TPA2016_ADDR, &data, 1, false, 1000);
        isExist_ = r == 1;
    }

    void
    TPA2016::writeValue(uint8_t reg, uint8_t v)
    {
        uint8_t data[2] = {reg, v};
        i2c_write_blocking(i2c_, TPA2016_ADDR, data, 2, false);

        i2c_write_blocking(i2c_, TPA2016_ADDR, &reg, 1, true);
        uint8_t tmp = 255;
        i2c_read_blocking(i2c_, TPA2016_ADDR, &tmp, 1, false);
        // printf("send %02x:%02x -> %02x\n", reg, v, tmp);
    }

    void
    TPA2016::setControl(bool enableL, bool enableR, bool shutdown, bool noiseGate)
    {
        int v = (enableL ? 1 << 6 : 0) |
                (enableR ? 1 << 7 : 0) |
                (shutdown ? 1 << 5 : 0) |
                (noiseGate ? 1 << 0 : 0);
        writeValue(1, v);
    }

    void TPA2016::setAGCAttack(int v)
    {
        writeValue(2, std::clamp(v, 0, 63));
    }

    void TPA2016::setAGCRelease(int v)
    {
        writeValue(3, std::clamp(v, 0, 63));
    }

    void TPA2016::setAGCHoldTime(int v)
    {
        writeValue(4, std::clamp(v, 0, 63));
    }

    void TPA2016::setAGCControl1(bool enableLimiter,
                                 int limiterLevelDBx2,
                                 int noiseGateThreshold)
    {
        int v = (enableLimiter ? 0 : 1 << 7) |
                (std::clamp(noiseGateThreshold, 0, 3) << 5) |
                (std::clamp(limiterLevelDBx2 + 13, 0, 31) << 0);
        writeValue(6, v);
    }

    void TPA2016::setAGCControl2(int maxGainDB, int compressionRatioLog2)
    {
        int v = (std::clamp(maxGainDB - 18, 0, 12) << 4) |
                (std::clamp(compressionRatioLog2, 0, 3) << 0);
        writeValue(7, v);
    }

    void TPA2016::setGain(int db)
    {
        int v = std::clamp(db, -28, 30);
        writeValue(5, v & 63);
    }

}
