/*
 * author : Shuichi TAKANO
 * since  : Fri Sep 29 2023 10:44:25
 */
#pragma once

#include <stdint.h>
#include <hardware/i2c.h>

namespace device
{
    class TPA2016
    {
    public:
        void init(i2c_inst_t *i2c);
        bool isExist() const { return isExist_; }

        void setControl(bool enableL, bool enableR, bool shutdown, bool noiseGate);
        void setAGCAttack(int v);
        void setAGCRelease(int v);
        void setAGCHoldTime(int v);
        void setAGCControl1(bool enableLimiter,
                            int limiterLevelDBx2 /*-13:18*/,
                            int noiseGateThreshold /* 0:3*/);
        void setAGCControl2(int maxGainDB /*18:30*/,
                            int compressionRatioLog2 /*0:3*/);
        void setGain(int db /*-28:30*/);

    protected:
        void writeValue(uint8_t reg, uint8_t v);

    private:
        i2c_inst_t *i2c_{};
        bool isExist_ = false;
    };
}
