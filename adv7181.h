/*
 * author : Shuichi TAKANO
 * since  : Sun Aug 28 2022 06:32:30
 */
#pragma once

#include <stdint.h>
#include <hardware/i2c.h>
#include "device_def.h"

namespace device
{

    class ADV7181
    {
    public:
        struct STDIState
        {
            bool enabled = false;
            bool interlaced = false;
            uint8_t nLinesInVSync = 0;
            uint16_t nLinesInField = 0;
            uint16_t nCyclesInField_256 = 0;
            uint16_t blockSize = 0; // clock cycles / 8line

            void dump() const;
            friend bool test(const STDIState &a, const STDIState &b, int cycleMarginPerLine);
            friend bool operator==(const STDIState &, const STDIState &);
        };

        struct SSPDState
        {
            bool enabled = false;
            bool isHSActive = false;
            bool isHSNegative = false;
            bool isVSActive = false;
            bool isVSNegative = false;

            void dump() const;
        };

    public:
        bool init(i2c_inst_t *i2cInst, bool master);

        void reset() const;
        void selectInput(SignalInput input);
        SignalInput getCurrentInput() const { return input_; }
        void updateStatus();
        int getStatus1() const { return statusRegs_[0]; }
        int getStatus2() const { return statusRegs_[2]; }
        int getStatus3() const { return statusRegs_[3]; }
        bool isPLLLockedCP() const;
        bool isFreeRunCP() const;
        bool isPLLLockedSDP() const;
        bool isFreeRunSDP() const;
        bool isFreeRun() const { return isSDPMode() ? isFreeRunSDP() : isFreeRunCP(); }

        void clearStatusCache();

        void startSTDILineCountMode() const;
        void stopSTDILineCountMode() const;

        void setLineLength(int l) const;
        void setRGBSyncModeCSync(bool f) const;
        void setPLL(bool manual, bool immediate, int div, int hFreq) const;

        void setChGain(bool manual, int chA, int chB, int chC) const;
        void setChOffset(int chA, int chB, int chC) const;

        bool waitForCounterStable(int timeOutTimeInMS);

        const STDIState &getSTDIState() const { return STDIState_; }
        const SSPDState &getSSPDState() const { return SSPDState_; }

        bool isSDPMode() const { return input_ == SignalInput::COMPOSITE || input_ == SignalInput::S_VIDEO; }

    protected:
        bool sendSingleCommand(uint8_t reg, uint8_t value) const;

    private:
        i2c_inst_t *i2c_{};
        int addr_{};
        SignalInput input_{SignalInput::NONE};

        uint8_t statusRegs_[4]{};
        STDIState STDIState_;
        SSPDState SSPDState_;
    };

}
