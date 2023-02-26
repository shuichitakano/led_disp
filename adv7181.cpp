/*
 * author : Shuichi TAKANO
 * since  : Sun Aug 28 2022 06:34:11
 */

#include "adv7181.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <tuple>
#include "util.h"

namespace
{
    // ##CP RGB SOG 525i & 625i##
    // :525I RGB SOG In 8Bit 422 EAV/SAV out Encoder:
    static constexpr std::array<uint8_t, 2> i2cDataRGB_CP_[] = {
        {0x05, 0x00}, // Prim_Mode =000b for SD-M
        {0x06, 0x0A}, // VID_STD=1010b for SD 4x1 525i
        //{0x06, 0x0B}, // VID_STD=1011b for SD 4x1 625i
        //{0x06, 0x0C}, // VID_STD=1010b for SD 1x1 525i
        //{0x06, 0x0E}, // VID_STD=1010b for SD 2x1 525i
        {0x1D, 0x47}, // Enable 28MHz Crystal
        {0x3A, 0x11}, // Set Latch Clock 01b, Power Down ADC3
        //{0x3A, 0x21}, // Set Latch Clock 10b (>55MHz), Power Down ADC3
        {0x3B, 0x81}, // Enable internal Bias
        {0x3C, 0x52}, // PLL_QPUMP to 010b
        //{0x3C, 0x32}, // PLL_QPUMP to 010b, sync lv 56.25mv
        {0x52, 0x00}, // Colour Space Conversion from RGB->YCrCb
        {0x53, 0x00}, // CSC
        {0x54, 0x12}, // CSC
        {0x55, 0x90}, // CSC
        {0x56, 0x38}, // CSC
        {0x57, 0x69}, // CSC
        {0x58, 0x48}, // CSC
        {0x59, 0x08}, // CSC
        {0x5A, 0x00}, // CSC
        {0x5B, 0x75}, // CSC
        {0x5C, 0x21}, // CSC
        {0x5D, 0x00}, // CSC
        {0x5E, 0x1A}, // CSC
        {0x5F, 0xB8}, // CSC
        {0x60, 0x08}, // CSC
        {0x61, 0x00}, // CSC
        {0x62, 0x20}, // CSC
        {0x63, 0x03}, // CSC
        {0x64, 0xD7}, // CSC
        {0x65, 0x19}, // CSC
        {0x66, 0x48}, // CSC last
        {0x67, 0x13}, // DPP Filters
        {0x6B, 0xC3}, // Select 422 8 bit YPrPb out from CP
        {0x73, 0xCF}, // Enable Manual Gain and set CH_A gain
        {0x74, 0xA3}, // Set CH_A and CH_B Gain - 0FAh
        {0x75, 0xE8}, // Set CH_B and CH_C Gain
        {0x76, 0xFA}, // Set CH_C Gain
        {0x7B, 0x06}, // clears the bits CP_DUP_AV and AV_Blank_EN
        {0x85, 0x19}, // Turn off SSPD and force SOY.
        //{0x85, 0x11}, // Turn off SSPD and force CSync.
        //{0x85, 0x91}, // Turn off SSPD and force CSync, neg pol.
        //{0x85, 0xb1}, // Turn off SSPD and force CSync, pos pol.
        //{0x85, 0x02}, // Turn on SSPD.
        {0x86, 0x0B}, // Enable stdi_line_count_mode
        {0x8F, 0x77}, // FR_LL to 1820 & Enable 28.63MHz LLC
        {0x90, 0x1C}, // FR_LL to 1820
        // {0xBF, 0x06}, // Blue Screen Free Run Colour
        {0xBF, 0x00}, // Blue Screen Free Run Colour
        {0xC0, 0x40}, // default color
        {0xC1, 0xF0}, // default color
        {0xC2, 0x80}, // Default color
        {0xC5, 0x01}, // CP_CLAMP_AVG_FACTOR[1-0] = 00b
        {0xC9, 0x0C}, // Enable DDR Mode
        //{0xF3, 0x07}, // Enable Anti Alias Filter on ADC 0,1,2
        {0xF3, 0x00}, // Disable Anti Alias Filter
#if 1
        {0x0E, 0x80}, // ADI Recommended Setting
        {0x52, 0x46}, // ADI Recommended Setting
        {0x54, 0x80}, // ADI Recommended Setting
        {0xF6, 0x3B}, // ADI Recommended Setting
        {0x0E, 0x00}, // ADI Recommended Setting
#else
        {0x05, 0x01}, // Comp
        {0x06, 0x00}, // 525i
        {0x3C, 0x53}, // PLL_QPUMP to 011b
        {0xB7, 0x1B}, // ADI Recommended
        {0x0E, 0x80}, // ADI recommended sequence
        {0x52, 0x46}, // ADI recommended sequence
        {0x54, 0x80}, // ADI Recommended Setting
        {0xF6, 0x3B}, // ADI Recommended Setting
        {0x0E, 0x00}, // ADI recommended sequence
#endif
        {0xC3, 0x56}, // ADC1 to Ain8 (Pr), ADC0 to Ain10 (Y),
        {0xC4, 0xF4}, // ADC2 to Ain6 (Pb) and enables manual override of mux, SOG
        // {0xC3, 0x46}, // ADC1 to Ain6 (Pr), ADC0 to Ain10 (Y),
        // {0xC4, 0xF5}, // ADC2 to Ain8 (Pb) and enables manual override of mux, SOG
        {0x69, 0x40}, // 1.0v sync
        {0xb3, 0x51}, // free run th
        //{0xf4, 0x2a}, // drive strength
        {},
    };

    // ##CP YPrPb 525i & 625i##
    // :525I YPrPb In 8Bit 422 EAV/SAV out Encoder:
    static constexpr std::array<uint8_t, 2> i2cDataComponentCP_[] = {
        {0x05, 0x00}, // Prim_Mode =000b for SD-M
        {0x06, 0x0A}, // VID_STD=1010b for SD 4x1 525i
        {0x1D, 0x47}, // Enable 28MHz Crystal
        {0x3A, 0x11}, // Set Latch Clock 01b, Power Down ADC3
        {0x3B, 0x81}, // Enable internal Bias
        {0x3C, 0x52}, // PLL_QPUMP to 010b
        {0x67, 0x13}, // DPP Filters
        {0x6B, 0xC3}, // Select 422 8 bit YPrPb out from CP
        {0x7B, 0x06}, // clears the bits CP_DUP_AV and AV_Blank_EN
        {0x85, 0x19}, // Turn off SSPD and force SOY. For Eval Board.
        {0x86, 0x0B}, // Enable stdi_line_count_mode
        {0x8F, 0x77}, // FR_LL to 1820 & Enable 28.63MHz LLC
        {0x90, 0x1C}, // FR_LL to 1820
        {0xC5, 0x01}, // CP_CLAMP_AVG_FACTOR[1-0] = 00b
        {0xC9, 0x0C}, // Enable DDR Mode
        {0xF3, 0x01}, // Enable Anti Alias Filter on ADC 0
        {0x0E, 0x80}, // ADI Recommended Setting
        {0x52, 0x46}, // ADI Recommended Setting
        {0x54, 0x80}, // ADI Recommended Setting
        {0xF6, 0x3B}, // ADI Recommended Setting
        {0x0E, 0x00}, // ADI Recommended Setting
        {0xC3, 0x31}, // ADC1 to Ain5 (Pr), ADC0 to Ain2 (Y),
        {0xC4, 0xb2}, // ADC2 to Ain3 (Pb) and enables manual override of mux, SOY
        {},
    };

    static constexpr int I2CADDR_CTRL = 0x42 >> 1;
    static constexpr int I2CADDR_VBI = 0x23 >> 1;
}

namespace device
{
    void
    ADV7181::reset() const
    {
        assert(i2c_);
        static constexpr std::array<uint8_t, 2> data = {0x0f, 0x80};
        auto r = i2c_write_blocking(i2c_, I2CADDR_CTRL, data.data(), data.size(), false);
        assert(r >= 0);

        sleep_ms(5);

        // どうもリセット後の最初のコマンドは失敗するようなので適当な書き込みしておく
        uint8_t reg_st = 0x10;
        i2c_write_timeout_us(i2c_, I2CADDR_CTRL, &reg_st, 1, false, 1000);
    }

    void
    ADV7181::selectInput(SignalInput input)
    {
        assert(i2c_);
        if (input_ == input)
        {
            return;
        }

        auto i2cData = [&]() -> const std::array<uint8_t, 2> *
        {
            switch (input)
            {
            case SignalInput::COMPONENT:
                return i2cDataComponentCP_;

            case SignalInput::RGB21:
                return i2cDataRGB_CP_;
            };
            return {};
        }();

        assert(i2cData);
        if (i2cData)
        {
            printf("sending ADV7181 data %p...\n", i2cData);

            reset();

            int sendCount = 0;
            while (true)
            {
                auto data = *i2cData++;
                if (!data[0])
                {
                    break;
                }
                auto r = i2c_write_blocking(i2c_, I2CADDR_CTRL, data.data(), data.size(), false);
                // printf("r = %d [%02x %02x]\n", r, data[0], data[1]);
                assert(r >= 0);
                sendCount += r;
            }

            printf("done. (%d bytes)\n", sendCount);
        }

        startSTDILineCountMode();

        input_ = input;
    }

    bool ADV7181::sendSingleCommand(uint8_t reg, uint8_t value) const
    {
        uint8_t data[] = {reg, value};
        return i2c_write_blocking(i2c_, I2CADDR_CTRL, data, 2, false) == 2;
    }

    void ADV7181::startSTDILineCountMode() const
    {
        sendSingleCommand(0x86, 0x0b);
    }

    void ADV7181::stopSTDILineCountMode() const
    {
        sendSingleCommand(0x86, 0x09); // 一回 off
    }

    void ADV7181::setLineLength(int l) const
    {
        sendSingleCommand(0x8f, 0x70 | (l >> 8)); // LLC_PAD_SEL
        sendSingleCommand(0x90, l);
    }

    void ADV7181::setRGBSyncModeCSync(bool f) const
    {
        sendSingleCommand(0x85, f ? 0x11 : 0x19);
    }

    bool ADV7181::isPLLLocked() const
    {
        return getStatus2() & 0x80;
    }

    bool ADV7181::isFreeRun() const
    {
        return getStatus2() & 0x40;
    }

    void ADV7181::setPLL(bool manual, bool immediate, int div, int hFreq) const
    {
        div *= 4;
        // div *= 2;

        int pixFreq = div * hFreq;
        auto [vcoRangeCode, postDiv] = [&]() -> std::tuple<int, int>
        {
            if (pixFreq < 30000000)
            {
                return {0b00, 6};
            }
            else if (pixFreq < 45000000)
            // else if (pixFreq < 60000000)
            {
                return {0b01, 4};
            }
            else if (pixFreq < 90000000)
            {
                return {0b10, 2};
            }
            return {0b11, 1};
        }();

        constexpr float SR = 11.0f;
        // constexpr float SR = 8.0f;
        float f1 = hFreq * (1 / 1000.0f * 3.14f * 2 / SR);
        auto qpump = static_cast<int>(f1 * f1 * div * postDiv * (0.082f / 310));
        auto qpumpCode = [&]
        {
            constexpr int thresQP[] = {75, 125, 200, 300, 425, 625, 1000};
            int i = 0;
            for (; i < 7; ++i)
            {
                if (qpump < thresQP[i])
                {
                    break;
                }
            }
            return i;
        }();

        printf("div %d, h %d, vco %d, qpump %d:%d %fMHz\n", div, hFreq, vcoRangeCode, qpump, qpumpCode, pixFreq / 1000000.0f);

        int ctrl1 = (manual ? 0x80 : 0) | 0b1100000 | (immediate ? 0 : 0x10) | (div >> 8);
        int ctrl2 = div;
        int ctrl4 = 0x80 | (vcoRangeCode << 5) | 0x10;
        // defaultDiv = 0b1101011010 = 858;

        // SOG SYNC LEVEl: 0x50 = 93.75mv
        int tllcCtrl = 0x50 | qpumpCode;

        sendSingleCommand(0x87, ctrl1);
        sendSingleCommand(0x88, ctrl2);
        sendSingleCommand(0x8a, ctrl4);
        sendSingleCommand(0x3c, tllcCtrl);
    }

    void ADV7181::updateStatus()
    {
        assert(i2c_);
        {
            uint8_t reg = 0x10;
            // uint8_t reg = 0xb5;
            i2c_write_blocking(i2c_, I2CADDR_CTRL, &reg, 1, true);
            i2c_read_blocking(i2c_, I2CADDR_CTRL, statusRegs_, 4, false);
        }
        {
            {
                uint8_t reg = 0xb1;
                uint8_t buf[5];
                i2c_write_blocking(i2c_, I2CADDR_CTRL, &reg, 1, true);
                i2c_read_blocking(i2c_, I2CADDR_CTRL, buf, 5, false);

                {
                    auto &s = STDIState_;
                    s.enabled = buf[0] & 0x80;
                    s.interlaced = buf[0] & 0x40;
                    s.blockSize = ((buf[0] & 0x3f) << 8) + buf[1];
                    s.nLinesInVSync = buf[2] >> 3;
                    s.nLinesInField = ((buf[2] & 7) << 8) + buf[3];
                }
                {
                    auto &s = SSPDState_;
                    s.enabled = buf[4] & 0x80;
                    s.isHSNegative = buf[4] & 0x08;
                    s.isHSActive = buf[4] & 0x10;
                    s.isVSNegative = buf[4] & 0x20;
                    s.isVSActive = buf[4] & 0x40;
                }
            }
            {
                uint8_t reg = 0xca;
                uint8_t buf[2];
                i2c_write_blocking(i2c_, I2CADDR_CTRL, &reg, 1, true);
                i2c_read_blocking(i2c_, I2CADDR_CTRL, buf, 2, false);
                STDIState_.nCyclesInField_256 = ((buf[0] & 0x1f) << 8) + buf[1];
            }
        }
    }

    void ADV7181::clearStatusCache()
    {
        STDIState_.enabled = false;
        SSPDState_.enabled = false;
    }

    void ADV7181::STDIState::dump() const
    {
        printf("STDI: %s\n", enabled ? "enabled" : "disabled");
        if (enabled)
        {
            printf(" interlaced: %d\n", interlaced);
            printf(" VSync: %d lines\n", nLinesInVSync);
            printf(" Field: %d lines, %d cycles\n", nLinesInField, nCyclesInField_256 << 8);
            printf(" block size: %d\n", blockSize);
        }
    }

    void ADV7181::SSPDState::dump() const
    {
        printf("SSPD: %s\n", enabled ? "enabled" : "disabled");
        if (enabled)
        {
            printf(" HS: %sactive, %s\n", isHSActive ? "" : "in", isHSNegative ? "negative" : "positive");
            printf(" VS: %sactive, %s\n", isVSActive ? "" : "in", isVSNegative ? "negative" : "positive");
        }
    }

    bool ADV7181::waitForCounterStable(int timeOutTimeInMS)
    {
        enum class State
        {
            IDLE,
            WAIT_DISABLE,
            WAIT_ENABLE,
            WAIT_STABLE1,
            WAIT_STABLE2,
            WAIT_STABLE3,
        };

        STDIState prevSTDI{};
        State state{};

        while (true)
        {
            auto isEnableSTDI = [&]
            {
                return getSTDIState().enabled;
            };

            auto next = [&]
            {
                state = static_cast<State>(static_cast<int>(state) + 1);
            };

            int wait = 1;
            auto computeWait = [&]
            {
                // 28.63MHz/256 のカウンタでの周期の2.5倍 (Frame + margin 0.5)
                wait = getSTDIState().nCyclesInField_256 * 1000 * 5 / (28636363 / 256 * 2);
                // wait = getSTDIState().nCyclesInField_256 * 1000 * 10 / (28636363 / 256 * 2);
                //  printf("wait = %d\n", wait);
            };

            switch (state)
            {
            case State::IDLE:
                stopSTDILineCountMode();
                next();
                break;

            case State::WAIT_DISABLE:
                if (!isEnableSTDI())
                {
                    startSTDILineCountMode();
                    wait = 80;
                    next();
                }
                else if (timeOutTimeInMS < 0)
                {
                    return false;
                }
                break;

            case State::WAIT_ENABLE:
                if (isEnableSTDI())
                {
                    prevSTDI = getSTDIState();
                    computeWait();
                    next();
                }
                else if (timeOutTimeInMS < 0)
                {
                    return false;
                }
                break;

            case State::WAIT_STABLE1:
            case State::WAIT_STABLE2:
            case State::WAIT_STABLE3:
                if (!isEnableSTDI() || !test(prevSTDI, getSTDIState(), 10))
                {
                    return false;
                }

                if (state == State::WAIT_STABLE3)
                {
                    return true;
                }
                computeWait();
                next();
                break;
            };

#if 0
            if (isEnableSTDI())
            {
                printf("%d:\n", (int)state);
                getSTDIState().dump();
            }
#endif

            sleep_ms(wait);
            timeOutTimeInMS -= wait;

            updateStatus();
        }
        return false;
    }

    bool
    test(const ADV7181::STDIState &a,
         const ADV7181::STDIState &b,
         int cycleMarginPerLine)
    {
        auto check = [](int va, int vb, int m)
        {
            auto d = va - vb;
            return d > -m && d < m;
        };

        return (a.enabled == b.enabled &&
                a.interlaced == b.interlaced &&
                a.nLinesInVSync == b.nLinesInVSync &&
                a.nLinesInField == b.nLinesInField &&
                check(a.nCyclesInField_256, b.nCyclesInField_256, a.nLinesInField * cycleMarginPerLine / 256) &&
                check(a.blockSize, b.blockSize, cycleMarginPerLine * 8));
    }
}
