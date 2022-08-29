/*
 * author : Shuichi TAKANO
 * since  : Sun Aug 28 2022 06:34:11
 */

#include "adv7181.h"
#include <array>
#include <cassert>
#include <cstdio>

namespace
{
    // ##CP RGB SOG 525i & 625i##
    // :525I RGB SOG In 8Bit 422 EAV/SAV out Encoder:
    static constexpr std::array<uint8_t, 2> i2cDataRGB_CP_[] = {
        {0x05, 0x00}, // Prim_Mode =000b for SD-M
        {0x06, 0x0A}, // VID_STD=1010b for SD 4x1 525i
        {0x1D, 0x47}, // Enable 28MHz Crystal
        {0x3A, 0x11}, // Set Latch Clock 01b, Power Down ADC3
        {0x3B, 0x81}, // Enable internal Bias
        {0x3C, 0x52}, // PLL_QPUMP to 010b
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
        {0x85, 0x19}, // Turn off SSPD and force SOY. For Eval Board.
        {0x86, 0x0B}, // Enable stdi_line_count_mode
        {0x8F, 0x77}, // FR_LL to 1820 & Enable 28.63MHz LLC
        {0x90, 0x1C}, // FR_LL to 1820
        {0xBF, 0x06}, // Blue Screen Free Run Colour
        {0xC0, 0x40}, // default color
        {0xC1, 0xF0}, // default color
        {0xC2, 0x80}, // Default color
        {0xC5, 0x01}, // CP_CLAMP_AVG_FACTOR[1-0] = 00b
        {0xC9, 0x0C}, // Enable DDR Mode
        {0xF3, 0x07}, // Enable Anti Alias Filter on ADC 0,1,2
        {0x0E, 0x80}, // ADI Recommended Setting
        {0x52, 0x46}, // ADI Recommended Setting
        {0x54, 0x80}, // ADI Recommended Setting
        {0xF6, 0x3B}, // ADI Recommended Setting
        {0x0E, 0x00}, // ADI Recommended Setting
        {0xC3, 0x56}, // ADC1 to Ain8 (Pr), ADC0 to Ain10 (Y),
        {0xC4, 0xF4}, // ADC2 to Ain6 (Pb) and enables manual override of mux, SOG
        // {0xC3, 0x46}, // ADC1 to Ain6 (Pr), ADC0 to Ain10 (Y),
        // {0xC4, 0xF5}, // ADC2 to Ain8 (Pb) and enables manual override of mux, SOG
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

    static constexpr int I2CADDR_CTRL_W = 0x42;
    static constexpr int I2CADDR_VBI_R = 0x23;
}

namespace device
{
    void
    ADV7181::reset() const
    {
        assert(i2c_);
        static constexpr std::array<uint8_t, 2> data = {0x0f, 0x80};
        i2c_write_blocking(i2c_, I2CADDR_CTRL_W >> 1, data.data(), data.size(), false);

        sleep_ms(5);

        // どうもリセット後の最初のコマンドは失敗するようなので適当な書き込みしておく
        uint8_t reg_st = 0x10;
        i2c_write_timeout_us(i2c_, I2CADDR_CTRL_W >> 1, &reg_st, 1, false, 1000);
    }

    void
    ADV7181::selectInput(Input input)
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
            case Input::COMPONENT:
                return i2cDataComponentCP_;

            case Input::RGB21:
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
                sendCount += i2c_write_blocking(i2c_, I2CADDR_CTRL_W >> 1, data.data(), data.size(), false);
            }

            printf("done. (%d bytes)\n", sendCount);
        }
    }

}
