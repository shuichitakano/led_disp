// #include <stdio.h>
#include <stdint.h>

#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/vreg.h>
#include <hardware/divider.h>
#include <hardware/i2c.h>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <vector>
#include <tuple>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>

#include <led.pio.h>
#include <bt656.pio.h>

#include "image_proc.h"
#include "bmp.h"
#include "framebuffer.h"
#include "textplane.h"
#include "menu.h"
#include "util.h"
#include "video_stream_buffer.h"
#include "video_stream.h"
#include "video_capture.h"
#include "adv7181.h"
#include "pca9554.h"
#include "tpa2016.h"
#include "nv_settings.h"
#include "app.h"
#include "video_serializer.h"

#include "ili9341.h"
#include "parameter_communicator.h"
#include "remote_video_capture.h"

namespace
{
    // #include "test.h"
#include "test_320x240.h"
}

#define LED_PANEL_DRIVER_TYPE_SHIFT_ONLY 0
#define LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH 1

// #define LED_PANEL_DRIVER_TYPE LED_PANEL_DRIVER_TYPE_SHIFT_ONLY // V1
#define LED_PANEL_DRIVER_TYPE LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH // V2
#define BOARD_VER 3

#if BOARDTYPE_SEPARATE_LED
static constexpr bool LED_PANEL_DRIVER_DEBUG = 0; // パネル駆動だけのデバッグモード
static constexpr bool ENABLE_LED_PANEL = true;
static constexpr bool ENABLE_VIDEO_BOARD = false;

static constexpr uint32_t PIN_VIDEO_SYNC = 1;
static constexpr uint32_t PIN_VIDEO_IN0 = 2;
static constexpr uint32_t PIN_VIDEO_IN0_SYNC = 5;
static constexpr uint32_t PIN_VIDEO_IN1 = 6;
static constexpr uint32_t PIN_VIDEO_IN1_SYNC = 9;

// dummy
static constexpr uint32_t PIN_VD0 = 6;
#define USE_VIDEO_DESERIALIZER 1

#elif BOARDTYPE_SEPARATE_VIDEO
static constexpr bool LED_PANEL_DRIVER_DEBUG = 0; // パネル駆動だけのデバッグモード
static constexpr bool ENABLE_VIDEO_BOARD = true;
static constexpr bool ENABLE_LED_PANEL = false;

// #define USE_LCD_PREVIEW 1
#define USE_DATA_SERIALIZER 1

// BT656
static constexpr uint32_t PIN_VD0 = 6;
static constexpr uint32_t PIN_VD1 = 7;
static constexpr uint32_t PIN_VD2 = 8;
static constexpr uint32_t PIN_VD3 = 9;
static constexpr uint32_t PIN_VD4 = 10;
static constexpr uint32_t PIN_VD5 = 11;
static constexpr uint32_t PIN_VD6 = 12;
static constexpr uint32_t PIN_VD7 = 13;
static constexpr uint32_t PIN_VCLK = 1;

static constexpr uint32_t PIN_VIDEO_BEGIN = 2;

static constexpr uint32_t PIN_HSYNC = 14;
static constexpr uint32_t PIN_VSYNC = 15;

static constexpr uint32_t PIN_OUT_DATA0 = 17;
static constexpr uint32_t PIN_OUT_DATA1 = 18;
static constexpr uint32_t PIN_OUT_CLK = 19;
static constexpr uint32_t PIN_OUT_SYNC = 20;

static constexpr uint32_t PIN_SLAVE_SELECT = 28;

#else
// OLD, ~ MFT2023
static constexpr bool LED_PANEL_DRIVER_DEBUG = 0; // パネル駆動だけのデバッグモード
static constexpr bool ENABLE_VIDEO_BOARD = !LED_PANEL_DRIVER_DEBUG;
static constexpr bool ENABLE_LED_PANEL = true;

#if BOARD_VER == 3
// BT656
static constexpr uint32_t PIN_VD0 = 1;
static constexpr uint32_t PIN_VD1 = 2;
static constexpr uint32_t PIN_VD2 = 3;
static constexpr uint32_t PIN_VD3 = 4;
static constexpr uint32_t PIN_VD4 = 5;
static constexpr uint32_t PIN_VD5 = 6;
static constexpr uint32_t PIN_VD6 = 7;
static constexpr uint32_t PIN_VD7 = 8;
static constexpr uint32_t PIN_VCLK = 9;
#else
// BT656
static constexpr uint32_t PIN_VD0 = 14;
static constexpr uint32_t PIN_VD1 = 15;
static constexpr uint32_t PIN_VD2 = 16;
static constexpr uint32_t PIN_VD3 = 17;
static constexpr uint32_t PIN_VD4 = 18;
static constexpr uint32_t PIN_VD5 = 19;
static constexpr uint32_t PIN_VD6 = 20;
static constexpr uint32_t PIN_VD7 = 21;
static constexpr uint32_t PIN_VCLK = 22;
#endif

#endif

#if BOARD_VER == 3

// LED data
static constexpr uint32_t PIN_R1 = 10;
static constexpr uint32_t PIN_R2 = 11;
static constexpr uint32_t PIN_G1 = 12;
static constexpr uint32_t PIN_G2 = 13;
static constexpr uint32_t PIN_B1 = 14;
static constexpr uint32_t PIN_B2 = 15;
static constexpr uint32_t PIN_RGB_TOP = PIN_R1;

// row select
static constexpr uint32_t PIN_A = 20;
static constexpr uint32_t PIN_B = 21;
static constexpr uint32_t PIN_C = 22;

// control
static constexpr uint32_t PIN_LATCH = 16;
static constexpr uint32_t PIN_FFCLK = 17;
static constexpr uint32_t PIN_CLK = 18;
static constexpr uint32_t PIN_LAT = 19;
static constexpr uint32_t PIN_OE = 28;

#else

// LED data
static constexpr uint32_t PIN_R1 = 8;
static constexpr uint32_t PIN_R2 = 9;
static constexpr uint32_t PIN_G1 = 10;
static constexpr uint32_t PIN_G2 = 11;
static constexpr uint32_t PIN_B1 = 12;
static constexpr uint32_t PIN_B2 = 13;
static constexpr uint32_t PIN_RGB_TOP = PIN_R1;

// row select
static constexpr uint32_t PIN_A = 5;
static constexpr uint32_t PIN_B = 6;
static constexpr uint32_t PIN_C = 7;

// control
#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
static constexpr uint32_t PIN_LATCH = 1;
static constexpr uint32_t PIN_FFCLK = 2;
static constexpr uint32_t PIN_CLK = 3;
#else
static constexpr uint32_t PIN_CLK = 2;
static constexpr uint32_t PIN_FFCLK = 3;
#endif
static constexpr uint32_t PIN_LAT = 4;
static constexpr uint32_t PIN_OE = 28;

#endif

// I2C
static constexpr uint32_t PIN_SDA = 26;
static constexpr uint32_t PIN_SCL = 27;
auto *i2cIF = i2c1;

auto *pioDataCmd_ = pio0;
auto *pioPWM_ = pio1;
static constexpr int SM_DATA1 = 0;
static constexpr int SM_DATA2 = 1;
static constexpr int SM_DATA3 = 2;
static constexpr int SM_COMMAND = 3;
static constexpr int SM_PWM = 0;
static constexpr int SM_VIDEO_IN = 1;
static constexpr int SM_VIDEO_IN_1 = 2;

static constexpr int PIOIRQ_DATAIDLE = 0;

uint ofsPWM;
uint ofsVideoIn;
// uint ofsData;
// uint ofsCommand;

class PIOProgram
{
    const pio_program *program_;
    int ofs_ = -1;

public:
    PIOProgram(const pio_program *prg)
        : program_(prg)
    {
    }

    int load(pio_hw_t *pio)
    {
        if (ofs_ >= 0)
        {
            return ofs_;
        }
        ofs_ = pio_add_program(pio, program_);
        return ofs_;
    }

    void release(pio_hw_t *pio)
    {
        if (ofs_ < 0)
        {
            return;
        }
        pio_remove_program(pio, program_, ofs_);
        ofs_ = -1;
    }

    int getOffset() const { return ofs_; }
};

#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
PIOProgram pioPrgCommand{&led_command_with_latch_program};
PIOProgram pioPrgData{&led_data_with_latch_program};
#else
PIOProgram pioPrgCommand{&led_command_program};
PIOProgram pioPrgData{&led_data_program};
#endif

constexpr int getPWMPIOClockDiv()
{
    return CPU_CLOCK_KHZ / 25000;
}

void setDataClockDiv()
{
    pio_sm_set_clkdiv_int_frac(pioPWM_, SM_PWM, getPWMPIOClockDiv(), 0);

    static constexpr int clkdiv250 = 1;
#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
    constexpr auto div = clkdiv250 * 3;
#else
    constexpr auto div = clkdiv250;
#endif
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA1, div, 0);
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA2, div, 0);
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA3, div, 0);
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_COMMAND, div, 0);
}

void setupPIOCommandProgram()
{
    pioPrgData.release(pioDataCmd_);
    auto ofs = pioPrgCommand.load(pioDataCmd_);
    assert(ofs >= 0);

#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
    initProgramLEDCommandWithLatch(pioDataCmd_, SM_COMMAND, ofs, PIN_RGB_TOP, PIN_LATCH, PIN_LAT);
#else
    initProgramLEDCommand(pioDataCmd_, SM_COMMAND, ofs, PIN_RGB_TOP, PIN_CLK, PIN_LAT);
#endif
    setDataClockDiv();
}

void setupPIODataProgram()
{
    pioPrgCommand.release(pioDataCmd_);
    auto ofs = pioPrgData.load(pioDataCmd_);
    assert(ofs >= 0);

#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
    initProgramLEDDataWithLatch1(pioDataCmd_, SM_DATA1, ofs, PIN_R1, PIN_LATCH, PIN_LAT);
    initProgramLEDDataWithLatch23(pioDataCmd_, SM_DATA2, ofs, PIN_G1);
    initProgramLEDDataWithLatch23(pioDataCmd_, SM_DATA3, ofs, PIN_B1);
#else
    initProgramLEDData1(pioDataCmd_, SM_DATA1, ofs, PIN_R1, PIN_CLK, PIN_LAT);
    initProgramLEDData23(pioDataCmd_, SM_DATA2, ofs, PIN_G1);
    initProgramLEDData23(pioDataCmd_, SM_DATA3, ofs, PIN_B1);
#endif
    setDataClockDiv();
}

void initPIO()
{
    if (ENABLE_LED_PANEL)
    {
        ofsPWM = pio_add_program(pioPWM_, &led_pwm_program);
        initProgramLEDPWM(pioPWM_, SM_PWM, ofsPWM, PIN_A, PIN_OE);
    }

    if (ENABLE_VIDEO_BOARD)
    {
#if BOARDTYPE_SEPARATE_VIDEO
        ofsVideoIn = pio_add_program(pioPWM_, &capture_bt656_ddr_hs_program);
        initProgramCaptureBT656DDR_HS(pioPWM_, SM_VIDEO_IN, ofsVideoIn, PIN_VIDEO_BEGIN, PIN_VCLK);
#else
        ofsVideoIn = pio_add_program(pioPWM_, &capture_bt656_program);
        initProgramCaptureBT656(pioPWM_, SM_VIDEO_IN, ofsVideoIn, PIN_VD0);
#endif
    }

    static constexpr int clkdiv25 = CPU_CLOCK_KHZ / 25000;
    pio_sm_set_clkdiv_int_frac(pioPWM_, SM_PWM, clkdiv25, 0);

#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_ONLY
    gpio_set_slew_rate(PIN_R1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_G1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_B1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_R2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_G2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_B2, GPIO_SLEW_RATE_FAST);

    gpio_set_slew_rate(PIN_A, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_B, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_C, GPIO_SLEW_RATE_FAST);

    gpio_set_slew_rate(PIN_CLK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_FFCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_OE, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_LAT, GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(PIN_A, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_B, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_C, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_LAT, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_CLK, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(PIN_OE, GPIO_DRIVE_STRENGTH_12MA);
#if 1
    gpio_set_drive_strength(PIN_R1, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_R2, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_G1, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_G2, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_B1, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_B2, GPIO_DRIVE_STRENGTH_8MA);
#endif
#endif
}

static constexpr int dmaChData1 = 0;
static constexpr int dmaChData2 = 1;
static constexpr int dmaChData3 = 2;
static constexpr int dmaChCommand = 3;
static constexpr int dmaChPWM = 4;
static constexpr int dmaChVideoIn = 5;

void initDMA()
{
    dma_claim_mask((1 << 7) - 1);

    auto initCH = [](int ch, auto *pio, int sm, int chainCh = -1, bool irqQuiet = false)
    {
        dma_channel_config channel_config = dma_channel_get_default_config(ch);
        channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true /* tx */));
        if (chainCh >= 0)
        {
            channel_config_set_chain_to(&channel_config, chainCh);
        }
        channel_config_set_irq_quiet(&channel_config, irqQuiet);
        dma_channel_configure(ch,
                              &channel_config,
                              &pio->txf[sm],
                              nullptr, 0, // あとで
                              false);
    };

    initCH(dmaChData1, pioDataCmd_, SM_DATA1);
    initCH(dmaChData2, pioDataCmd_, SM_DATA2);
    initCH(dmaChData3, pioDataCmd_, SM_DATA3);
    initCH(dmaChCommand, pioDataCmd_, SM_COMMAND);

    // auto initChain = [](int ch, int dataCh)
    // {
    //     auto cfg = dma_channel_get_default_config(ch);
    //     dma_channel_configure(ch,
    //                           &cfg,
    //                           &dma_channel_hw_addr(dataCh)->al3_read_addr_trig,
    //                           NULL, 1,
    //                           false);
    // };

    // initChain(dmaChChain1, dmaChData1);
    // initChain(dmaChChain2, dmaChData2);
    // initChain(dmaChChain3, dmaChData3);

    auto initIn = [](int ch, auto *pio, int sm)
    {
        dma_channel_config config = dma_channel_get_default_config(ch);
        channel_config_set_dreq(&config, pio_get_dreq(pio, sm, false /* tx */));
        channel_config_set_read_increment(&config, false);
        channel_config_set_write_increment(&config, true);
        dma_channel_configure(ch, &config, nullptr, &pio->rxf[sm], 0, false);
    };

    initIn(dmaChVideoIn, pioPWM_, SM_VIDEO_IN);
}

// LED PWM の1周期ぶんの出力を開始する
void __not_in_flash_func(startLEDPWM)(int clocksPerLine, int nLines)
{
    clocksPerLine -= 1;
    nLines -= 2;

    uint32_t data = (nLines << 16) | (clocksPerLine << 0);
    pio_sm_put_blocking(pioPWM_, SM_PWM, data);
}

void __not_in_flash_func(startLEDPWM)(int clocksPerLine, int nLines, int n)
{
    clocksPerLine -= 1;
    nLines -= 2;

    static uint32_t dmaPWMData = (nLines << 16) | (clocksPerLine << 0);

    auto cfg = dma_channel_get_default_config(dmaChPWM);
    channel_config_set_dreq(&cfg, pio_get_dreq(pioPWM_, SM_PWM, true /* tx */));
    channel_config_set_read_increment(&cfg, false);

    dma_channel_configure(dmaChPWM, &cfg, &pioPWM_->txf[SM_PWM],
                          &dmaPWMData, n, true);
}

constexpr int computeLEDPWMCycle(int clocksPerLine, int nLines)
{
    auto c = (2 * clocksPerLine + 5 + 13 + 15 * 2) * (nLines - 1) + 6 + 2 + 15 * 2 + 2 * clocksPerLine;
    return c * getPWMPIOClockDiv();
}

void __not_in_flash_func(finishLEDPWM)()
{
    dma_channel_wait_for_finish_blocking(dmaChPWM);
}

// Commandの出力を開始する
void __not_in_flash_func(sendCommand)(const uint32_t *cmdBits, size_t size)
{
    pio_sm_set_enabled(pioDataCmd_, SM_COMMAND, true);

    dma_channel_set_trans_count(dmaChCommand, size, false);
    dma_channel_set_read_addr(dmaChCommand, cmdBits, true);
}

// データのないコマンドを作成
void makeSimpleCommand(std::vector<uint32_t> &dst, int nL, int nH)
{
    int n = (nL + nH + 3) & ~3;
    nL = n - nH;

    dst.push_back((nL - 1) | ((nH - 1) << 16));
    dst.insert(dst.end(), n >> 2, 0);
}

// データ付きのコマンドを作成
void makeCommand(std::vector<uint32_t> &dst, int nBits, int nH, int r, int g, int b)
{
    nBits = (nBits + 3) & ~3;
    int nL = nBits - nH;
    int nPad = nBits - N_CASCADE * 16;

#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
    // latch タイミングに合わせてコマンドの byte を１つ前にずらす
    assert(nPad);
    --nPad;
#endif

    dst.push_back((nL - 1) | ((nH - 1) << 16));

    auto pos = dst.size();
    dst.insert(dst.end(), nBits >> 2, 0);

    auto p = reinterpret_cast<uint8_t *>(dst.data() + pos) + nPad;

    constexpr int rMask = (1 << (PIN_R1 - PIN_RGB_TOP)) | (1 << (PIN_R2 - PIN_RGB_TOP));
    constexpr int gMask = (1 << (PIN_G1 - PIN_RGB_TOP)) | (1 << (PIN_G2 - PIN_RGB_TOP));
    constexpr int bMask = (1 << (PIN_B1 - PIN_RGB_TOP)) | (1 << (PIN_B2 - PIN_RGB_TOP));

    for (int i = 0; i < N_CASCADE; ++i)
    {
        for (int mask = 0x8000; mask; mask >>= 1)
        {
            int rv = (r & mask) ? rMask : 0;
            int gv = (g & mask) ? gMask : 0;
            int bv = (b & mask) ? bMask : 0;
            *p++ = rv | gv | bv;
        }
    }
    assert((char *)p <= (char *)(dst.data() + dst.size()));
}

void __not_in_flash_func(resetPIOTxStalled)(PIO pio, uint sm)
{
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
}

bool __not_in_flash_func(isPIOTxStalled)(PIO pio, uint sm)
{
    return pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm));
}

bool __not_in_flash_func(isLEDPWMStalled)()
{
    // return pio_sm_is_exec_stalled(pioPWM_, SM_PWM);
    return isPIOTxStalled(pioPWM_, SM_PWM);
}

bool __not_in_flash_func(isCommandTransferStalled)()
{
    return isPIOTxStalled(pioDataCmd_, SM_COMMAND);
}

void __not_in_flash_func(finishCommandTransfer)()
{
    dma_channel_wait_for_finish_blocking(dmaChCommand);

    resetPIOTxStalled(pioDataCmd_, SM_COMMAND);
    while (!isCommandTransferStalled())
    {
        tight_loop_contents();
    }

    pio_sm_set_enabled(pioDataCmd_, SM_COMMAND, false);

    // LAT = 0
    pio_sm_exec(pioDataCmd_, SM_COMMAND, pio_encode_set(pio_pins, 0));
}

void __not_in_flash_func(setDataTransferParams)(int sm, int unitW, int h, int nCascades, int bpp)
{
    // isr: nCascades * bpp - 1 = 10 * 16 - 1
    uint32_t isr = nCascades * bpp - 1;
    // printf("isr %d\n", isr);

    pio_sm_set_enabled(pioDataCmd_, sm, false);

    assert(pio_sm_is_tx_fifo_empty(pioDataCmd_, SM_DATA1));
    // pio_sm_clear_fifos(pioDataCmd_, sm);

    pio_sm_put_blocking(pioDataCmd_, sm, isr);
    pio_sm_exec(pioDataCmd_, sm, pio_encode_pull(false, false));
    pio_sm_exec(pioDataCmd_, sm, pio_encode_out(pio_isr, 32));
    // pio_sm_exec(pioDataCmd_, sm, pio_encode_jmp(ofsData));

    pio_sm_clear_fifos(pioDataCmd_, sm);
}

void __not_in_flash_func(startDataTransfer)()
{
    setDataTransferParams(SM_DATA1, UNIT_WIDTH, N_SCAN_LINES, N_CASCADE, BPP);
    setDataTransferParams(SM_DATA2, UNIT_WIDTH, N_SCAN_LINES, N_CASCADE, BPP);
    setDataTransferParams(SM_DATA3, UNIT_WIDTH, N_SCAN_LINES, N_CASCADE, BPP);

    pio_enable_sm_mask_in_sync(pioDataCmd_, 7);
}

void __not_in_flash_func(waitDataTransferStalled)()
{
    resetPIOTxStalled(pioDataCmd_, SM_DATA1);
    while (!isPIOTxStalled(pioDataCmd_, SM_DATA1))
    {
        tight_loop_contents();
    }
}

void __not_in_flash_func(waitForDataFIFOEmpty)()
{
    // while (!pio_sm_is_tx_fifo_empty(pioDataCmd_, SM_DATA1))
    while (((pioDataCmd_->fstat >> PIO_FSTAT_TXEMPTY_LSB) & 7) != 7)
    {
        tight_loop_contents();
    }
}

void __not_in_flash_func(waitForDataFIFOFull)()
{
    // while (!pio_sm_is_tx_fifo_full(pioDataCmd_, SM_DATA1 + i))
    while (((pioDataCmd_->fstat >> PIO_FSTAT_TXFULL_LSB) & 7) != 7)
    {
        tight_loop_contents();
    }
}

void __not_in_flash_func(transferData)(const std::array<uint32_t *, 3> &data, size_t size)
{
    // FIFO empty　まち
    waitForDataFIFOEmpty();

#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
    ++size; // prologue
#endif

    // DMA 開始
    for (int i = 0; i < 3; ++i)
    {
        dma_channel_set_trans_count(dmaChData1 + i, size, false);
        dma_channel_set_read_addr(dmaChData1 + i, data[i], true);
    }

    // FIFO fullまち
    waitForDataFIFOFull();

    // IRQ0 が立つのをまつ
    // while (!pio_interrupt_get(pioDataCmd_, PIOIRQ_DATAIDLE))
    while (pioDataCmd_->irq != 7)
    {
        tight_loop_contents();
    }

    // IRQ0 をクリアして転送開始
    // pio_interrupt_clear(pioDataCmd_, PIOIRQ_DATAIDLE);
    hw_set_bits(&pioDataCmd_->irq, 7);
}

void __not_in_flash_func(finishDataTransfer)()
{
    dma_channel_wait_for_finish_blocking(dmaChData1);
    dma_channel_wait_for_finish_blocking(dmaChData2);
    dma_channel_wait_for_finish_blocking(dmaChData3);

    waitForDataFIFOEmpty();

    // 先頭に戻っているのを確認する
    auto ofs = pioPrgData.getOffset();
    while (1)
    {
        auto pc = pio_sm_get_pc(pioDataCmd_, SM_DATA1);
        if (pc == ofs || pc == ofs + 1)
        {
            break;
        }
    }

    hw_clear_bits(&pioDataCmd_->ctrl, 7u << PIO_CTRL_SM_ENABLE_LSB);
}

void __not_in_flash_func(waitVideoCapture)()
{
    resetPIOTxStalled(pioPWM_, SM_VIDEO_IN);
    while (!isPIOTxStalled(pioPWM_, SM_VIDEO_IN))
    {
        tight_loop_contents();
    }
}

void __not_in_flash_func(startVideoCapture)(uint32_t *dst,
                                            uint32_t startCode, uint32_t transferWords)
{
    assert(transferWords > 0);
    // waitVideoCapture();

    dma_channel_set_trans_count(dmaChVideoIn, transferWords, false);
    dma_channel_set_write_addr(dmaChVideoIn, dst, true);

#if BOARDTYPE_SEPARATE_VIDEO
    // pio_sm_put(pioPWM_, SM_VIDEO_IN, startCode);
    pio_sm_put(pioPWM_, SM_VIDEO_IN, transferWords - 1);
#else
    pio_sm_put(pioPWM_, SM_VIDEO_IN, startCode);
    pio_sm_put(pioPWM_, SM_VIDEO_IN, transferWords * 4 - 1);
#endif
}

bool __not_in_flash_func(getVSync)()
{
#if BOARDTYPE_SEPARATE_VIDEO
    return gpio_get(PIN_VSYNC);
#else
    return false;
#endif
}

////////////////////////
////////////////////////

struct LEDDriver;
namespace
{
    LEDDriver *LEDDriverInst_{};

    struct DeviceSet
    {
        bool enabled = false;
        device::ADV7181 adv7181;
        device::PCA9554 pca9554;

    public:
        void init(bool primary)
        {
            enabled = adv7181.init(i2cIF, primary);
            enabled &= pca9554.init(i2cIF, device::PCA9554::Type::C, primary ? 4 : 0);
        }
    };

    DeviceSet deviceSet_[2];
    device::TPA2016 tpa2016_;

    template <class F>
    void callADV7181(const F &f)
    {
        for (auto &ds : deviceSet_)
        {
            if (ds.enabled)
            {
                f(ds.adv7181);
            }
        }
    }
    template <class F>
    void callPCA9554(const F &f)
    {
        for (auto &ds : deviceSet_)
        {
            if (ds.enabled)
            {
                f(ds.pca9554);
            }
        }
    }

    struct SpeakerSettings
    {
        // 保存したいよな
        int gainDB = -28;
        int limiterLevelDBx2 = static_cast<int>(6.5 * 2);
        int enableLimiter = true;
        int noiseGateThreshold = 1; // 4mv
        int enableNoiseGate = true;
        int maxGainDB = 30;
        int compressionRatioLog2 = 2; // 4:1

        void apply(device::TPA2016 &d) const
        {
            d.setControl(true, true, false, enableNoiseGate);
            d.setGain(gainDB);
            d.setAGCControl1(enableLimiter, limiterLevelDBx2, noiseGateThreshold);
            d.setAGCControl2(maxGainDB, compressionRatioLog2);
        }
    };

    SpeakerSettings speakerSettings_;
}

struct LEDDriver
{
    std::vector<uint32_t> command0_;
    std::vector<uint32_t> command1_;

    const graphics::BMP *bmp_ = {};

    static constexpr int MODULE_WIDTH = 160;
    static constexpr int MODULE_HEIGHT = 120;
    static constexpr int X_MODULES = 2;
    static constexpr int Y_MODULES = 2;
    static constexpr int TOTAL_WIDTH = MODULE_WIDTH * X_MODULES;
    static constexpr int TOTAL_HEIGHT = MODULE_HEIGHT * Y_MODULES;
    static constexpr int UNIT_DATA_BUFFER_SIZE = TOTAL_WIDTH * Y_MODULES;
    static constexpr int N_DATA_BUFFERS = 2;
    static constexpr int N_RGB_CH = 3;

    // 先頭 1 word がラインサイズ。1 word が guard
    uint32_t dataBuffer_[N_DATA_BUFFERS][N_RGB_CH][UNIT_DATA_BUFFER_SIZE + 2]; // 22.5KB

    graphics::FrameBuffer frameBuffer_;
    graphics::TextPlane textPlane_;
    video::VideoCapture capture_;
    ui::Menu menu_;

    int currentInput_ = 0;

    enum class InfoMode
    {
        NONE,
        FPS_ONLY,
        ALL,
    };
    int infoMode_ = static_cast<int>(InfoMode::FPS_ONLY);

    int currentRefreshPerField_ = 0; // 1フィールドあたりのPWM更新回数

    void init()
    {
        LEDDriverInst_ = this;

        constexpr int margin_lines = 20;
        frameBuffer_.initialize(TOTAL_WIDTH, TOTAL_HEIGHT, margin_lines);

        bmp_ = (const graphics::BMP *)MagickImage;

        makeSimpleCommand(command0_, 1, 14); // pre-active
        makeSimpleCommand(command0_, 8, 12); // enable all output
        makeSimpleCommand(command0_, 8, 3);  // v sync
        makeSimpleCommand(command0_, 8, 14);

        makeCommand(command1_, 168, 4, 0x3b70, 0x3b70, 0x3b70); // config 1

        makeSimpleCommand(command1_, 8, 14);
        makeCommand(command1_, 168, 6, 0x7f35, 0x6735, 0x5f35); // config 2

        makeSimpleCommand(command1_, 8, 14);
        makeCommand(command1_, 168, 8, 0x40f7, 0x40f7, 0x40f7); // config 3

        makeSimpleCommand(command1_, 8, 14);
        makeCommand(command1_, 168, 10, 0x0000, 0x0000, 0x0000); // config 4

        makeSimpleCommand(command1_, 8, 14);
        makeCommand(command1_, 168, 2, 0x0000, 0x0000, 0x0000); // debug

        printf("dataBuffer (%p, %p, %p) (%p, %p, %p)\n",
               dataBuffer_[0][0],
               dataBuffer_[0][1],
               dataBuffer_[0][2],
               dataBuffer_[1][0],
               dataBuffer_[1][1],
               dataBuffer_[1][2]);
    }

    void __not_in_flash_func(scan)()
    {
        startLEDPWM(PWM_PULSE_PER_LINE, N_SCAN_LINES);
    }

    void __not_in_flash_func(scanIfStalled)()
    {
        if (isLEDPWMStalled())
        {
            scan();
        }
    }

    void __not_in_flash_func(waitForScanStall)()
    {
        resetPIOTxStalled(pioPWM_, SM_PWM);
        while (!isLEDPWMStalled())
        {
            tight_loop_contents();
        }
    }

    volatile bool dataTransferComplete_ = true;

    void __not_in_flash_func(initFrameState)()
    {
        dataTransferComplete_ = true;
    }

    bool __not_in_flash_func(isDataTransferCompleted)()
    {
        return dataTransferComplete_;
    }

    void __not_in_flash_func(waitDataTransfer)()
    {
        while (!isDataTransferCompleted())
        {
            tight_loop_contents();
        }
    }

    int textLayerCompositeLine_ = 0;

    void __not_in_flash_func(compositeTextLayer)()
    {
        int y0 = textLayerCompositeLine_;
        if (y0 >= N_SCAN_LINES)
        {
            return;
        }
        int y1 = y0 + N_SCAN_LINES;
        int y2 = y1 + N_SCAN_LINES;
        int y3 = y2 + N_SCAN_LINES;

        auto [p0, p0e] = frameBuffer_.peekCurrentPlaneLineUnsafe(y0);
        auto [p1, p1e] = frameBuffer_.peekCurrentPlaneLineUnsafe(y1);
        auto [p2, p2e] = frameBuffer_.peekCurrentPlaneLineUnsafe(y2);
        auto [p3, p3e] = frameBuffer_.peekCurrentPlaneLineUnsafe(y3);
        if (!(p0e & p1e & p2e & p3e))
        {
            return;
        }
        if (!p0 || !p1 || !p2 || !p3)
        {
            ++textLayerCompositeLine_;
            return;
        }

        textPlane_.setupComposite();
        textPlane_.composite(p0, y0);
        textPlane_.composite(p1, y1);
        textPlane_.composite(p2, y2);
        textPlane_.composite(p3, y3);
        ++textLayerCompositeLine_;
    }

    void __not_in_flash_func(waitForUpdateReady)(int y)
    {
        frameBuffer_.waitForCurrentPlaneLineReady(y + N_SCAN_LINES * 3);
    }

    struct
    {
        uint32_t t0;
        uint32_t t1;
        uint32_t t2;
        uint32_t t3;
        uint32_t t4;
    } updateDataBuffertTiming_;

    void __not_in_flash_func(updateDataBuffer)(int dataBufferID, int y, bool textComp)
    {
        updateDataBuffertTiming_.t0 = util::getSysTickCounter24();

        auto *dstR = dataBuffer_[dataBufferID][0];
        auto *dstG = dataBuffer_[dataBufferID][1];
        auto *dstB = dataBuffer_[dataBufferID][2];

        int y1 = y + N_SCAN_LINES;
        int y2 = y1 + N_SCAN_LINES;
        int y3 = y2 + N_SCAN_LINES;

        int lineID0 = frameBuffer_.moveCurrentPlaneLine(y);
        int lineID1 = frameBuffer_.moveCurrentPlaneLine(y1);
        int lineID2 = frameBuffer_.moveCurrentPlaneLine(y2);
        int lineID3 = frameBuffer_.moveCurrentPlaneLine(y3);

        updateDataBuffertTiming_.t1 = util::getSysTickCounter24();

        auto *line0 = frameBuffer_.getLineBuffer(lineID0);
        auto *line1 = frameBuffer_.getLineBuffer(lineID1);
        auto *line2 = frameBuffer_.getLineBuffer(lineID2);
        auto *line3 = frameBuffer_.getLineBuffer(lineID3);

        if (textComp && 0)
        {
            textPlane_.setupComposite();
            textPlane_.composite(line0, y);
            textPlane_.composite(line1, y1);
            textPlane_.composite(line2, y2);
            textPlane_.composite(line3, y3);
            textLayerCompositeLine_ = y + 1;
        }
        updateDataBuffertTiming_.t2 = util::getSysTickCounter24();

        // 先頭に ラインの繰り返し数
#if LED_PANEL_DRIVER_TYPE == LED_PANEL_DRIVER_TYPE_SHIFT_WITH_LATCH
        int loop = UNIT_WIDTH;
#else
        int loop = UNIT_WIDTH - 1;
#endif
        dstR[0] = loop;
        dstG[0] = loop;
        dstB[0] = loop;
        graphics::convert4(dstR + 1, dstG + 1, dstB + 1, line0, line1, line2, line3);

        updateDataBuffertTiming_.t3 = util::getSysTickCounter24();

        frameBuffer_.freeLine({lineID0, lineID1, lineID2, lineID3});

        updateDataBuffertTiming_.t4 = util::getSysTickCounter24();
    }

    ///
    int nextTransferLine_ = 0;

    constexpr int getTransferLineDBID() const
    {
        return nextTransferLine_ & 1;
    }

    void __not_in_flash_func(startTransferFrame)()
    {
        dataTransferComplete_ = false;
        nextTransferLine_ = 0;

#if 0
        waitForUpdateReady(0);
        updateDataBuffer(getTransferLineDBID(), nextTransferLine_, true);
        transferAndUpdateNextLine();
#else
        updateDataBuffer(0, 0, true);
        updateDataBuffer(1, 1, true);

        int dbid = getTransferLineDBID();
        transferData({dataBuffer_[dbid][0],
                      dataBuffer_[dbid][1],
                      dataBuffer_[dbid][2]},
                     UNIT_DATA_BUFFER_SIZE + 1); // DMA開始

        ++nextTransferLine_;
#endif
    }

    void __not_in_flash_func(transferAndUpdateNextLine)()
    {
        if (nextTransferLine_ >= N_SCAN_LINES)
        {
            dataTransferComplete_ = true;
            return;
        }

        int dbid = getTransferLineDBID();
        transferData({dataBuffer_[dbid][0],
                      dataBuffer_[dbid][1],
                      dataBuffer_[dbid][2]},
                     UNIT_DATA_BUFFER_SIZE + 1); // DMA開始

        if (++nextTransferLine_ < N_SCAN_LINES)
        {
            // setup next line
            updateDataBuffer(getTransferLineDBID(), nextTransferLine_, false);
        }
    }

    void __not_in_flash_func(dmaIRQHandler)()
    {
        if (dma_hw->ints0 & (1 << dmaChData1))
        {
#if 1
            dma_hw->ints0 = 1 << dmaChData1;
            dataTransferComplete_ = true;
#else
            if (auto nextUpdateLine = nextTransferLine_ + 1; nextUpdateLine < N_SCAN_LINES)
            {
                waitForUpdateReady(nextUpdateLine);
            }

            dma_hw->ints0 = 1 << dmaChData1;
            // dataTransferComplete_ = true;

            transferAndUpdateNextLine();
#endif
        }
    }

    static void __isr __not_in_flash_func(dmaIRQHandlerEntry)()
    {
        LEDDriverInst_->dmaIRQHandler();
    }

    uint32_t latestSendCycles_ = 0;
    bool requestSendLoopDump_ = false;

    void __not_in_flash_func(loop)()
    {
        irq_set_exclusive_handler(DMA_IRQ_0, dmaIRQHandlerEntry);
        dma_channel_set_irq0_enabled(dmaChData1, true);
        irq_set_enabled(DMA_IRQ_0, true);

        int led = 0;

        while (1)
        {
            auto t0 = util::getSysTickCounter24();
            setupPIOCommandProgram();

            // まず VSync 等のコマンドを送る
            sendCommand(command0_.data(), command0_.size());
            finishCommandTransfer();

            // ここから scan はじめていいはず
            // startLEDPWM(PWM_PULSE_PER_LINE, N_SCAN_LINES, 23);
            // todo: 回数はフレームレートから計算する

            // 残りのコマンドを送る
            sendCommand(command1_.data(), command1_.size());
            finishCommandTransfer();

            setupPIODataProgram();
            // コード入れ替え中のクロックが止まる間にPWMクロックを出してはいけない
            // (仮に)ここからPWM始める
            // startLEDPWM(PWM_PULSE_PER_LINE, N_SCAN_LINES, 23);

            // データ送出の SM を開始 (DCLK 送出を開始)
            initFrameState();
            startDataTransfer();

            auto t1 = util::getSysTickCounter24();
            auto dt10 = (t0 - t1) & 0xffffff;
            // int frameInterval = LED_PANEL_DRIVER_DEBUG ? frameTick_ : capture_.getFieldIntervalCycles();
            int frameInterval = CPU_CLOCK_KHZ * 1000 / 60; // とりあえず固定！！
            int leftCycle = frameInterval - dt10;
            constexpr auto cyclePWM = computeLEDPWMCycle(PWM_PULSE_PER_LINE, N_SCAN_LINES);
            auto nPWM = std::max(1, (leftCycle - (cyclePWM >> 6)) / cyclePWM); // 1/64周期マージン
            currentRefreshPerField_ = nPWM;

            // clk出るまでPWM始めない
            startLEDPWM(PWM_PULSE_PER_LINE, N_SCAN_LINES, nPWM);

#if 1
            struct Log
            {
                uint32_t update;
                uint32_t updateWait;
                uint32_t updateText;
                uint32_t updateCvt;
                uint32_t updateFree;
                uint32_t wait;
                uint32_t transfer;
            };
            static Log log[N_SCAN_LINES];
            auto *plog = log;

            int dbid = 0;
            for (int i = 0; i < N_SCAN_LINES; ++i)
            {
                auto lt0 = util::getSysTickCounter24();
                updateDataBuffer(dbid, i, true);
                auto lt1 = util::getSysTickCounter24();
                waitDataTransfer(); // DMAまち
                dataTransferComplete_ = false;
                auto lt2 = util::getSysTickCounter24();
                transferData({dataBuffer_[dbid][0],
                              dataBuffer_[dbid][1],
                              dataBuffer_[dbid][2]},
                             UNIT_DATA_BUFFER_SIZE + 1); // DMA開始
                dbid ^= 1;

                auto lt3 = util::getSysTickCounter24();
                plog->update = (lt0 - lt1) & 0xffffff;
                plog->updateWait = (updateDataBuffertTiming_.t0 - updateDataBuffertTiming_.t1) & 0xffffff;
                plog->updateText = (updateDataBuffertTiming_.t1 - updateDataBuffertTiming_.t2) & 0xffffff;
                plog->updateCvt = (updateDataBuffertTiming_.t2 - updateDataBuffertTiming_.t3) & 0xffffff;
                plog->updateFree = (updateDataBuffertTiming_.t3 - updateDataBuffertTiming_.t4) & 0xffffff;
                plog->wait = (lt1 - lt2) & 0xffffff;
                plog->transfer = (lt2 - lt3) & 0xffffff;
                ++plog;
            }

            // DBGPRINT("!\n");

            finishLEDPWM();
            waitForScanStall();

            waitDataTransfer();
            finishDataTransfer();
            assert(dataTransferComplete_);
#else
            startTransferFrame();

            while (!isDataTransferCompleted())
            {
                compositeTextLayer();
            }

            finishLEDPWM();
            waitForScanStall();

            //            waitDataTransfer();
            finishDataTransfer();
#endif

            // sleep_us(1);
            frameBuffer_.flipReadPlane();

            gpio_put(PICO_DEFAULT_LED_PIN, led);
            led ^= 1;

            NVSettings::instance().tick();

            auto t2 = util::getSysTickCounter24();
            latestSendCycles_ = (t0 - t2) & 0xffffff;

            if (requestSendLoopDump_)
            {
                requestSendLoopDump_ = false;

                int i = 0;
                for (auto &l : log)
                {
                    printf("%d: u%d(%d+%d+%d+%d) w%d t%d\n", i, l.update, l.updateWait, l.updateText, l.updateCvt, l.updateFree, l.wait, l.transfer);
                    ++i;
                }
                printf("t0: %d\n", (t0 - t1) & 0xffffff);
                printf("t1: %d\n", (t1 - t2) & 0xffffff);
                printf("total: %d\n", latestSendCycles_);
            }
        }
    }

    void __not_in_flash_func(drawStatusLine)(int fps16)
    {
        // textPlane_.fillBlack(4, 8, 12, 6);
        textPlane_.setColor(2);
        textPlane_.setShadowColor(0);
        textPlane_.enableShadow(true);
        textPlane_.enableTransBlack(false);

        if (!menu_.isOpened())
        {
            if (infoMode_ >= static_cast<int>(InfoMode::FPS_ONLY))
            {
                textPlane_.printf(40 - 8, 25, "%d.%dFPS",
                                  fps16 >> 4, ((fps16 & 15) * 10) >> 4);
            }
#ifndef USE_VIDEO_DESERIALIZER
            if (infoMode_ >= static_cast<int>(InfoMode::ALL))
            {
                textPlane_.printf(40 - 14, 25, "%2dRPF", currentRefreshPerField_);
                switch (1)
                {
                case 0:
                    textPlane_.printf(40 - 13, 24, "Send %7d", latestSendCycles_);
                    break;

                case 1:
                    // textPlane_.printf(0, 24, "ST%d SS%d FR%d %d LK%d %d",
                    //                   adv7181_.getSTDIState().enabled, adv7181_.getSSPDState().enabled,
                    //                   adv7181_.isFreeRunCP(), adv7181_.isFreeRunSDP(),
                    //                   adv7181_.isPLLLockedCP(), adv7181_.isPLLLockedSDP());
                    break;
                }
            }
#endif
        }
    }

    void __not_in_flash_func(captureDMAIRQHandler)(uint32_t time)
    {
        if (dma_hw->ints1 & (1 << dmaChVideoIn))
        {
            dma_hw->ints1 = 1 << dmaChVideoIn;

            hw_divider_state_t divState;
            hw_divider_save_state(&divState);

            if (!capture_.irq(time))
            {
                enableVideoCaptureIRQ(false);
            }

            hw_divider_restore_state(&divState);
        }
    }

    static void __isr __not_in_flash_func(captureDMAIRQHandlerEntry)()
    {
        auto t = util::getSysTickCounter24();
        LEDDriverInst_->captureDMAIRQHandler(t);
    }

    void __not_in_flash_func(captureVSyncIRQHandler)(bool rise, uint32_t time)
    {
        capture_.vsyncIRQ(rise, time);
    }

    static void __isr __not_in_flash_func(captureGPIOCallbackEntry)(uint gpio,
                                                                    uint32_t event)
    {
#if USE_VSYNC_PIN
        //        auto irq = save_and_disable_interrupts();

        auto t = util::getSysTickCounter24();
        bool rise = event & GPIO_IRQ_EDGE_RISE;
        LEDDriverInst_->captureVSyncIRQHandler(rise, t);

        gpio_set_irq_enabled(PIN_VSYNC,
                             GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                             true);

//        restore_interrupts(irq);
#endif
    }

    void setVideoCaptureIRQ()
    {
        irq_set_exclusive_handler(DMA_IRQ_1, captureDMAIRQHandlerEntry);
        irq_set_enabled(DMA_IRQ_1, true);

        enableVideoCaptureIRQ(false);

#if USE_VSYNC_PIN
        // VSync
        gpio_set_irq_enabled_with_callback(PIN_VSYNC,
                                           GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                                           true /* enable */,
                                           captureGPIOCallbackEntry);
        irq_set_priority(IO_IRQ_BANK0, 0);
#endif
    }

    void initMenu()
    {
        static bool inputSelClose = false;
#if 0
        static constexpr const char *inputTexts[] = {"VIDEO", "S VIDEO", "COMPONENT", "RGB"};
        menu_.appendItem(&currentInput_, "INPUT", std::begin(inputTexts), std::end(inputTexts),
                         "Select Input",
                         {},
                         [&]
                         {
                             auto v = static_cast<device::SignalInput>(currentInput_ + 1);
                             callADV7181([&](auto &adv)
                                         { adv.selectInput(v); });
                             callPCA9554([&](auto &pca)
                                         { selectAudioInput(pca, v); });
                             resetSignalDetection_ = true;
                             inputSelClose = true;
                             menu_.close();
                         });
#endif

        capture_.initMenu(menu_);

        if (tpa2016_.isExist())
        {
            static constexpr std::array<const char *, 2> enableTexts = {"DISABLE", "ENABLE"};
            auto apply = [&]
            { speakerSettings_.apply(tpa2016_); };
            menu_.appendItem(&speakerSettings_.gainDB, {-28, 30}, "Spk GAIN", {}, apply);
#ifndef NDEBUG
            menu_.appendItem(&speakerSettings_.maxGainDB, {18, 30}, "Spk MGAIN", {}, apply);
            menu_.appendItem(&speakerSettings_.compressionRatioLog2, {0, 3}, "Spk CmpR", {}, apply);
            menu_.appendItem(&speakerSettings_.enableLimiter, "Spk Limit", enableTexts.begin(), enableTexts.end(), {}, apply);
            menu_.appendItem(&speakerSettings_.limiterLevelDBx2, {-13, 18}, "Spk LimLV2", {}, apply);
            menu_.appendItem(&speakerSettings_.enableNoiseGate, "Spk N.Gate", enableTexts.begin(), enableTexts.end(), {}, apply);
            menu_.appendItem(&speakerSettings_.noiseGateThreshold, {0, 3}, "Spk N.G.Th", {}, apply);
#endif
        }

        static constexpr const char *
            infoModeTexts[] = {"NONE", "FPS ONLY", "ALL"};
        menu_.appendItem(&infoMode_, "INFO", std::begin(infoModeTexts), std::end(infoModeTexts));

        menu_.appendItem("CLOSE MENU", {},
                         [&]
                         { menu_.close(); });

        infoMode_ = NVSettings::instance().getState().infoMode;

#if !defined(NDEBUG) && 0
        menu_.appendItem("DUMP FB", "Dump FrameBuffer",
                         [&]
                         { frameBuffer_.requestDump(); });
        menu_.appendItem("DUMP SEND", "Dump Send Timing",
                         [&]
                         { requestSendLoopDump_ = true; });
#endif

        menu_.setCloseFunc([&]
                           {
                            auto& nvs = NVSettings::instance();
                            nvs.setSpeakerGain(speakerSettings_.gainDB);
                            nvs.setLatestInput(static_cast<device::SignalInput>(currentInput_));
                            nvs.setInfoMode(infoMode_);
                            // if (!inputSelClose)
                            // {
                            //     capture_.onMenuClose();
                            // }
                            inputSelClose = false;
                            nvs.flash(); });
    }

    int frameTick_ = 1;
    bool resetSignalDetection_ = true;

    //////////////////////////
    //////////////////////////
    void __not_in_flash_func(mainProcFixImage)()
    {
        int yofs = 0;
        auto prevTick = util::getSysTickCounter24();

        while (true)
        {
            watchdog_update();

            textPlane_.clear();
            auto w = bmp_->getWidth();
            auto h = bmp_->getHeight();
            auto *img = (uint8_t *)bmp_->getBits() + 3 * w * (h - 1);
            int stride = -w * 3;

            for (int y = 0; y < 240; ++y)
            {
                int lineID = frameBuffer_.allocateLine();
                auto p = frameBuffer_.getLineBuffer(lineID);
                graphics::convertBGRB888toBGR565(p, img + stride * ((y + yofs) % h), w);
                frameBuffer_.commitNextLine(lineID);
            }

            auto curTick = util::getSysTickCounter24();
            auto frameTick = (prevTick - curTick) & 0xffffff;
            frameTick_ = frameTick;
            auto fps16 = hw_divider_u32_quotient_inlined(CPU_CLOCK_KHZ * 1000 * 16, frameTick);
            prevTick = curTick;

            // printf("%d %d\n", fps16 >> 4, yofs);

            drawStatusLine(fps16);
            menu_.render(textPlane_);

            textPlane_.flip();
            frameBuffer_.finishPlane();
            // printf("y=%d\n", yofs);

            ++yofs;
        }
    }

    //////////////////////////
    //////////////////////////
    void __not_in_flash_func(mainProcVideoCapture)()
    {
        initMenu();
        MasterParameterCommunicator mpc;
        mpc.init(i2cIF);

        video::MasterVideoCaptureHandler mvch(&capture_);
        video::RemoteVideoCaptureHandler rvch(&mpc);

        video::VideoCaptureController captureControllers[2];
        captureControllers[0].setADV7181(&deviceSet_[0].adv7181);
        captureControllers[0].setHandler(&mvch);
        captureControllers[0].setID(0);

        captureControllers[1].setADV7181(&deviceSet_[1].adv7181);
        captureControllers[1].setHandler(&rvch);
        captureControllers[1].setID(1);

        int nCaptures = deviceSet_[1].enabled ? 2 : 1;
        DBGPRINT("nCaptures=%d\n", nCaptures);

        capture_.setFrameBuffer(&frameBuffer_);
        capture_.setTextPlane(&textPlane_);
        setVideoCaptureIRQ();

        capture_.setResampleParams(648, 0, 55, 0);

        auto prevTick = util::getSysTickCounter24();

        watchdog_enable(5000, true);

        bool ledState = true;

        while (true)
        {
            watchdog_update();

            textPlane_.clear();
            gpio_put(PICO_DEFAULT_LED_PIN, ledState & 1);
            ledState ^= true;

            auto curButtons = deviceSet_[0].pca9554.input();
            menu_.update(curButtons);

            for (int i = 0; i < nCaptures; ++i)
            {
                if (resetSignalDetection_)
                {
                    captureControllers[i].resetSignalDetection();
                    resetSignalDetection_ = false;
                }
                captureControllers[i].tick();
            }
            for (int i = 0; i < nCaptures; ++i)
            {
                captureControllers[i].wait();
            }

            auto curTick = util::getSysTickCounter24();
            auto frameTick = (prevTick - curTick) & 0xffffff;
            frameTick_ = frameTick;
            auto fps16 = hw_divider_u32_quotient_inlined(CPU_CLOCK_KHZ * 1000 * 16, frameTick);
            prevTick = curTick;

            // printf("%d %d\n", fps16 >> 4, yofs);

            drawStatusLine(fps16);
            menu_.render(textPlane_);

            textPlane_.flip();
            frameBuffer_.finishPlane();
            // printf("y=%d\n", yofs);
        }
    }

    void __not_in_flash_func(mainProcSlaveCapture)()
    {
        capture_.setFrameBuffer(&frameBuffer_);
        setVideoCaptureIRQ();

        capture_.setResampleParams(648, 0, 53, 0);

        video::RemoteVideoCaptureManager capManager(&capture_);
        capManager.setAfterCaptureFunc([&]
                                       {
                                           textPlane_.flip();
                                           frameBuffer_.finishPlane();
                                           textPlane_.clear();
                                           watchdog_update(); });

        SlaveParameterCommunicator spc;
        spc.setRemoteVideoCaptureManager(&capManager);
        spc.init(i2cIF, getI2CSlaveAddress(I2CTargetType::VIDEO_BOARD_SLAVE), PIN_SDA, PIN_SCL);

        capManager.loop();
    }

#if USE_VIDEO_DESERIALIZER
    void __not_in_flash_func(mainProcDeserialize)()
    {
        initializeDeserializerProgram(pioPWM_);

        VideoDeserializer deserializer;
        deserializer.init(SM_VIDEO_IN, PIN_VIDEO_IN0);

        deserializer.setOffset(-32);

        auto prevTick = util::getSysTickCounter24();

        while (true)
        {
            watchdog_update();
            deserializer.startReceive(&frameBuffer_);
            deserializer.waitTransfer();

            auto curTick = util::getSysTickCounter24();
            frameTick_ = (prevTick - curTick) & 0xffffff;
            prevTick = curTick;

            frameBuffer_.finishPlane();
        }
    }

    void __not_in_flash_func(mainProcDualDeserialize)()
    {
        initializeDeserializerProgram(pioPWM_);

        LineCompositor compositor(&frameBuffer_);
#if 0
        compositor.setLineInfo(false, 320 - 80, 0, 80);
        compositor.setLineInfo(true, 0, 80, 320 - 80);
#else
        compositor.setLineInfo(true, 0, 0, 288);
        compositor.setLineInfo(false, 0, 288, 32);
#endif

        VideoDeserializer2 deserializers[2]{{&compositor, false}, {&compositor, true}};
        deserializers[0].init(SM_VIDEO_IN, PIN_VIDEO_IN0, 0);
        deserializers[1].init(SM_VIDEO_IN_1, PIN_VIDEO_IN1, 1);

        auto prevTick = util::getSysTickCounter24();

        while (true)
        {
            watchdog_update();

            compositor.clear();

            for (auto &deserializer : deserializers)
            {
                deserializer.startReceive(&frameBuffer_);
            }
            for (auto &deserializer : deserializers)
            {
                deserializer.waitTransfer();
            }

            compositor.finish();

            auto curTick = util::getSysTickCounter24();
            frameTick_ = (prevTick - curTick) & 0xffffff;
            prevTick = curTick;

            auto fps16 = hw_divider_u32_quotient_inlined(CPU_CLOCK_KHZ * 1000 * 16, frameTick_);
            // DBGPRINT("%d.%d\n", fps16 >> 4, (fps16 & 15) * 10);

            auto tt0 = util::getSysTickCounter24();
            frameBuffer_.finishPlane();
            auto tt1 = util::getSysTickCounter24();

#if 0
            static int ct = 100;
            if (--ct == 0)
            {
                DBGPRINT("finishPlane %d\n", (tt0 - tt1) & 0xffffff);
            }
#endif
        }
    }
#endif // USE_VIDEO_DESERIALIZER
};

////////////////////////

void __not_in_flash_func(enableVideoCaptureIRQ)(bool f)
{
    dma_hw->ints1 = 1 << dmaChVideoIn;
    dma_channel_set_irq1_enabled(dmaChVideoIn, f);
}

namespace
{
    LEDDriver driver_;
}

////////////////////////

void __not_in_flash_func(core1_main)()
{
    util::initSysTick();
    driver_.loop();
}

void __not_in_flash_func(loopLCDPreview)()
{
    util::initSysTick();

    device::initLCD();

    gpio_init(28);
    gpio_set_dir(28, GPIO_OUT);

    while (1)
    {
        auto t0 = util::getSysTickCounter24();
        device::drawLCD(driver_.frameBuffer_);
        auto t1 = util::getSysTickCounter24();
        auto dt = (t0 - t1) & 0xffffff;
        // printf("dt %d\n", dt);
    }
}

#if USE_DATA_SERIALIZER
void __not_in_flash_func(loopDataSerializer)()
{
    util::initSysTick();

    VideoSerializer serializer;
    serializer.init(pio0_hw, 0, PIN_OUT_DATA0, PIN_OUT_CLK, PIN_OUT_SYNC);

    while (1)
    {
        serializer.sendFrame(driver_.frameBuffer_);
    }
}
#endif

struct Test
{
    int v;
    Test()
    {
        constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, 1);

        v = LED_PIN;
    }
};

Test test_;

void initBoard()
{
#if BOARDTYPE_SEPARATE_VIDEO
    gpio_init(PIN_VSYNC);
    gpio_set_dir(PIN_VSYNC, GPIO_IN);
#endif
}

int main()
{
#if 0
    // vreg_set_voltage(VREG_VOLTAGE_1_20);
    // sleep_ms(10);
    // set_sys_clock_khz(252000UL, true);

    stdio_init_all();
    device::testLCD();
    return 0;
#endif

    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(CPU_CLOCK_KHZ, true);
    stdio_init_all();

    util::initSysTick();

    constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    printf("\n\nstart.\n");

    // NVSettings::instance().load();
    graphics::initImageProcessor();

    initBoard();
    initPIO();
    initDMA();

#if BOARDTYPE_SEPARATE_VIDEO
    // Video board
    gpio_init(PIN_SLAVE_SELECT);
    gpio_set_dir(PIN_SLAVE_SELECT, GPIO_IN);
    gpio_pull_up(PIN_SLAVE_SELECT);
    sleep_ms(1);
    bool isVideoMaster = gpio_get(PIN_SLAVE_SELECT);
    DBGPRINT("Video Master : %d\n", isVideoMaster);

    I2CTargetType i2cTargetType = isVideoMaster ? I2CTargetType::VIDEO_BOARD_MASTER : I2CTargetType::VIDEO_BOARD_SLAVE;

    gpio_init(PIN_OUT_SYNC);
    gpio_set_dir(PIN_OUT_SYNC, GPIO_OUT);
    gpio_put(PIN_OUT_SYNC, 1);

#else
    bool isVideoMaster = ENABLE_VIDEO_BOARD;

#if BOARDTYPE_SEPARATE_LED
    // LED board

    gpio_init(PIN_VIDEO_IN0_SYNC);
    gpio_set_dir(PIN_VIDEO_IN0_SYNC, GPIO_IN);

    DBGPRINT("Wait for Video Sync 0...\n");
    while (!gpio_get(PIN_VIDEO_IN0_SYNC))
    {
        sleep_ms(1);
    }
    DBGPRINT("Video Sync 0 detected.\n");

    gpio_init(PIN_VIDEO_IN1_SYNC);
    gpio_set_dir(PIN_VIDEO_IN1_SYNC, GPIO_IN);
    gpio_pull_down(PIN_VIDEO_IN1_SYNC);

    DBGPRINT("Check for Video Sync 1...\n");
    auto rightDetected = []
    {
        for (int i = 0; i < 1000; ++i)
        {
            if (gpio_get(PIN_VIDEO_IN1_SYNC))
            {
                DBGPRINT("Video Sync 1 detected.\n");
                return true;
            }
            sleep_ms(1);
        }
        return false;
    }();
    DBGPRINT("Right LED = %d\n", rightDetected);

    I2CTargetType i2cTargetType =
        rightDetected ? I2CTargetType::LED_PANEL_DRIVER_RIGHT : I2CTargetType::LED_PANEL_DRIVER_LEFT;
#else
    I2CTargetType i2cTargetType = I2CTargetType::VIDEO_BOARD_MASTER;
#endif
#endif

    if (isVideoMaster)
    {
        // i2c
        i2c_init(i2cIF, 400000);
        gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);

#if 0
        for (int addr = 0; addr < (1 << 7); ++addr)
        {
            if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78)
            {
                continue;
            }
            uint8_t rxdata;
            auto r = i2c_read_blocking(i2cIF, addr, &rxdata, 1, false);
            if (r >= 0)
            {
                DBGPRINT("I2C Device Found : %02x\n", addr);
            }
        }
#endif
        deviceSet_[0].init(true);
        deviceSet_[1].init(false);
        tpa2016_.init(i2cIF);

        if (deviceSet_[1].enabled)
        {
            MasterParameterCommunicator mpc;
            mpc.init(i2cIF);

            DBGPRINT("Request Reset SlaveVideo\n");
            mpc.requestReset(I2CTargetType::VIDEO_BOARD_SLAVE);
        }
    }

    gpio_put(LED_PIN, 1);

    driver_.init();

#if 0
    sleep_ms(1000);
    adv7181_.selectInput(device::SignalInput::COMPONENT);

    driver_.capture_.simpleCaptureTest();
    while (1)
        ;
#endif

    if (isVideoMaster)
    {
        callPCA9554([](auto &pca9554_)
                    { pca9554_.setPortDir(0b00111111); });

        // auto defaultInput = device::SignalInput::COMPOSITE;
        auto defaultInput = device::SignalInput::RGB21;
        // auto defaultInput = device::SignalInput::COMPONENT;
        callADV7181([&](auto &adv7181_)
                    { adv7181_.selectInput(defaultInput); });
        device::selectAudioInput(deviceSet_[0].pca9554, defaultInput);

        // adv7181_.setPLL(true, false, 900, 15980);

        if (tpa2016_.isExist())
        {
            speakerSettings_.gainDB = NVSettings::instance().getState().speakerGain;
            speakerSettings_.apply(tpa2016_);
        }
    }

    if (ENABLE_LED_PANEL)
    {
        multicore_launch_core1(core1_main);
    }

#if USE_LCD_PREVIEW
    {
        multicore_launch_core1(loopLCDPreview);
    }
#endif

#if USE_DATA_SERIALIZER
    {
        multicore_launch_core1(loopDataSerializer);
    }
#endif

#if USE_VIDEO_DESERIALIZER
    if (rightDetected)
    {
        driver_.mainProcDualDeserialize();
    }
    else
    {
        driver_.mainProcDeserialize();
    }
#else

#if BOARDTYPE_SEPARATE_VIDEO
    if (!isVideoMaster)
    {
        driver_.mainProcSlaveCapture();
    }
#endif

    driver_.mainProcVideoCapture();
#endif
    return 0;
}
