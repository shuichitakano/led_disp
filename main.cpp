//#include <stdio.h>
#include <stdint.h>

#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/vreg.h>
#include <vector>
#include <array>
#include <assert.h>
#include <cstdio>

#include <led.pio.h>
#include <bt656.pio.h>

#include "image_proc.h"
#include "bmp.h"
#include "framebuffer.h"

namespace
{
    constexpr uint8_t fontData_[] = {
#include "font.h"
    };

#include "test.h"

}

static constexpr uint32_t CPU_CLOCK_KHZ = 250000;

// LED data
static constexpr uint32_t PIN_R1 = 4;
static constexpr uint32_t PIN_R2 = 5;
static constexpr uint32_t PIN_G1 = 6;
static constexpr uint32_t PIN_G2 = 7;
static constexpr uint32_t PIN_B1 = 8;
static constexpr uint32_t PIN_B2 = 9;
static constexpr uint32_t PIN_RGB_TOP = PIN_R1;

// row select
static constexpr uint32_t PIN_A = 10;
static constexpr uint32_t PIN_B = 11;
static constexpr uint32_t PIN_C = 12;

// control
static constexpr uint32_t PIN_CLK = 2;
static constexpr uint32_t PIN_FFCLK = 3;
static constexpr uint32_t PIN_OE = 13;
static constexpr uint32_t PIN_LAT = 28;

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

// I2C
static constexpr uint32_t PIN_SDA = 26;
static constexpr uint32_t PIN_SCL = 27;

//
static constexpr int UNIT_WIDTH = 16;
static constexpr int N_CASCADE = 10; // 16 * 10
static constexpr int N_SCAN_LINES = 60;

#if 0
#define SLEEP sleep_us(0)
#else
#define SLEEP \
    do        \
    {         \
    } while (0)
#endif

auto *pioDataCmd_ = pio0;
auto *pioPWM_ = pio1;
static constexpr int SM_DATA1 = 0;
static constexpr int SM_DATA2 = 1;
static constexpr int SM_DATA3 = 2;
static constexpr int SM_COMMAND = 3;
static constexpr int SM_PWM = 0;
static constexpr int SM_VIDEO_IN = 1;

uint ofsPWM;
uint ofsVideoIn;
uint ofsData1;
uint ofsData23;
uint ofsCommand;

void initPIO()
{
    ofsPWM = pio_add_program(pioPWM_, &led_pwm_program);
    ofsData1 = pio_add_program(pioDataCmd_, &led_data_program);
    ofsData23 = pio_add_program(pioDataCmd_, &led_data_program);
    ofsCommand = pio_add_program(pioDataCmd_, &led_command_program);
    initProgramLEDPWM(pioPWM_, SM_PWM, ofsPWM, PIN_A, PIN_OE);

    initProgramLEDData1(pioDataCmd_, SM_DATA1, ofsData1, PIN_R1, PIN_CLK, PIN_LAT);
    initProgramLEDData23(pioDataCmd_, SM_DATA2, ofsData23, PIN_G1);
    initProgramLEDData23(pioDataCmd_, SM_DATA3, ofsData23, PIN_B1);
    initProgramLEDCommand(pioDataCmd_, SM_COMMAND, ofsCommand, PIN_RGB_TOP, PIN_CLK, PIN_LAT);

    static constexpr int clkdiv = CPU_CLOCK_KHZ / 25000;
    pio_sm_set_clkdiv_int_frac(pioPWM_, SM_PWM, clkdiv, 0);
    // pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA1, clkdiv, 0);
    // pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA2, clkdiv, 0);
    // pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA3, clkdiv, 0);
    // pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_COMMAND, clkdiv, 0);

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
}

static constexpr int dmaChData1 = 0;
static constexpr int dmaChData2 = 1;
static constexpr int dmaChData3 = 2;
static constexpr int dmaChCommand = 3;
static constexpr int dmaChChain1 = 4;
static constexpr int dmaChChain2 = 5;
static constexpr int dmaChChain3 = 6;
static constexpr int dmaChPWM = 7;

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

    initCH(dmaChData1, pioDataCmd_, SM_DATA1, dmaChChain1);
    initCH(dmaChData2, pioDataCmd_, SM_DATA2, dmaChChain2);
    initCH(dmaChData3, pioDataCmd_, SM_DATA3, dmaChChain3, true);
    initCH(dmaChCommand, pioDataCmd_, SM_COMMAND);

    auto initChain = [](int ch, int dataCh)
    {
        auto cfg = dma_channel_get_default_config(ch);
        dma_channel_configure(ch,
                              &cfg,
                              &dma_channel_hw_addr(dataCh)->al3_read_addr_trig,
                              NULL, 1,
                              false);
    };

    initChain(dmaChChain1, dmaChData1);
    initChain(dmaChChain2, dmaChData2);
    initChain(dmaChChain3, dmaChData3);
}

// LED PWM の1周期ぶんの出力を開始する
void startLEDPWM(int clocksPerLine, int nLines, int maskB)
{
    maskB ^= 0xf;
    clocksPerLine -= 1;
    nLines -= 2;

    uint32_t data = (maskB << 24) | (nLines << 16) | (clocksPerLine << 0);
    pio_sm_put_blocking(pioPWM_, SM_PWM, data);
}

void startLEDPWM(int clocksPerLine, int nLines, int maskB, int n)
{
    maskB ^= 0xf;
    clocksPerLine -= 1;
    nLines -= 2;

    static uint32_t dmaPWMData = (maskB << 24) | (nLines << 16) | (clocksPerLine << 0);

    auto cfg = dma_channel_get_default_config(dmaChPWM);
    channel_config_set_dreq(&cfg, pio_get_dreq(pioPWM_, SM_PWM, true /* tx */));
    channel_config_set_read_increment(&cfg, false);

    dma_channel_configure(dmaChPWM, &cfg, &pioPWM_->txf[SM_PWM],
                          &dmaPWMData, n, true);
}

void finishLEDPWM()
{
    dma_channel_wait_for_finish_blocking(dmaChPWM);
}

// Commandの出力を開始する
void sendCommand(const uint32_t *cmdBits, size_t size)
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

void resetPIOTxStalled(PIO pio, uint sm)
{
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
}

bool isPIOTxStalled(PIO pio, uint sm)
{
    return pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm));
}

bool isLEDPWMStalled()
{
    // return pio_sm_is_exec_stalled(pioPWM_, SM_PWM);
    return isPIOTxStalled(pioPWM_, SM_PWM);
}

bool isCommandTransferStalled()
{
    return isPIOTxStalled(pioDataCmd_, SM_COMMAND);
}

void finishCommandTransfer()
{
    dma_channel_wait_for_finish_blocking(dmaChCommand);

    resetPIOTxStalled(pioDataCmd_, SM_COMMAND);
    while (!isCommandTransferStalled())
    {
        tight_loop_contents();
    }

    pio_sm_set_enabled(pioDataCmd_, SM_COMMAND, false);
}

void setDataTransferParams(int sm, int unitW, int h, int nCascades, int bpp)
{
    // isr: 16 : nCascades * bpp - 2 = 10 * 16 - 2
    // y  : 16 : unitW * h - 1 = 16 * 120 / 2 - 1

    uint32_t isr = nCascades * bpp - 2;
    uint32_t y = unitW * h - 1;
    uint32_t data = (y << 16) | isr; // right shift
    // uint32_t data = (isr << 16) | y; // left shift

    pio_sm_set_enabled(pioDataCmd_, sm, false);
    pio_sm_put_blocking(pioDataCmd_, sm, data);
    pio_sm_exec(pioDataCmd_, sm, pio_encode_pull(false, false));
    pio_sm_exec(pioDataCmd_, sm, pio_encode_out(pio_isr, 16));
    pio_sm_exec(pioDataCmd_, sm, pio_encode_out(pio_y, 16));

    pio_sm_clear_fifos(pioDataCmd_, sm);
}

void startDataTransfer()
{
    setDataTransferParams(SM_DATA1, 16, N_SCAN_LINES, N_CASCADE, 16);
    setDataTransferParams(SM_DATA2, 16, N_SCAN_LINES, N_CASCADE, 16);
    setDataTransferParams(SM_DATA3, 16, N_SCAN_LINES, N_CASCADE, 16);
}

// void transferData(std::array<uint32_t *, 3> data, size_t size, bool first)
// {
//     dma_channel_wait_for_finish_blocking(dmaChData1);
//     for (int i = 0; i < 3; ++i)
//     {
//         // dma_channel_wait_for_finish_blocking(dmaChData1 + i);

//         dma_channel_set_trans_count(dmaChData1 + i, size, false);
//         dma_channel_set_read_addr(dmaChData1 + i, data[i], true);
//     }
//     if (first)
//     {
//         for (int i = 0; i < 3; ++i)
//         {
//             while (!pio_sm_is_tx_fifo_full(pioDataCmd_, SM_DATA1 + i))
//             {
//                 tight_loop_contents();
//             }
//         }
//         pio_enable_sm_mask_in_sync(pioDataCmd_, 7);
//         // hw_set_bits(&pioDataCmd_->ctrl, 7u << PIO_CTRL_SM_ENABLE_LSB);
//     }
// }

void finishDataTransfer()
{
    dma_channel_wait_for_finish_blocking(dmaChChain1);
    dma_channel_wait_for_finish_blocking(dmaChChain2);
    dma_channel_wait_for_finish_blocking(dmaChChain3);

    dma_channel_wait_for_finish_blocking(dmaChData1);
    dma_channel_wait_for_finish_blocking(dmaChData2);
    dma_channel_wait_for_finish_blocking(dmaChData3);

    resetPIOTxStalled(pioDataCmd_, SM_DATA1);
    while (!isPIOTxStalled(pioDataCmd_, SM_DATA1))
    {
        tight_loop_contents();
    }

    hw_clear_bits(&pioDataCmd_->ctrl, 7u << PIO_CTRL_SM_ENABLE_LSB);
}

void sendFrameData(std::array<uint32_t **, 3> lists, size_t size)
{
    for (int i = 0; i < 3; ++i)
    {
        dma_channel_wait_for_finish_blocking(dmaChChain1 + i);
        dma_channel_wait_for_finish_blocking(dmaChData1 + i);
        dma_channel_set_trans_count(dmaChData1 + i, size, false);

        auto list = lists[i];
        dma_channel_set_read_addr(dmaChChain1 + i, list, true);
    }
    for (int i = 0; i < 3; ++i)
    {
        while (!pio_sm_is_tx_fifo_full(pioDataCmd_, SM_DATA1 + i))
        {
            tight_loop_contents();
        }
    }
    pio_enable_sm_mask_in_sync(pioDataCmd_, 7);
}

////////////////////////
////////////////////////

struct LEDDriver;
namespace
{
    LEDDriver *LEDDriverInst_{};
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
    static constexpr int N_DATA_BUFFERS = 3;
    static constexpr int N_RGB_CH = 3;

    uint32_t dataBuffer_[N_DATA_BUFFERS][N_RGB_CH][UNIT_DATA_BUFFER_SIZE + 1]; // 22.5KB
    std::array<uint32_t *, N_SCAN_LINES + 1> chainBuffer_[3]{};

    graphics::FrameBuffer frameBuffer_;

    void init()
    {
        LEDDriverInst_ = this;

        constexpr int margin_lines = 2;
        frameBuffer_.initialize(TOTAL_WIDTH, TOTAL_HEIGHT, margin_lines);

        for (int i = 0; i < N_SCAN_LINES; ++i)
        {
            int b = i % N_DATA_BUFFERS;
            chainBuffer_[0][i] = dataBuffer_[b][0];
            chainBuffer_[1][i] = dataBuffer_[b][1];
            chainBuffer_[2][i] = dataBuffer_[b][2];
        }

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
    }

    void scan()
    {
        startLEDPWM(74, N_SCAN_LINES, 0xf);
    }

    void scanIfStalled()
    {
        if (isLEDPWMStalled())
        {
            scan();
        }
    }

    void waitForScanStall()
    {
        while (!isLEDPWMStalled())
        {
            tight_loop_contents();
        }
    }

    int dataBufferID_ = 0;
    int currentLine_ = 0;
    volatile bool dataTransferComplete_ = true;

    void startFrameTransfer()
    {
        dataBufferID_ = 0;
        currentLine_ = 0;
        dataTransferComplete_ = false;
    }

    bool isDataTransferCompleted()
    {
        return dataTransferComplete_;
    }

    void updateDataBuffer()
    {
        int y = currentLine_;
        if (y >= N_SCAN_LINES)
        {
            return;
        }

        auto *dstR = dataBuffer_[dataBufferID_][0];
        auto *dstG = dataBuffer_[dataBufferID_][1];
        auto *dstB = dataBuffer_[dataBufferID_][2];

        int lineID0 = frameBuffer_.moveCurrentPlaneLine(y);
        int lineID1 = frameBuffer_.moveCurrentPlaneLine(y + N_SCAN_LINES * 1);
        int lineID2 = frameBuffer_.moveCurrentPlaneLine(y + N_SCAN_LINES * 2);
        int lineID3 = frameBuffer_.moveCurrentPlaneLine(y + N_SCAN_LINES * 3);

        auto *line0 = frameBuffer_.getLineBuffer(lineID0);
        auto *line1 = frameBuffer_.getLineBuffer(lineID1);
        auto *line2 = frameBuffer_.getLineBuffer(lineID2);
        auto *line3 = frameBuffer_.getLineBuffer(lineID3);

        graphics::convert4(dstR, dstG, dstB, line0, line1, line2, line3);

        frameBuffer_.freeLine({lineID0, lineID1, lineID2, lineID3});

        ++currentLine_;
        if (++dataBufferID_ == N_DATA_BUFFERS)
        {
            dataBufferID_ = 0;
        }
    }

    void dmaIRQHandler()
    {
        if (dma_hw->ints0 & (1 << dmaChData1))
        {
            dma_hw->ints0 = 1 << dmaChData1;
            updateDataBuffer();
        }
        else if (dma_hw->ints0 & (1 << dmaChData3))
        {
            dma_hw->ints0 = 1 << dmaChData3;
            dataTransferComplete_ = true;
        }
    }

    static void __isr dmaIRQHandlerEntry()
    {
        LEDDriverInst_->dmaIRQHandler();
    }

    void loop()
    {
        irq_set_exclusive_handler(DMA_IRQ_0, dmaIRQHandlerEntry);
        dma_channel_set_irq0_enabled(dmaChData1, true);
        dma_channel_set_irq0_enabled(dmaChData3, true);
        irq_set_enabled(DMA_IRQ_0, true);

        int led = 0;

        while (1)
        {
            // printf("led %d\n", led);
            // dump(command0_);

            // ("%d\n", __LINE__);

            startFrameTransfer();
            for (int i = 0; i < N_DATA_BUFFERS; ++i)
            {
                updateDataBuffer();
            }

            sendCommand(command0_.data(), command0_.size());
            finishCommandTransfer();

            // scan();

            sendCommand(command1_.data(), command1_.size());
            finishCommandTransfer();

            // printf("%d\n", __LINE__);
            // sleep_us(1);
            startLEDPWM(74, N_SCAN_LINES, 0xf, 23);

            // int pc = pio_sm_get_pc(pioDataCmd_, SM_COMMAND);
            // printf("pc = %d\n", pc);

            startDataTransfer();
            // printf("%d\n", __LINE__);

            sendFrameData({chainBuffer_[0].data(),
                           chainBuffer_[1].data(),
                           chainBuffer_[2].data()},
                          UNIT_DATA_BUFFER_SIZE);

            while (!isDataTransferCompleted())
            {
                tight_loop_contents();
            }
            // printf("%d\n", __LINE__);
            finishDataTransfer();

            finishLEDPWM();
            waitForScanStall();

            sleep_us(1);
            // waitForScanStall();

            // printf("ct %d\n", procLine_);

            frameBuffer_.flipReadPlane();

            gpio_put(PICO_DEFAULT_LED_PIN, led);
            led ^= 1;
        }
    }

    void __not_in_flash_func(main)()
    {
        int yofs = 0;
        while (1)
        {
            auto w = bmp_->getWidth();
            auto h = bmp_->getHeight();
            auto *img = (uint8_t *)bmp_->getBits() + 3 * w * (h - 1);
            int stride = -w * 3;

            for (int y = 0; y < 240; ++y)
            {
                int lineID = frameBuffer_.allocateLine();
                auto p = frameBuffer_.getLineBuffer(lineID);

                graphics::convertBGRB888toBGR565(p, img + stride * ((y + yofs) % h), w);
                graphics::convertBGRB888toBGR565(p + 160, img + stride * ((y + yofs) % h), w);
                frameBuffer_.commitNextLine(lineID);
            }
            frameBuffer_.finishPlane();

            ++yofs;
        }
    }
};

////////////////////////

LEDDriver driver_;

void __not_in_flash_func(core1_main)()
{
    driver_.loop();
}

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(CPU_CLOCK_KHZ, true);
    stdio_init_all();

    constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    printf("\n\nstart.\n");
    graphics::initImageProcessor();

    initPIO();
    initDMA();

    gpio_put(LED_PIN, 1);

    driver_.init();

    multicore_launch_core1(core1_main);
    driver_.main();
    return 0;
}
