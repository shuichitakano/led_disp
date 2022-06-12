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
#include <cassert>
#include <cstdio>
#include <cstring>

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
static constexpr int BPP = 16;
static constexpr int PWM_PULSE_PER_LINE = 74;

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

static constexpr int PIOIRQ_DATAIDLE = 0;

uint ofsPWM;
uint ofsVideoIn;
uint ofsData;
uint ofsCommand;

void initPIO()
{
    ofsPWM = pio_add_program(pioPWM_, &led_pwm_program);
    ofsData = pio_add_program(pioDataCmd_, &led_data_program);
    ofsCommand = pio_add_program(pioDataCmd_, &led_command_program);
    printf("pwm:%d, data:%d, cmd:%d\n", ofsPWM, ofsData, ofsCommand);

    initProgramLEDPWM(pioPWM_, SM_PWM, ofsPWM, PIN_A, PIN_OE);

    initProgramLEDData1(pioDataCmd_, SM_DATA1, ofsData, PIN_R1, PIN_CLK, PIN_LAT);
    initProgramLEDData23(pioDataCmd_, SM_DATA2, ofsData, PIN_G1);
    initProgramLEDData23(pioDataCmd_, SM_DATA3, ofsData, PIN_B1);
    initProgramLEDCommand(pioDataCmd_, SM_COMMAND, ofsCommand, PIN_RGB_TOP, PIN_CLK, PIN_LAT);

    static constexpr int clkdiv25 = CPU_CLOCK_KHZ / 25000;
    pio_sm_set_clkdiv_int_frac(pioPWM_, SM_PWM, clkdiv25, 0);
    static constexpr int clkdiv125 = 2;
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA1, clkdiv125, 0);
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA2, clkdiv125, 0);
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_DATA3, clkdiv125, 0);
    pio_sm_set_clkdiv_int_frac(pioDataCmd_, SM_COMMAND, clkdiv125, 0);

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
// static constexpr int dmaChChain1 = 4;
// static constexpr int dmaChChain2 = 5;
// static constexpr int dmaChChain3 = 6;
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
}

// LED PWM の1周期ぶんの出力を開始する
void startLEDPWM(int clocksPerLine, int nLines)
{
    clocksPerLine -= 1;
    nLines -= 2;

    uint32_t data = (nLines << 16) | (clocksPerLine << 0);
    pio_sm_put_blocking(pioPWM_, SM_PWM, data);
}

void startLEDPWM(int clocksPerLine, int nLines, int n)
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

    // LAT = 0
    pio_sm_exec(pioDataCmd_, SM_COMMAND, pio_encode_set(pio_pins, 0));
}

void setDataTransferParams(int sm, int unitW, int h, int nCascades, int bpp)
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

void startDataTransfer()
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
    while (1)
    {
        auto pc = pio_sm_get_pc(pioDataCmd_, SM_DATA1);
        if (pc == ofsData || pc == ofsData + 1)
        {
            break;
        }
    }

    hw_clear_bits(&pioDataCmd_->ctrl, 7u << PIO_CTRL_SM_ENABLE_LSB);
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
    static constexpr int N_DATA_BUFFERS = 2;
    static constexpr int N_RGB_CH = 3;

    // 先頭 1 word がラインサイズ。1 word が guard
    uint32_t dataBuffer_[N_DATA_BUFFERS][N_RGB_CH][UNIT_DATA_BUFFER_SIZE + 2]; // 22.5KB

    graphics::FrameBuffer frameBuffer_;

    void init()
    {
        LEDDriverInst_ = this;

        constexpr int margin_lines = 2;
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

    void scan()
    {
        startLEDPWM(PWM_PULSE_PER_LINE, N_SCAN_LINES);
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

    volatile bool dataTransferComplete_ = true;

    void initFrameState()
    {
        dataTransferComplete_ = true;
    }

    bool isDataTransferCompleted()
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

    void __not_in_flash_func(updateDataBuffer)(int dataBufferID, int y)
    {
        auto *dstR = dataBuffer_[dataBufferID][0];
        auto *dstG = dataBuffer_[dataBufferID][1];
        auto *dstB = dataBuffer_[dataBufferID][2];

        int lineID0 = frameBuffer_.moveCurrentPlaneLine(y);
        int lineID1 = frameBuffer_.moveCurrentPlaneLine(y + N_SCAN_LINES * 1);
        int lineID2 = frameBuffer_.moveCurrentPlaneLine(y + N_SCAN_LINES * 2);
        int lineID3 = frameBuffer_.moveCurrentPlaneLine(y + N_SCAN_LINES * 3);

        auto *line0 = frameBuffer_.getLineBuffer(lineID0);
        auto *line1 = frameBuffer_.getLineBuffer(lineID1);
        auto *line2 = frameBuffer_.getLineBuffer(lineID2);
        auto *line3 = frameBuffer_.getLineBuffer(lineID3);

        // 先頭に ラインの繰り返し数
        dstR[0] = UNIT_WIDTH - 1;
        dstG[0] = UNIT_WIDTH - 1;
        dstB[0] = UNIT_WIDTH - 1;

        graphics::convert4(dstR + 1, dstG + 1, dstB + 1, line0, line1, line2, line3);

        frameBuffer_.freeLine({lineID0, lineID1, lineID2, lineID3});
    }

    void __not_in_flash_func(dmaIRQHandler)()
    {
        if (dma_hw->ints0 & (1 << dmaChData1))
        {
            dma_hw->ints0 = 1 << dmaChData1;
            dataTransferComplete_ = true;
        }
    }

    static void __isr __not_in_flash_func(dmaIRQHandlerEntry)()
    {
        LEDDriverInst_->dmaIRQHandler();
    }

    void __not_in_flash_func(loop)()
    {
        irq_set_exclusive_handler(DMA_IRQ_0, dmaIRQHandlerEntry);
        dma_channel_set_irq0_enabled(dmaChData1, true);
        irq_set_enabled(DMA_IRQ_0, true);

        int led = 0;

        while (1)
        {
            static int frame = 0;
            printf("frame %d\n", frame++);

            // まず VSync 等のコマンドを送る
            sendCommand(command0_.data(), command0_.size());
            finishCommandTransfer();

            // ここから scan はじめていいはず
            startLEDPWM(PWM_PULSE_PER_LINE, N_SCAN_LINES, 23);
            // todo: 回数はフレームレートから計算する

            // 残りのコマンドを送る
            sendCommand(command1_.data(), command1_.size());
            finishCommandTransfer();

            // データ送出の SM を開始 (DCLK 送出を開始)
            initFrameState();
            startDataTransfer();

            int dbid = 0;
            for (int i = 0; i < N_SCAN_LINES; ++i)
            {
                updateDataBuffer(dbid, i);
                waitDataTransfer(); // DMAまち
                dataTransferComplete_ = false;
                transferData({dataBuffer_[dbid][0],
                              dataBuffer_[dbid][1],
                              dataBuffer_[dbid][2]},
                             UNIT_DATA_BUFFER_SIZE + 1); // DMA開始
                dbid ^= 1;
            }

            finishLEDPWM();
            waitForScanStall();

            waitDataTransfer();
            finishDataTransfer();
            assert(dataTransferComplete_);

            // sleep_us(1);
            frameBuffer_.flipReadPlane();

            gpio_put(PICO_DEFAULT_LED_PIN, led);
            led ^= 1;
        }
    }

    void __not_in_flash_func(mainProc)()
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
#if 1
#if 1
                int y2 = (y + yofs) % 240;
                if (y2 < h)
                {
                    memset(p, 0, 160 * 2);
                    graphics::convertBGRB888toBGR565(p + 160, img + stride * y2, w);
                }
                else
                {
                    memset(p, 0, 320 * 2);
                }
#else
                graphics::convertBGRB888toBGR565(p, img + stride * ((y + yofs) % h), w);
                graphics::convertBGRB888toBGR565(p + 160, img + stride * ((y + yofs) % h), w);
#endif
#endif
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
    driver_.mainProc();
    return 0;
}
