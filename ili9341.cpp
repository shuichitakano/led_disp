/*
 * author : Shuichi TAKANO
 * since  : Sat Aug 10 2024 17:27:29
 */

#include "ili9341.h"

#include <hardware/pio.h>
#include <hardware/gpio.h>
#include <pico/time.h>
#include <initializer_list>
#include <array>

#include <simple_serial.pio.h>

#include "bmp.h"
#include "test_320x240.h"

namespace
{
    static constexpr int PIN_DC = 20;
    static constexpr int PIN_CS = 22;
    static constexpr int PIN_SCK = 18;
    static constexpr int PIN_MOSI = 19;
    static constexpr int PIN_RST = 21;

    static constexpr int SM_LCD = 0;
    auto *pioLCD_ = pio0;
    uint programOffset_ = 0;
}

void __not_in_flash_func(writeLCD)(uint8_t x)
{
    while (pio_sm_is_tx_fifo_full(pioLCD_, SM_LCD))
        ;
    *(volatile uint8_t *)&pioLCD_->txf[SM_LCD] = x;
}

void __not_in_flash_func(writeLCD32)(uint32_t x)
{
    while (pio_sm_is_tx_fifo_full(pioLCD_, SM_LCD))
        ;
    *(volatile uint32_t *)&pioLCD_->txf[SM_LCD] = x;
}

void waitLCD()
{
    uint32_t sm_stall_mask = 1u << (SM_LCD + PIO_FDEBUG_TXSTALL_LSB);
    pioLCD_->fdebug = sm_stall_mask;
    while (!(pioLCD_->fdebug & sm_stall_mask))
        ;
}

void setLCD_DC_CS(bool dc, bool cs)
{
    waitLCD();
    sleep_us(1);
    gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

namespace device
{

    //            sm_config_set_clkdiv(&c, clk_div);

    void sendCommand(uint8_t command,
                     std::initializer_list<uint8_t> data = {})
    {
        setLCD_DC_CS(false, false);
        writeLCD(command);

        if (data.size())
        {
            waitLCD();
            setLCD_DC_CS(true, false);
            for (auto d : data)
            {
                writeLCD(d);
            }
        }
        setLCD_DC_CS(false, true);
    }

    void setMemoryAccessControl(uint8_t rotation)
    {
        uint8_t mac = 0;

        switch (rotation)
        {
        case 0:
            mac = 0x40 | 0x08; // Default orientation
            break;
        case 1:
            mac = 0x20 | 0x08; // Rotate 90 degrees
            break;
        case 2:
            mac = 0x80 | 0x08; // Rotate 180 degrees
            break;
        case 3:
            mac = 0x40 | 0x80 | 0x20 | 0x08; // Rotate 270 degrees
            break;
        }

        sendCommand(0x36, {mac});
    }

    void setPixelFormat(uint8_t format)
    {
        sendCommand(0x3A, {format});
    }

    void setFrameRateControl(uint8_t mode, uint8_t frameRate)
    {
        sendCommand(0xB1, {mode, frameRate});
    }

    void setDrawingArea(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
    {
        int x1 = x + width - 1;
        int y1 = y + height - 1;

        // Column Address Set
        sendCommand(0x2A, {static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(x & 0xFF),
                           static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1 & 0xFF)});

        // Row Address Set
        sendCommand(0x2B, {static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(y & 0xFF),
                           static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1 & 0xFF)});
    }

    void initLCD()
    {
        auto initGPIO = [](int pin)
        {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
        };
        initGPIO(PIN_DC);
        initGPIO(PIN_CS);
        initGPIO(PIN_RST);
        initGPIO(PIN_SCK);
        initGPIO(PIN_MOSI);

        programOffset_ = pio_add_program(pioLCD_, &simple_serial_program);
        initProgramSimpleSerial(pioLCD_, SM_LCD, programOffset_, PIN_MOSI, PIN_SCK);

        gpio_put(PIN_RST, 0);
        sleep_ms(10);
        gpio_put(PIN_RST, 1);
        sleep_ms(120);

        sendCommand(0x01); // Reset
        sleep_ms(5);

        sendCommand(0x28); // Display OFF

        sendCommand(0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}); // Power Control A
        sendCommand(0xCF, {0x00, 0xC1, 0x30});             // Power Control B
        sendCommand(0xE8, {0x85, 0x00, 0x78});             // Driver Timing Control A
        sendCommand(0xEA, {0x00, 0x00});                   // Driver Timing Control B
        sendCommand(0xED, {0x64, 0x03, 0x12, 0x81});       // Power On Sequence Control
        sendCommand(0xF7, {0x20});                         // Pump Ratio Control
        sendCommand(0xC0, {0x23});                         // Power Control, VRH[5:0]
        sendCommand(0xC1, {0x10});                         // Power Control, SAP[2:0]; BT[3:0]
        sendCommand(0xC5, {0x3E, 0x28});                   // VCM Control
        sendCommand(0xC7, {0x86});                         // VCM Control 2
        sendCommand(0x36, {0x48});                         // Memory Access Control
        sendCommand(0x3A, {0x55});                         // Pixel Format, 16bit
        sendCommand(0xB1, {0x00, 0x18});                   // Frame Rate Control
        sendCommand(0xB6, {0x08, 0x82, 0x27});             // Display Function Control
        sendCommand(0xF2, {0x00});                         // 3Gamma Function Disable
        sendCommand(0x26, {0x01});                         // Gamma Curve Selected

        sendCommand(0xE0, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                           0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}); // Positive Gamma Correction

        sendCommand(0xE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                           0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}); // Negative Gamma Correction

        sendCommand(0x11); // Sleep OUT
        sleep_ms(120);
        sendCommand(0x29); // Display ON

        ///
        setMemoryAccessControl(1);

        setPixelFormat(0x55); // 16 bit color
        // setFrameRateControl(0x00, 0x18);　// 79Hz
        setFrameRateControl(0x00, 0x10); // 119Hz

        setDrawingArea(0, 0, 320, 240);
    }

    void startSendPixelData()
    {
        sendCommand(0x2C); // メモリ書き込み開始
        setLCD_DC_CS(true, false);

        waitLCD();
        setProgramSimpleSerialConfig(pioLCD_, SM_LCD, programOffset_, PIN_MOSI, PIN_SCK, 32);
    }

    void endSendPixelData()
    {
        waitLCD();
        setProgramSimpleSerialConfig(pioLCD_, SM_LCD, programOffset_, PIN_MOSI, PIN_SCK, 8);
        setLCD_DC_CS(false, true);
    }

    void __not_in_flash_func(sendPixelData)(const uint16_t *colors, size_t count)
    {
        // sendCommand(0x2C); // メモリ書き込み開始
        // setLCD_DC_CS(true, false);

#if 0
        for (size_t i = 0; i < count; i++)
        {
            uint16_t color = colors[i];
            writeLCD(color >> 8);
            writeLCD(color & 0xFF);
        }
#else
        auto *p = reinterpret_cast<const uint8_t *>(colors);
        auto *tail = p + count * 2;
        while (p < tail)
        {
            auto v0 = p[0] << 24;
            auto v1 = p[1] << 16;
            auto v2 = p[2] << 8;
            auto v3 = p[3] << 0;
            writeLCD32(v0 | v1 | v2 | v3);
            p += 4;
        }
#endif

        // setLCD_DC_CS(false, true);
    }

    constexpr uint16_t makeColor(int r, int g, int b)
    {
        return ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);
    }

    void fillScreen(uint16_t color)
    {
        setDrawingArea(0, 0, 320, 240);
        std::array<uint16_t, 320> line;

        startSendPixelData();

        line.fill(color);

        for (int i = 0; i < 240; i++)
        {
            sendPixelData(line.data(), line.size());
        }

        endSendPixelData();
    }

    void testLCD()
    {
        initLCD();

        int yofs = 0;
        while (true)
        {
#if 0
            for (int r = 0; r < 32; ++r)
            {
                for (int g = 0; g < 64; ++g)
                {
                    for (int b = 0; b < 32; ++b)
                    {
                        fillScreen(makeColor(r, g, b));
                    }
                }
            }
#else
            auto *bmp = (const graphics::BMP *)MagickImage;

            int w = bmp->getWidth();
            int h = bmp->getHeight();
            auto *img = (uint8_t *)bmp->getBits() + 3 * w * (h - 1);
            int stride = -w * 3;

            setDrawingArea(0, 0, w, h);
            startSendPixelData();
            for (int y = 0; y < h; ++y)
            {
                auto *p = img + ((y + yofs) % h) * stride;
                for (int x = 0; x < w; ++x)
                {
                    int r = p[0];
                    int g = p[1];
                    int b = p[2];
                    auto color = makeColor(r >> 3, g >> 2, b >> 3);
                    writeLCD(color >> 8);
                    writeLCD(color & 0xFF);
                    p += 3;
                }
            }

            endSendPixelData();
            ++yofs;
#endif
        }
    }

    void __not_in_flash_func(drawLCD)(graphics::FrameBuffer &fb)
    {
        int w = fb.getWidth();
        int h = fb.getHeight();

        setDrawingArea(0, 0, w, h);
        startSendPixelData();

        for (int y = 0; y < h; ++y)
        {
            int line = fb.moveCurrentPlaneLine(y);
            auto *data = fb.getLineBuffer(line);
            sendPixelData(data, w);
            fb.freeLine({line});
        }

        endSendPixelData();
        fb.flipReadPlane();
    }

    // 320*240*60*16*2 = 147456000 以上に overclock 必要
}
