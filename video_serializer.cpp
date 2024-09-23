/*
 * author : Shuichi TAKANO
 * since  : Wed Aug 28 2024 03:01:55
 */

#include "video_serializer.h"

#include <hardware/gpio.h>
#include <hardware/dma.h>
#include <hardware/divider.h>
#include <pico/time.h>
#include <cstring>
#include "util.h"

#include <simple_serial.pio.h>

namespace
{
    void __not_in_flash_func(copyLine)(uint16_t *dst, const uint16_t *src, int w)
    {
        while (w--)
        {
            *dst++ = *src++;
        }
    }
}

void VideoSerializer::init(PIO pio, int sm, int pinData0, int pinClk, int pinSync)
{
    pio_ = pio;
    sm_ = sm;
    pinSync_ = pinSync;

    programOfs_ = pio_add_program(pio_, &simple_2bit_serialize_ddr_program);
    initProgramSimple2bitSerializeDDR(pio_, sm_, programOfs_, pinData0, pinClk);

    gpio_init(pinSync_);
    gpio_set_dir(pinSync_, GPIO_OUT);
}

void VideoSerializer::sendData(const uint32_t *data, size_t size) const
{
    auto dst = (volatile uint32_t *)&pio_->txf[sm_];
    auto tail = data + size;
    while (data != tail)
    {
        while (pio_sm_is_tx_fifo_full(pio_, sm_))
            ;
        *dst = *data++;
    }
}

void VideoSerializer::sendFrame(graphics::FrameBuffer &fb)
{
    // Start pulse
    gpio_put(pinSync_, 0);
    // sleep_us(1);

    int w = fb.getWidth();
    int h = fb.getHeight();

    // DBGPRINT("r %d\n", fb.getReadPlaneID());
    for (int y = 0; y < h; ++y)
    {
        int line = fb.moveCurrentPlaneLine(y);
        gpio_put(pinSync_, 1);
        auto *data = fb.getLineBuffer(line);
        sendData(reinterpret_cast<const uint32_t *>(data), w >> 1);
        fb.freeLine({line});
    }
    fb.flipReadPlane();
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
namespace
{
    struct DeserializerIRQHandler
    {
        int ch = -1;
        VideoDeserializerBase *deserializer{};
    };
    DeserializerIRQHandler deserializerIRQHandlers_[2];

    void __isr __not_in_flash_func(deserializerIRQHandlerEntry)()
    {
        hw_divider_state_t divState;
        hw_divider_save_state(&divState);
        for (auto &h : deserializerIRQHandlers_)
        {
            if (h.ch >= 0)
            {
                auto mask = 1u << h.ch;
                if (dma_hw->ints1 & mask)
                {
                    h.deserializer->_finishCurrentLine();
                    dma_hw->ints1 = mask;
                }
            }
        }
        hw_divider_restore_state(&divState);
    }

    uint32_t startTime_;
    uint32_t lineTime_[2][240];
    uint32_t allocTime_[240];

    void dumpTime()
    {
        uint32_t prev0 = startTime_;
        uint32_t prev1 = startTime_;
        for (int i = 0; i < 240; ++i)
        {
            auto t0 = lineTime_[0][i];
            auto t1 = lineTime_[1][i];
            printf("%d: %d %d : a %d\n", i, t0 ? (prev0 - t0) & 0xffffff : -1, t1 ? (prev1 - t1) & 0xffffff : -1, allocTime_[i]);
            prev0 = t0;
            prev1 = t1;
        }
    }

    struct DeserializerProgram
    {
        PIO pio_{};
        uint programOfs_{};

    public:
        void init(PIO pio)
        {
            pio_ = pio;
            programOfs_ = pio_add_program(pio, &simple_2bit_deserialize_ddr_program);
        }
    };

    DeserializerProgram deserializerProgram_;
}

void initializeDeserializerProgram(PIO pio)
{
    deserializerProgram_.init(pio);
}

void VideoDeserializerBase::init(int sm, int pin0, int dmaID)
{
    assert(deserializerProgram_.pio_);
    sm_ = sm;

    dmaID_ = dmaID;

    auto pio = deserializerProgram_.pio_;
    auto programOfs = deserializerProgram_.programOfs_;
    initProgramSimple2BitDeserializeDDR(pio, sm, programOfs, pin0);

    // DMA
    dmaCh_ = dma_claim_unused_channel(true);
    dmaCtrlCh_ = dma_claim_unused_channel(true);
    DBGPRINT("VideoIn: dmaCh=%d, dmaCtrlCh=%d\n", dmaCh_, dmaCtrlCh_);

    {
        dma_channel_config config = dma_channel_get_default_config(dmaCh_);
        channel_config_set_dreq(&config, pio_get_dreq(pio, sm, false /* tx */));
        channel_config_set_read_increment(&config, false);
        channel_config_set_write_increment(&config, true);
        channel_config_set_chain_to(&config, dmaCtrlCh_);
        channel_config_set_irq_quiet(&config, true); // dst が nullで割り込み
        dma_channel_configure(dmaCh_, &config, nullptr, &pio->rxf[sm], 0, false);
    }
    {
        dma_channel_config config = dma_channel_get_default_config(dmaCtrlCh_);
        channel_config_set_read_increment(&config, false);
        channel_config_set_write_increment(&config, false);
        dma_channel_configure(dmaCtrlCh_, &config,
                              &dma_hw->ch[dmaCh_].al2_write_addr_trig,
                              &nextTransferAddr_, 1, false);
    }

    deserializerIRQHandlers_[dmaID] = {dmaCtrlCh_, this};

    irq_set_exclusive_handler(DMA_IRQ_1, deserializerIRQHandlerEntry);
    irq_set_enabled(DMA_IRQ_1, true);

    dma_hw->ints1 = 1 << dmaCtrlCh_;
    dma_channel_set_irq1_enabled(dmaCtrlCh_, true);
}

void VideoDeserializerBase::startReceive(graphics::FrameBuffer *fb)
{
    fb_ = fb;
    currentY_ = 0;

    int w = fb_->getWidth();
    // int h = fb_->getHeight();
    int h = fb_->getHeight();
    auto lineSize = w >> 1; // 16bpp

    if (receiveBuffer_.size() != lineSize * 2)
    {
        receiveBuffer_.resize(lineSize * 2);
    }

    startFrame();

    dma_channel_set_trans_count(dmaCh_, lineSize, false);

    setupNextLine();
    dma_start_channel_mask(1u << dmaCtrlCh_);

    startTime_ = util::getSysTickCounter24();
    memset(lineTime_, 0, sizeof(lineTime_));

    auto totalSize = lineSize * h;
    assert(totalSize);
    pio_sm_put(deserializerProgram_.pio_, sm_, totalSize * 8 - 1);
}

void VideoDeserializerBase::setupNextLine()
{
    if (currentY_ < fb_->getHeight())
    {
        nextTransferAddr_ = getReceiveBuffer(dbid_);
    }
    else
    {
        nextTransferAddr_ = nullptr;
    }
    ++currentY_;
    dbid_ ^= 1;
}

void VideoDeserializerBase::_finishCurrentLine()
{
    auto *data = reinterpret_cast<uint16_t *>(getReceiveBuffer(dbid_));
    setupNextLine();
    if (currentY_ > 2)
    {
        lineTime_[dmaID_][currentY_ - 3] = util::getSysTickCounter24();
        processLine(data);

#if 0
        if (currentY_ == 3)
        {
            auto p = (uint32_t *)dst;
            for (int i = 0; i < 160; ++i)
            {
                if ((i & 7) == 0)
                {
                    printf("\n");
                }
                printf("%08x ", p[i]);
            }
            printf("\n");
        }
#endif
    }
}

void VideoDeserializerBase::waitTransfer()
{
    deserializerProgram_.pio_->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm_);

    while (!(deserializerProgram_.pio_->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm_))))
    {
        tight_loop_contents();
    }

    dma_channel_wait_for_finish_blocking(dmaCtrlCh_);

    endFrame();
}

//////////
//////////

void VideoDeserializer::startFrame()
{
    committed_ = 0;
}

void VideoDeserializer::processLine(uint16_t *data)
{
    auto *fb = getFrameBuffer();
    int dstLine = fb->allocateLine();
    auto *dst = fb->getLineBuffer(dstLine);

    //
    if (offset_ < 0)
    {
        copyLine(dst, data - offset_, fb->getWidth() + offset_);
        memset(dst + fb->getWidth() + offset_, 0, -offset_ * 2);
    }
    else
    {
        memset(dst, 0, offset_ * 2);
        copyLine(dst + offset_, data, fb->getWidth() - offset_);
    }
    fb->commitNextLine(dstLine);
    ++committed_;
}

void VideoDeserializer::endFrame()
{
    auto *fb = getFrameBuffer();

    // 表示側での消費タイミングによってバッファが確保できず、抜けが出ることがある
    if (committed_ < fb->getHeight())
    {
        while (committed_ < fb->getHeight())
        {
            int dstLine = fb->allocateLine();
            fb->commitNextLine(dstLine);
            ++committed_;
        }
        sleep_ms(1); // 適当に同期がズレて、いい感じに噛み合うタイミングが来るのを期待する
    }

    assert(committed_ == fb->getHeight());
}

//////////
//////////

void LineCompositor::commit(bool right, uint16_t *data)
{
    auto *fb = getFrameBuffer();
    const auto &li = lineInfo_[right ? 1 : 0];

    if (lineQueue_.empty() || frontRight_ == right)
    {
        auto tt = util::getSysTickCounter24();
        int line = fb->allocateLine();
        // allocTime_[committed_ + lineQueue_.size()] = (tt - util::getSysTickCounter24() & 0xffffff);
        allocTime_[committed_ + lineQueue_.size()] = fb->getFreeLineCount();

        auto *lineBuffer = fb->getLineBuffer(line);
        copyLine(lineBuffer + li.dstOfs_, data + li.srcOfs_, li.w_);

        if (lineQueue_.empty())
        {
            frontRight_ = right;
        }
        lineQueue_.push_back(line);
        assert(lineQueue_.size() <= fb->getHeight());
    }
    else
    {
        int line = lineQueue_.front();
        auto *lineBuffer = fb->getLineBuffer(line);
        copyLine(lineBuffer + li.dstOfs_, data + li.srcOfs_, li.w_);
        lineQueue_.erase(lineQueue_.begin());
        fb->commitNextLine(line);
        ++committed_;
    }
}

void LineCompositor::finish()
{
    auto *fb = getFrameBuffer();
    if (committed_ < fb->getHeight())
    {
#if 0
        static int ct = 200;
        if (--ct == 0)
        {
            int t = util::getSysTickCounter24();
            dumpTime();
            DBGPRINT("%d lines, %d\n", committed_, (startTime_ - t) & 0xffffff);
        }
#endif
        DBGPRINT("finish r%d, n%d, commit %d\n", frontRight_, lineQueue_.size(), committed_);
        for (auto line : lineQueue_)
        {
            auto *p = fb->getLineBuffer(line);
            memset(p, 0x55, 320 * 2);
            fb->commitNextLine(line);
            ++committed_;
        }
        lineQueue_.clear();
        while (committed_ < fb->getHeight())
        {
            int dstLine = fb->allocateLine();
            auto *p = fb->getLineBuffer(dstLine);
            memset(p, 0xaa, 320 * 2);
            fb->commitNextLine(dstLine);
            ++committed_;
        }

        while (fb->getFreeLineCount() < 240 / 4)
            tight_loop_contents();
    }

    assert(lineQueue_.empty());

    // DBGPRINT("%d\n", fb->getFreeLineCount());
    // while (fb->getFreeLineCount() < 240 / 4)
    //     tight_loop_contents();
}

//////////
//////////
void VideoDeserializer2::startFrame() {}

void VideoDeserializer2::processLine(uint16_t *line)
{
    lineCompositor_->commit(right_, line);
}

void VideoDeserializer2::endFrame() {}

void serializerDebug()
{
    dumpTime();
}