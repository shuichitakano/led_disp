/*
 * author : Shuichi TAKANO
 * since  : Mon Aug 29 2022 02:42:51
 */

#include "pca9554.h"

#include "util.h"
#include <cassert>

namespace device
{
    namespace
    {
        enum class Command
        {
            INPUT_PORT,
            OUTPUT_PORT,
            POLARITY_INVERSION,
            CONFIGURATION,
        };
    }

    bool
    PCA9554::init(i2c_inst_t *i2cInst, Type type, int addrSel)
    {
        i2c_ = i2cInst;
        addr_ = static_cast<int>(type) | addrSel;

        uint8_t rxdata{};
        auto r = i2c_read_blocking(i2c_, addr_, &rxdata, 1, false);

        DBGPRINT("PCA9554 init[%02x]: %d\n", addr_, r);
        return r >= 0;
    }

    void
    PCA9554::setPortDir(uint8_t inputPortBits) const
    {
        assert(i2c_);
        uint8_t data[] = {static_cast<uint8_t>(Command::CONFIGURATION), inputPortBits};
        i2c_write_blocking(i2c_, addr_, data, 2, false);
    }

    void
    PCA9554::output(uint8_t bits) const
    {
        assert(i2c_);
        uint8_t data[] = {static_cast<uint8_t>(Command::OUTPUT_PORT), bits};
        i2c_write_blocking(i2c_, addr_, data, 2, false);
    }

    int
    PCA9554::input() const
    {
        assert(i2c_);
        auto cmd = static_cast<uint8_t>(Command::INPUT_PORT);
        i2c_write_blocking(i2c_, addr_, &cmd, 1, true);

        uint8_t data = 0;
        i2c_read_blocking(i2c_, addr_, &data, 1, false);
        return data;
    }

    //
    void selectAudioInput(PCA9554 &pca9554, SignalInput input)
    {
        int v = 0;
        switch (input)
        {
        case SignalInput::COMPOSITE:
            v = 0;
            break;

        case SignalInput::S_VIDEO:
            v = 1;
            break;

        case SignalInput::COMPONENT:
            v = 2;
            break;

        case SignalInput::RGB21:
            v = 3;
            break;
        };

        pca9554.output(v << 6);
    }

}
