/*
 * author : Shuichi TAKANO
 * since  : Mon Aug 29 2022 02:41:05
 */

#include <stdint.h>
#include <hardware/i2c.h>
#include "device_def.h"

namespace device
{
    class PCA9554
    {
    public:
        enum class Type
        {
            B = 0x20,
            C = 0x38,
        };

    private:
        i2c_inst_t *i2c_{};
        uint8_t addr_ = 0;

    public:
        bool init(i2c_inst_t *i2cInst, Type type, int addrSel = 0);

        void setPortDir(uint8_t inputPortBits) const;
        void output(uint8_t bits) const;
        int input() const;
    };

    void selectAudioInput(PCA9554 &pca9554, SignalInput input);
}
