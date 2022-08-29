/*
 * author : Shuichi TAKANO
 * since  : Mon Aug 29 2022 02:41:05
 */

#include <stdint.h>
#include <hardware/i2c.h>

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
        void init(i2c_inst_t *i2cInst, Type type, int addrSel = 0)
        {
            i2c_ = i2cInst;
            addr_ = static_cast<int>(type) | addrSel;
        }

        void setPortDir(uint8_t inputPortBits) const;
        void output(uint8_t bits) const;
        int input() const;
    };
}
