/*
 * author : Shuichi TAKANO
 * since  : Sun Aug 28 2022 06:32:30
 */
#pragma once

#include <stdint.h>
#include <hardware/i2c.h>

namespace device
{

    class ADV7181
    {
    public:
        enum class Input
        {
            NONE,
            COMPOSITE,
            S_VIDEO,
            COMPONENT,
            RGB21,
        };

    public:
        void init(i2c_inst_t *i2cInst)
        {
            i2c_ = i2cInst;
        }

        void reset() const;
        void selectInput(Input input);

    private:
        i2c_inst_t *i2c_{};
        Input input_{Input::NONE};
    };
}
