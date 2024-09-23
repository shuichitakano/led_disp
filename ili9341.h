/*
 * author : Shuichi TAKANO
 * since  : Sat Aug 10 2024 17:27:22
 */
#pragma once

#include <cstdint>
#include "framebuffer.h"

namespace device
{
    void initLCD();
    void testLCD();

    void drawLCD(graphics::FrameBuffer &fb);
}
