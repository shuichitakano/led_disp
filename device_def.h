/*
 * author : Shuichi TAKANO
 * since  : Thu Sep 15 2022 02:49:34
 */
#pragma once

namespace device
{
    enum class SignalInput : uint8_t
    {
        NONE,
        COMPOSITE,
        S_VIDEO,
        COMPONENT,
        RGB21,
    };

    enum class Button
    {
        A, // up
        B, // right
        C, // left
        D, // down
        CENTER,
        UP = A,
        RIGHT = B,
        LEFT = C,
        DOWN = D,
    };

    inline bool isButtonPushed(int cur, Button b)
    {
        auto mask = 1 << static_cast<int>(b);
        return (cur & mask) == 0;
    }

    inline bool isButtonEdge(int cur, int prev, Button b)
    {
        auto mask = 1 << static_cast<int>(b);
        return ((cur ^ prev) & ~cur) & mask;
    }

    inline bool isAnyButtonPushed(int cur)
    {
        return (cur & 0b11111) != 0b11111;
    }
}
