/*
 * author : Shuichi TAKANO
 * since  : Tue Feb 28 2023 02:28:39
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <array>
#include <pico/stdlib.h>

namespace graphics
{

    class TextPlane
    {
    public:
        inline static constexpr size_t COLOR_BITS = 3;
        inline static constexpr size_t N_COLORS = 1 << COLOR_BITS;
        inline static constexpr size_t WIDTH = 40;
        inline static constexpr size_t HEIGHT = 26;

        union Character
        {
            struct
            {
                char chr;
                uint8_t color : COLOR_BITS;
                uint8_t shadowColor : COLOR_BITS;
                uint8_t enableShadow : 1;
                uint8_t transBlack : 1;
            };

            uint16_t packed;
        };

        struct Line
        {
            uint8_t begin, end;
            std::array<Character, WIDTH> text;

            void clear();
            void put(size_t x, int col, int shdCol, bool shadow, bool trans, const char *str);
            void fillBlack(size_t x, size_t n);

            bool isEmpty() const { return end < begin; }
        };
        using Plane = std::array<Line, HEIGHT>;

    public:
        TextPlane();

        const Plane &getReadPlane() const { return planes_[writePlaneID_ ^ 1]; }
        Plane &getWritePlane() { return planes_[writePlaneID_]; }
        void flip() { writePlaneID_ ^= 1; }

        void setColor(int c) { color_ = c; }
        void setShadowColor(int c) { shadowColor_ = c; }
        void enableShadow(bool f = true) { enableShadow_ = f; }
        void enableTransBlack(bool f = true) { transBlack_ = f; }

        void clear();
        void fillBlack(size_t x, size_t y, size_t w, size_t h);
        void put(size_t x, size_t y, const char *str);
        void printf(size_t x, size_t y, const char *fmt, ...);

        void setPalette(size_t idx, uint16_t col);
        void setPalette888(size_t idx, int r, int g, int b);
        void __not_in_flash_func(setupComposite)() const;
        void __not_in_flash_func(composite)(uint16_t *line, int y) const;
        void __not_in_flash_func(compositeChr)(uint16_t *dst, Character c, int yofs) const;

    private:
        Plane planes_[2];
        uint32_t palette_[8];

        uint8_t writePlaneID_ = 0;
        uint8_t color_ = 0;
        uint8_t shadowColor_ = 0;
        bool enableShadow_ = false;
        bool transBlack_ = false;
    };
}
