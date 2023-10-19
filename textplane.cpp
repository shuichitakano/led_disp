/*
 * author : Shuichi TAKANO
 * since  : Thu Mar 02 2023 23:32:27
 */

#include "textplane.h"

#include <algorithm>
#include <utility>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cassert>
#include <hardware/divider.h>
#include <hardware/interp.h>

namespace graphics
{
    namespace
    {
        constexpr uint8_t fontDataSrc_[] = {
#include "font.h"
        };

        constexpr int interleaveReverseBit(int v)
        {
            auto f = [](int v, int i)
            {
                return ((v >> i) & 1) << ((7 - i) * 2);
            };
            return f(v, 0) | f(v, 1) | f(v, 2) | f(v, 3) | f(v, 4) | f(v, 5) | f(v, 6) | f(v, 7);
        }

        template <std::size_t... I>
        constexpr std::array<uint16_t, 256> makeInterleaveReverseTable(std::index_sequence<I...>)
        {
            return {interleaveReverseBit(I)...};
        }

        constexpr std::array<uint16_t, 256> interleaveReverseTable = makeInterleaveReverseTable(std::make_index_sequence<256>{});

        uint16_t fontData_[9][128];

        void initFont()
        {
            for (int i = 0; i < 128; ++i)
            {
                for (int y = 0; y < 9; ++y)
                {
                    const int baseOfs = (i << 3) + y;
                    const int d = y < 1 ? 0 : fontDataSrc_[baseOfs - 1];
                    const int dm1 = y < 2 ? 0 : fontDataSrc_[baseOfs - 2];
                    const int dp1 = y >= 8 ? 0 : fontDataSrc_[baseOfs];

                    const int d_fg = d >> 1;
#if 0
                    const int d_shd = d | ((d_fg | dm1 | dp1) >> 1);
#else
                    const int d_shd_base = d | dm1 | dp1;
                    const int d_shd = d_shd_base | (d_shd_base >> 1) | (d_shd_base >> 2);
#endif

#if 0
                    fontData_[y][i] = d_fg | (d_shd << 8);
#else
                    fontData_[y][i] = (interleaveReverseTable[d_fg] << 1) | interleaveReverseTable[d_shd];

                    // 00: bg
                    // 01: shd
                    // 10: fg
                    // 11: fg
#endif
                }
            }
        }
    }

    TextPlane::TextPlane()
    {
        initFont();
        clear();
        flip();
        clear();
        for (int i = 0; i < 8; ++i)
        {
            setPalette888(i, i & 4 ? 255 : 0, i & 2 ? 255 : 0, i & 1 ? 255 : 0);
        }
    }

    void
    TextPlane::setPalette(size_t idx, uint16_t col)
    {
        assert(idx < N_COLORS);
        palette_[idx] = col | (col << 16);
    }

    void
    TextPlane::setPalette888(size_t idx, int r, int g, int b)
    {
        setPalette(idx, ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    }

    void
    TextPlane::setupComposite() const
    {
        // result[0] = accum[0] >> 2
        // result[2] = (accum[0] & 0b1100) + base1

        {
            auto c = interp_default_config();
            interp_config_set_shift(&c, 2);
            interp_set_config(interp0_hw, 0, &c);
        }
        {
            auto c = interp_default_config();
            interp_config_set_mask(&c, 2, 3);        // 0b1100
            interp_config_set_cross_input(&c, true); // accum[0] in
            // interp_config_set_cross_result(&c, true); // accum[0] in
            interp_set_config(interp0_hw, 1, &c);
        }

        interp0_hw->accum[0] = 0;
        // interp0_hw->accum[1] = 0;
        interp0_hw->base[0] = 0;
        interp0_hw->base[1] = 0;
    }

    void
    TextPlane::composite(uint16_t *line, int y) const
    {
        constexpr int fontH = 9;
        if (y >= HEIGHT * fontH)
        {
            return;
        }
        auto rdiv = hw_divider_divmod_u32(y, fontH);

        const auto &l = getReadPlane()[to_quotient_u32(rdiv)];
        if (!l.isEmpty())
        {
            int yofs = to_remainder_u32(rdiv);
            int n = l.end - l.begin;
            line += l.begin * 8;
            auto *src = l.text.data() + l.begin;
            const auto srcEnd = src + n;
            do
            {
                compositeChr(line, *src, yofs);
                ++src;
                line += 8;
            } while (src != srcEnd);
        }
    }

    namespace
    {
        inline void __not_in_flash_func(fillTransBlack)(uint16_t *dst)
        {
            auto proc2 = [](int i, auto d)
            {
                constexpr uint32_t trmask = 0b01111011111011110111101111101111;
                d[i] = (d[i] >> 1) & trmask; // 6
                // 3 cycle/pix
            };

            auto *d = reinterpret_cast<uint32_t *>(dst);
            proc2(0, d);
            proc2(1, d);
            proc2(2, d);
            proc2(3, d);
        }

        inline void __not_in_flash_func(compositeTransShadowed)(uint16_t *dst,
                                                                uint32_t col, uint32_t colShadow, int fd)
        {
            uint32_t table[4];
            table[1] = colShadow;
            table[2] = col;
            table[3] = col;

            interp0_hw->base[1] = reinterpret_cast<uint32_t>(table);
            interp0_hw->accum[0] = fd << 2;

            auto proc2 = [](int i, auto table, auto dst)
            {
                constexpr uint32_t trmask = 0b01111011111011110111101111101111;
                uint32_t d01 = *reinterpret_cast<const uint32_t *>(dst + i); // 2
                uint32_t t01 = (d01 >> 1) & trmask;                          // 2
                table[0] = t01;                                              // 2
                auto *p0 = reinterpret_cast<uint16_t *>(interp0_hw->pop[1]); // 1
                dst[i + 0] = p0[0];                                          // 3
                auto *p1 = reinterpret_cast<uint16_t *>(interp0_hw->pop[1]); // 1
                dst[i + 1] = p1[1];                                          // 3
                // 7 cycle/pix
            };

            proc2(0, table, dst);
            proc2(2, table, dst);
            proc2(4, table, dst);
            proc2(6, table, dst);
        }

        inline void __not_in_flash_func(compositeShadowed)(uint16_t *dst,
                                                           uint32_t col, uint32_t colShadow, int fd)
        {
            uint32_t table[4];
            table[1] = colShadow;
            table[2] = col;
            table[3] = col;

            interp0_hw->base[1] = reinterpret_cast<uint32_t>(table);
            interp0_hw->accum[0] = fd << 2;

            auto proc2 = [](int i, auto table, auto dst)
            {
                constexpr uint32_t trmask = 0b01111011111011110111101111101111;
                uint32_t d01 = *reinterpret_cast<const uint32_t *>(dst + i); // 2
                table[0] = d01;                                              // 2
                auto *p0 = reinterpret_cast<uint16_t *>(interp0_hw->pop[1]); // 1
                dst[i + 0] = p0[0];                                          // 3
                auto *p1 = reinterpret_cast<uint16_t *>(interp0_hw->pop[1]); // 1
                dst[i + 1] = p1[1];                                          // 3
                // 6 cycle/pix
            };

            proc2(0, table, dst);
            proc2(2, table, dst);
            proc2(4, table, dst);
            proc2(6, table, dst);

            // auto *tb = reinterpret_cast<uint16_t *>(interp0_hw->pop[1]); // 1
            // if (tb != table)                                             // 2 or 4
            // {
            //     dst[i] = *tb; // 3
            // }
            // // 5 or 6 cycle
        }
    }

    void
    TextPlane::compositeChr(uint16_t *dst, Character c, int yofs) const
    {
        if (c.chr == ' ')
        {
            if (c.transBlack)
            {
                fillTransBlack(dst);
            }
            return;
        }

        const uint32_t col = palette_[c.color];
        const uint32_t colShadow = palette_[c.shadowColor];

        int fd = fontData_[yofs][c.chr];

        if (c.enableShadow)
        {
            if (c.transBlack)
            {
                compositeTransShadowed(dst, col, colShadow, fd);
            }
            else
            {
                compositeShadowed(dst, col, colShadow, fd);
            }
        }
        else
        {
            // todo
            // 使用頻度が低いので最適化はいずれ
            fd &= 0xaaaa;
            if (c.transBlack)
            {
                compositeTransShadowed(dst, col, colShadow, fd);
            }
            else
            {
                compositeShadowed(dst, col, colShadow, fd);
            }
        }
    }

    //////

    void
    TextPlane::clear()
    {
        auto &pl = getWritePlane();
        for (auto &l : pl)
        {
            l.clear();
        }
    }

    void
    TextPlane::fillBlack(size_t x, size_t y, size_t w, size_t h)
    {
        auto &pl = getWritePlane();
        while (h && y < HEIGHT)
        {
            pl[y++].fillBlack(x, w);
            --h;
        }
    }

    void
    TextPlane::put(size_t x, size_t y, const char *str)
    {
        if (y >= HEIGHT)
        {
            return;
        }
        getWritePlane()[y].put(x, color_, shadowColor_, enableShadow_, transBlack_, str);
    }

    void
    TextPlane::printf(size_t x, size_t y, const char *fmt, ...)
    {
        va_list args;
        char tmp[WIDTH + 1];

        va_start(args, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, args);
        va_end(args);

        put(x, y, tmp);
    }

    ///////

    void TextPlane::Line::clear()
    {
        begin = WIDTH;
        end = 0;
    }

    void TextPlane::Line::put(size_t x, int col, int shdCol, bool shadow, bool trans,
                              const char *str)
    {
        if (x >= WIDTH)
        {
            return;
        }

        Character tmp;
        tmp.color = col;
        tmp.shadowColor = shdCol;
        tmp.enableShadow = shadow;
        tmp.transBlack = trans;

        auto prevBegin = begin;

        begin = std::min<int>(begin, x);

        auto p = text.data() + x;
        while (*str && x < WIDTH)
        {
            tmp.chr = *str++;
            *p++ = tmp;
            ++x;
        }

        tmp = {};
        tmp.chr = ' ';
        while (x < prevBegin)
        {
            *p++ = tmp;
            ++x;
        }

        end = std::max<int>(end, x);
    }

    void TextPlane::Line::fillBlack(size_t x, size_t n)
    {
        if (x >= WIDTH)
        {
            return;
        }

        Character tmp;
        tmp.packed = 0;
        tmp.chr = ' ';
        tmp.transBlack = true;

        begin = std::min<int>(begin, x);
        n = std::min(n, WIDTH - x);
        std::fill_n(text.data() + x, n, tmp);
        end = std::max<int>(end, x + n);
    }
}
