/*
 * author : Shuichi TAKANO
 * since  : Thu Mar 02 2023 23:32:27
 */

#include "textplane.h"

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cassert>
#include <hardware/divider.h>

namespace graphics
{
    namespace
    {
        constexpr uint8_t fontDataSrc_[] = {
#include "font.h"
        };

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
                    fontData_[y][i] = d_fg | (d_shd << 8);
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
        palette_[idx] = col;
    }

    void
    TextPlane::setPalette888(size_t idx, int r, int g, int b)
    {
        setPalette(idx, ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
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

    void
    TextPlane::compositeChr(uint16_t *dst, Character c, int yofs) const
    {
        constexpr int trmask = 0b0111101111101111;

        if (c.chr == ' ')
        {
            if (c.transBlack)
            {
                auto dstEnd = dst + 8;
                do
                {
                    *dst = (*dst >> 1) & trmask;
                } while (++dst != dstEnd);
            }
            return;
        }

        const int col = palette_[c.color];
        const int colShadow = palette_[c.shadowColor];

        const int fd = fontData_[yofs][c.chr];
        const int d_fg = fd;
        const int d_shd = fd >> 8;

        if (c.enableShadow)
        {
            if (c.transBlack)
            {
                int mask = 0x80;
                for (; mask; mask >>= 1, ++dst)
                {
                    if (d_fg & mask)
                    {
                        *dst = col;
                    }
                    else if (d_shd & mask)
                    {
                        *dst = colShadow;
                    }
                    else
                    {
                        *dst = (*dst >> 1) & trmask;
                    }
                }
            }
            else
            {
                int mask = 0x80;
                for (; mask; mask >>= 1, ++dst)
                {
                    if (d_fg & mask)
                    {
                        *dst = col;
                    }
                    else if (d_shd & mask)
                    {
                        *dst = colShadow;
                    }
                }
            }
        }
        else
        {
            if (c.transBlack)
            {
                int mask = 0x80;
                for (; mask; mask >>= 1, ++dst)
                {
                    if (d_fg & mask)
                    {
                        *dst = col;
                    }
                    else
                    {
                        *dst = (*dst >> 1) & trmask;
                    }
                }
            }
            else
            {
                int mask = 0x80;
                for (; mask; mask >>= 1, ++dst)
                {
                    if (d_fg & mask)
                    {
                        *dst = col;
                    }
                }
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

        begin = std::min<int>(begin, x);

        auto p = text.data() + x;
        while (*str && x < WIDTH)
        {
            tmp.chr = *str++;
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
