/*
 * author : Shuichi TAKANO
 * since  : Fri Mar 18 2022 04:20:17
 */

#include "image_proc.h"
#include <hardware/interp.h>
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <cassert>
#include "util.h"

extern "C"
{
    void convertRGB565toLEDSerial(
        const uint16_t *p0,
        const uint16_t *p1,
        const uint16_t *p2,
        const uint16_t *p3,
        uint32_t *dst, uint32_t *dstTail);

    void resizeYCbCr420(uint32_t *dst,
                        uint32_t *dstTail);

    void convertYCbCr2RGB565(uint16_t *dst, const uint32_t *src, size_t nPixels);
}

namespace graphics
{
    namespace
    {
        constexpr uint8_t fontData_[] = {
#include "font.h"
        };

        // uint32_t interleave0(uint16_t v)
        // {
        //     uint32_t r = 0;
        //     uint32_t d = 1;
        //     for (int i = 0; i < 16; ++i)
        //     {
        //         if (v & 1)
        //         {
        //             r |= d;
        //         }
        //         v >>= 1;
        //         d <<= 2;
        //     }
        //     return r;
        // }

        class Processor
        {
            // bit逆順に1byteずつ12bitつまったテーブル
            uint32_t table5_[32][4];
            uint32_t table6_[64][4];

            // uint32_t gammaTable5_[32];
            // uint32_t gammaTable6_[64]; // 1bit左シフトされている
            // uint32_t gammaTable8_[256];

        public:
            Processor()
            {
                //                makeGammaTable();
            }

            void makeGammaTable()
            {
#if 1
                auto compute = [](uint32_t dst[4], int i, float ir)
                {
                    float v = pow(static_cast<float>(i * ir), 2.2f);
                    int iv = static_cast<int>(v * 4095.99f);

                    auto *p = reinterpret_cast<uint8_t *>(dst);
                    for (int i = 0; i < 12; ++i)
                    {
                        p[i] = (iv & (1 << (11 - i))) ? 0x80 : 0;
                    }
                    // printf("%2d: %04x: %08x %08x %08x\n", i, iv, dst[2], dst[1], dst[0]);
                };

                for (int i = 0; i < 32; ++i)
                {
                    compute(table5_[i], i, 1.0f / 31);
                }
                for (int i = 0; i < 64; ++i)
                {
                    compute(table6_[i], i, 1.0f / 63);
                }
#else

                auto compute = [](int i, float ir)
                {
                    float v = pow(static_cast<float>(i * ir), 2.2f);
                    int iv = static_cast<int>(v * 4095.99f);
                    int iiv = interleave0(iv);
                    return iiv;
                };

                for (int i = 0; i < 32; ++i)
                {
                    gammaTable5_[i] = compute(i, 1.0f / 31);
                }
                for (int i = 0; i < 64; ++i)
                {
                    gammaTable6_[i] = compute(i, 1.0f / 63) << 1;
                }
                for (int i = 0; i < 256; ++i)
                {
                    gammaTable8_[i] = compute(i, 1.0f / 255);
                }
#endif
            }

#if 1
            void
            __not_in_flash_func(convertCH)(uint32_t *dst,
                                           const uint16_t *line0,
                                           const uint16_t *line1,
                                           const uint16_t *line2,
                                           const uint16_t *line3,
                                           int shift, bool _5bit)
            {
                const auto table = _5bit ? table5_ : table6_;

                // extract RGB CH
                {
                    // base shift: 16
                    constexpr int tableShift = 4; // 16 byte/entry
                    constexpr int baseShift = 16;
                    int rshift = shift + baseShift - tableShift;
                    int maskLSB = tableShift;
                    int maskMSB = tableShift + (_5bit ? 4 : 5);
                    auto c = interp_default_config();
                    interp_config_set_shift(&c, rshift);
                    interp_config_set_mask(&c, maskLSB, maskMSB);
                    interp_set_config(interp0_hw, 0, &c);

                    interp0_hw->base[0] = reinterpret_cast<uint32_t>(table);
                }

                // CH accum
                auto configAccum = [](auto interp, int lane)
                {
                    auto c = interp_default_config();
                    interp_config_set_shift(&c, 1);
                    interp_config_set_mask(&c, 0, 31);
                    interp_set_config(interp, lane, &c);
                };
                configAccum(interp0, 1);
                configAccum(interp1, 0);
                configAccum(interp1, 1);

                constexpr size_t size = 160 * 8 * sizeof(uint16_t) / sizeof(uint32_t);
                convertRGB565toLEDSerial(line0, line1, line2, line3,
                                         dst, dst + size);
            }

            void convertCH_notopt(uint32_t *dst,
                                  const uint16_t *line0,
                                  const uint16_t *line1,
                                  const uint16_t *line2,
                                  const uint16_t *line3,
                                  int shift, bool _5bit)
            {
                const auto table = _5bit ? table5_ : table6_;
                int mask = (_5bit ? (1 << 5) : (1 << 6)) - 1;

                constexpr int unitW = 160;
                for (int i = 0; i < 16; ++i)
                {
                    for (int j = 0; j < 10; ++j)
                    {
                        uint32_t tmp[3]{};
                        auto accum = [&](int v)
                        {
                            auto ch = (v >> shift) & mask;
                            const auto p = table[ch];
                            tmp[0] = (tmp[0] >> 1) | p[0];
                            tmp[1] = (tmp[1] >> 1) | p[1];
                            tmp[2] = (tmp[2] >> 1) | p[2];
                        };

                        int ofs = j * 16 + i;
                        accum(line0[ofs]);
                        accum(line1[ofs]);
                        accum(line2[ofs]);
                        accum(line3[ofs]);
                        accum(line0[ofs + unitW]);
                        accum(line1[ofs + unitW]);
                        accum(line2[ofs + unitW]);
                        accum(line3[ofs + unitW]);

                        dst[0] = 0;
                        dst[1] = tmp[0];
                        dst[2] = tmp[1];
                        dst[3] = tmp[2];
                        dst += 4;
                    }
                }
            }
#else
            void convertLine8(uint32_t *dst, const uint8_t *src, int ofs0, int ofs1,
                              int nColumnsPerModule, int nModules)
            {
                for (int j = 0; j < nColumnsPerModule; ++j)
                {
                    auto p = src;
                    for (int i = 0; i < nModules; ++i)
                    {
                        int v0 = p[ofs0];
                        int v1 = p[ofs1];
                        *dst++ = gammaTable8_[v0] | (gammaTable8_[v1] << 1);
                        p += nColumnsPerModule * 3;
                    }
                    src += 3;
                }
            }

            void convertLineRG(uint32_t *dst, const uint16_t *src,
                               int nColumnsPerModule, int nModules)
            {
                for (int j = 0; j < nColumnsPerModule; ++j)
                {
                    auto p = src;
                    for (int i = 0; i < nModules; ++i)
                    {
                        int v = *p;
                        int r = v >> 11;
                        int g = (v >> 5) & 63;
                        *dst++ = gammaTable5_[r] | gammaTable6_[g];
                        p += nColumnsPerModule;
                    }
                    ++src;
                }
            }

            void convertLineB(uint32_t *dst, const uint16_t *src, int ofs1,
                              int nColumnsPerModule, int nModules)
            {
                for (int j = 0; j < nColumnsPerModule; ++j)
                {
                    auto p = src;
                    for (int i = 0; i < nModules; ++i)
                    {
                        int b1 = p[0] & 31;
                        int b2 = p[ofs1] & 31;
                        *dst++ = gammaTable5_[b1] | (gammaTable5_[b2] << 1);
                        p += nColumnsPerModule;
                    }
                    ++src;
                }
            }
#endif
        };

        Processor __not_in_flash_func(processor_);
    }

    void initImageProcessor()
    {
        processor_.makeGammaTable();
    }

#if 1
    void
    __not_in_flash_func(convert4)(uint32_t *dstR,
                                  uint32_t *dstG,
                                  uint32_t *dstB,
                                  const uint16_t *line0,
                                  const uint16_t *line1,
                                  const uint16_t *line2,
                                  const uint16_t *line3)
    {
        // uint32_t tmp[160 * 4];
        // processor_.convertCH_notopt(tmp, line0, line1, line2, line3, 11, true);

        assert(dstR[0] == 0);
        dstR[640] = 0xdeadbeaf;
        processor_.convertCH(dstR, line0, line1, line2, line3, 11, true);
        assert(dstR[0] == 0);
        assert(dstR[640] == 0xdeadbeaf);

        // if (memcmp(tmp, dstR, sizeof(tmp)) != 0)
        // {
        //     printf("org: \n");
        //     util::dump(std::begin(tmp), std::end(tmp));

        //     printf("opt: \n");
        //     util::dump(dstR, dstR + 640);

        //     printf("guard: %08x\n", dstR[640]);
        //     assert(0);
        // }

        assert(dstG[0] == 0);
        dstG[640] = 0xdeadbeaf;
        processor_.convertCH(dstG, line0, line1, line2, line3, 5, false);
        assert(dstG[0] == 0);
        assert(dstG[640] == 0xdeadbeaf);

        assert(dstB[0] == 0);
        dstB[640] = 0xdeadbeaf;
        processor_.convertCH(dstB, line0, line1, line2, line3, 0, true);
        assert(dstB[0] == 0);
        assert(dstB[640] == 0xdeadbeaf);
    }
#else

    void convertLineBGR888(uint32_t *dstB1B2, uint32_t *dstR1G1, uint32_t *dstR2G2,
                           const uint8_t *image, int stride, int line,
                           int nColumnsPerModule, int nModules, int nScanLines)
    {
        auto src = image + stride * line;
        int ofs2 = stride * nScanLines;
        processor_.convertLine8(dstB1B2, src, 0, 0 + ofs2, nColumnsPerModule, nModules);
        processor_.convertLine8(dstR1G1, src, 2, 1, nColumnsPerModule, nModules);
        processor_.convertLine8(dstR2G2, src, 2 + ofs2, 1 + ofs2, nColumnsPerModule, nModules);
    }

    void convertLineBGR565(uint32_t *dstB1B2, uint32_t *dstR1G1, uint32_t *dstR2G2,
                           const uint16_t *line0, const uint16_t *line1,
                           int nColumnsPerModule, int nModules)
    {
        int ofs1 = line1 - line0; // 負のこともある
        processor_.convertLineB(dstB1B2, line0, ofs1, nColumnsPerModule, nModules);
        processor_.convertLineRG(dstR1G1, line0, nColumnsPerModule, nModules);
        processor_.convertLineRG(dstR2G2, line1, nColumnsPerModule, nModules);
    }
#endif

    void convertBGRB888toBGR565(uint16_t *dst, const uint8_t *src, size_t n)
    {
        while (n--)
        {
            int r = src[2];
            int g = src[1];
            int b = src[0];
            int v = (((r >> 3) & 31) << 11) | (((g >> 2) & 63) << 5) | (b >> 3);
            *dst = v;
            src += 3;
            ++dst;
        }
    }

    void __not_in_flash_func(compositeFont)(uint16_t *line, int x, int w, int yofs, const char *str)
    {
        if (x < 0)
            x = 0;
        auto p = line + x;
        auto end = p + w;
        while (char ch = *str++)
        {
            auto d = fontData_[ch * 8 + yofs];

            for (int i = 0; i < 8; ++i)
            {
                if (p < end)
                {
                    *p++ = (d & (0x80 >> i)) ? 0xffff : 0;
                }
            }
        }
    }

    //
    void __not_in_flash_func(resizeYCbCr420)(uint32_t *dst, size_t nDstPixels,
                                             const uint32_t *src, size_t nSrcPixels,
                                             size_t srcOfs)
    {
        {
            auto c0 = interp_default_config();
            interp_config_set_add_raw(&c0, true);
            interp_config_set_shift(&c0, 16);
            interp_config_set_mask(&c0, 1, 31);

            auto c1 = interp_default_config();

            interp_set_config(interp0_hw, 0, &c0);
            interp_set_config(interp0_hw, 1, &c1);

            // y の立場では 2 byte/pix で 16bit 固定少数
            int step = (nSrcPixels << 17) / nDstPixels;

            interp0_hw->accum[0] = (srcOfs << 17) + (step >> 1);
            interp0_hw->accum[1] = 0;
            interp0_hw->base[0] = step;
            interp0_hw->base[1] = 0;
            interp0_hw->base[2] = reinterpret_cast<uintptr_t>(src) + 1 /* y ofs */;
        }
        ::resizeYCbCr420(dst, dst + nDstPixels);
    }

    void __not_in_flash_func(convertYCbCr2RGB565)(uint16_t *dst,
                                                  const uint32_t *src, size_t nPixels)
    {
        {
            auto c0 = interp_default_config();
            interp_config_set_shift(&c0, 14);
            interp_config_set_mask(&c0, 11, 15);

            auto c1 = interp_default_config();
            interp_config_set_shift(&c1, 3);
            interp_config_set_mask(&c1, 5, 10);

            interp_set_config(interp0_hw, 0, &c0);
            interp_set_config(interp0_hw, 1, &c1);
        }
        {
            auto c0 = interp_default_config();
            interp_config_set_clamp(&c0, true);
            interp_config_set_signed(&c0, true);

            // auto c1 = interp_default_config();
            //  interp_config_set_cross_input(&c1, true);
            //  interp_config_set_shift(&c1, 6);
            //  interp_config_set_mask(&c1, 0, 4);

            interp_set_config(interp1_hw, 0, &c0);
            // interp_set_config(interp1_hw, 1, &c1);

            interp1_hw->base[0] = 0;
            interp1_hw->base[1] = 255 << 6;

            // c1側は使えないようだ
        }

        ::convertYCbCr2RGB565(dst, src, nPixels);
    }
}
