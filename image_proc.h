/*
 * author : Shuichi TAKANO
 * since  : Fri Mar 18 2022 04:28:18
 */
#pragma once

#include <stdint.h>
#include <vector>

namespace graphics
{
    void initImageProcessor();
    // void convertLineBGR888(uint32_t *dstB1B2, uint32_t *dstR1G1, uint32_t *dstR2G2,
    //                        const uint8_t *image, int stride, int line,
    //                        int nColumnsPerModule, int nModules, int nScanLines);

    // void convertLineBGR565(uint32_t *dstB1B2, uint32_t *dstR1G1, uint32_t *dstR2G2,
    //                        const uint16_t *line0, const uint16_t *line1,
    //                        int nColumnsPerModule, int nModules);

    void convert4(uint32_t *dstR, uint32_t *dstG, uint32_t *dstB,
                  const uint16_t *line0,
                  const uint16_t *line1,
                  const uint16_t *line2,
                  const uint16_t *line3);

    void convertBGRB888toBGR565(uint16_t *dst, const uint8_t *src, size_t n);

    void compositeFont(uint16_t *line, int x, int w, int yofs, const char *str);

    //

    void resizeYCbCr420(uint32_t *dst, size_t nDstPixels,
                        const uint32_t *src, size_t nSrcPixels, size_t srcOfs = 0);

    void convertYCbCr2RGB565(uint16_t *dst, const uint32_t *src, size_t nPixels);

    //
    void setupResizeYCbCr420Config(size_t nDstPixels,
                                   size_t nSrcPixels,
                                   size_t srcOfs);
    void resizeYCbCr420PreConfig(uint32_t *dst, size_t nDstPixels,
                                 const uint32_t *src);
}
