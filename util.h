#pragma once
/*
 * author : Shuichi TAKANO
 * since  : Wed May 04 2022 23:35:01
 */

#include <cstdio>
#include <vector>
#include <hardware/structs/systick.h>

namespace util
{
    template <class T>
    void dump(T p, T end, const char *fmt = "%08x ")
    {
        for (; p != end; ++p)
        {
            printf(fmt, *p);
        }
        printf("\n");
    }

    inline void initSysTick()
    {
        systick_hw->csr = 0x5;
        systick_hw->rvr = 0x00FFFFFF;
    }

    // tick counterを取得
    // カウンタは減っていくのに注意
    inline uint32_t getSysTickCounter24()
    {
        return systick_hw->cvr;
    }
}
