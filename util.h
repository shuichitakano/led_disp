#pragma once
/*
 * author : Shuichi TAKANO
 * since  : Wed May 04 2022 23:35:01
 */

#include <cstdio>
#include <vector>
#include <hardware/structs/systick.h>

#ifdef NDEBUG
// #if 0
#define DBGPRINT(...) \
    do                \
    {                 \
    } while (0)
#else
#define DBGPRINT(...) printf(__VA_ARGS__)
#endif

namespace util
{
    template <class T>
    void dump(T p, T end, const char *fmt = "%08x ")
    {
        int ct = 0;
        for (; p != end; ++p)
        {
            printf(fmt, *p);
            if ((ct++ & 7) == 7)
                printf("\n");
        }
        printf("\n");
    }

    template <class T, class F>
    void dumpF(T p, T end, const F &func)
    {
        int ct = 0;
        for (; p != end; ++p)
        {
            func(*p);
            if ((ct++ & 7) == 7)
                printf("\n");
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
