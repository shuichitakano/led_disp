#pragma once
/*
 * author : Shuichi TAKANO
 * since  : Wed May 04 2022 23:35:01
 */

#include <cstdio>
#include <vector>

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

}
