/*
 * author : Shuichi TAKANO
 * since  : Sat Sep 14 2024 05:31:40
 */

#include "system_parameters.h"

#define PARAM_ID(m1, m2) m1##_##m2
#define MEMBER(m1, m2) m1.m2
#define EVAL(f, m1, m2) f(PARAM_ID(m1, m2), MEMBER(m1, m2))

#define EVAL_SYSPARAMS(f)                      \
    EVAL(f, ledDisplay, imageSize);            \
    EVAL(f, ledDisplay, displayOffset);        \
    EVAL(f, videoCapture, samplingOffset);     \
    EVAL(f, videoCapture, samplingWidth);      \
    EVAL(f, videoCapture, samplingOffsetFrac); \
    EVAL(f, videoCapture, samplingWidthFrac);  \
    EVAL(f, videoStatus, flags);               \
    EVAL(f, root, signalInput)
