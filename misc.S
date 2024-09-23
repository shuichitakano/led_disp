#include "common.h.S"

//////
// resize simple 32bit stream
// arg0: dst begin
// arg1: dst end
decl_func resizeSimple32Loop
    push {r4, r5, lr}
    mov ip, r1
    ldr r1, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)

.macro do_pixel_resize dst
    ldr \dst, [r1, #POP2_OFFS + INTERP0]  // 1
    ldr \dst, [\dst, #0]           // 2
.endm

1:
    do_pixel_resize r2             // 3
    do_pixel_resize r3             // 3
    do_pixel_resize r4             // 3
    do_pixel_resize r5             // 3
    stmia r0!, { r2, r3, r4, r5 } // 5

    cmp r0, ip                  // 1
    bne 1b                      // 2

    pop {r4, r5, pc}

    // (3*4+5+1+2)/4=5 cycle/pix
    // 5*320=1600 cycle/line

//////
// fix bit order for V4 hw
// arg0: dst begin
// arg1: dst end
decl_func fixBitOrderV4Opt
    push {r4, r5, r6, r7, lr}
    mov ip, r1
    ldr r1, =0x0ff0ff
    ldr r2, =0xf00f00
    
.macro do_pixel_fixorder dst, tmp
    lsrs \tmp, \dst, #4 // 1
    ands \tmp, r1       // 1
    lsls \dst, \dst, #8 // 1
    ands \dst, r2       // 1
    orrs \dst, \tmp     // 1
.endm

1:
    ldmia r0!, { r3, r4, r5, r6 }   // 5
    adds r0, -16                    // 1

    do_pixel_fixorder r3, r7        // 5
    do_pixel_fixorder r4, r7
    do_pixel_fixorder r5, r7
    do_pixel_fixorder r6, r7

    stmia r0!, { r3, r4, r5, r6 }   // 5

    cmp r0, ip                      // 1
    bne 1b                          // 2

    pop {r4, r5, r6, r7, pc}

// (5+1+5*4+5+1+2)/4=8.5 cycle/pix
// 8.5*320=2720 cycle/line


//////
// xrgb8888 to rgb565
// arg0: dst
// arg1: src
// arg2: n
decl_func convertXRGB8888toRGB565Opt
    push {r4, r5, r6, r7, lr}
    lsls r2, #1
    adds r2, r0
    mov ip, r2
    ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)

.macro do_pixel_xrgb8888to565 dst, tmp
    lsls \tmp, \dst, #24
    lsrs \tmp, \tmp, #27
    str \dst, [r2, #ACCUM0_OFFS + INTERP0]
    ldr \dst, [r2, #PEEK2_OFFS + INTERP0]
    orrs \dst, \tmp
.endm

1:
    ldmia r1!, { r3, r4, r5, r6 }   // 5
    do_pixel_xrgb8888to565 r3, r7   // 5
    do_pixel_xrgb8888to565 r4, r7
    do_pixel_xrgb8888to565 r5, r7
    do_pixel_xrgb8888to565 r6, r7
    strh r3, [r0, #0]               // 2
    strh r4, [r0, #2]
    strh r5, [r0, #4]
    strh r6, [r0, #6]
    adds r0, #8                     // 1

    cmp r0, ip                      // 1
    bne 1b                          // 2

    pop {r4, r5, r6, r7, pc}

// (5+5*4+2*4+1+1+2)/4=9,25 cycle/pix
// 9.25*320=2960 cycle/line