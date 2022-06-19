#include "common.h.S"

/////
// resize YCbCr420 stream
// arg0: dst begin
// arg1: dst end
decl_func resizeYCbCr420
	push {r4, r5, r6, r7, lr}
    mov ip, r1
    ldr r1, =-4         // address mask
    ldr r2, =0x00ff00ff // 0v0u
  	ldr r3, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)

    // interpolator 0
    //  base2   = srcAddress + 1
    //  base0   = step << 16
    //  accum1  = 0
    //  mask    = 0xfffffffe
    //  shift   = 16
    //  accum0 += base0
    //  result2 = base2 + (accum0 >> shift) & mask + accum1
    // 
    // 00 Cr Y Cb みたいな 32bit ピクセル列ができる

.macro do_pixel wr0, wr1
    ldr \wr0, [r3, #POP2_OFFS + INTERP0]  // 1
    ldrb \wr1, [\wr0]           // 2
    lsls \wr1, #8               // 1
    ands \wr0, r1               // 1
    ldr \wr0, [\wr0]            // 2
    ands \wr0, r2               // 1
    orrs \wr0, \wr1             // 1
    // 9 cycle
.endm

1:
.rept 4
    do_pixel r4, r5             // 9
    do_pixel r5, r6             // 9
    stmia r0!, { r4, r5 }       // 3
.endr

    cmp r0, ip                  // 1
    bne 1b                      // 2
    
    // ((9+9+3) * 4 + 3)/8 = 10.875 cycle/pix

	pop {r4, r5, r6, r7, pc}


/////
// YCbCr to rgb565
// arg0: dst
// arg1: src
// arg2: nPix
decl_func convertYCbCr2RGB565
	push {r4, r5, r6, r7, lr}
    lsls r2, #1
    adds r2, r0
    mov ip, r2
   	ldr r2, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
    movs r7, #74

    // interpolator 0
    //  s0: 16 + 8 + 6 - 5 - 11 = 14
    //  m0: 0b1111100000000000 
    //  s1: 8 + 6 - 6 - 5 = 3
    //  m1: 0b0000011111100000
    //  accum0: R << 16
    //  accum1: G
    //  base2 : B >> (8 + 6 - 5)
    //  result2 = base2 + ((accum0 >> s0) & m0) + ((accum1 >> s1) & m1)
    //
    // interpolator 1
    //  clamp mode
    //   base0: min 0
    //   base1: max 255 * 64
    //   result0 = (accum0 >> s0) & m0
    //   result1 = (result0 >> s1) & m1 // for B
    // 
    // R = 1.164(Y-16)                 + 1.596(Cr-128)
    // G = 1.164(Y-16) - 0.391(Cb-128) - 0.813(Cr-128)
    // B = 1.164(Y-16) + 2.018(Cb-128)
    //
    // 1.164 * 64 =  74.496
    // 0.391 * 64 =  25.024
    // 2.018 * 64 = 129.152
    // 1.596 * 64 = 102.144
    // 0.813 * 64 =  52.032
    // 
    // 1%弱誤差が出るが1つ掛け算を減らせる
    // 1.164/1.009 * 64 = 73.83151635282458
    // 0.391/1.009 * 64 = 24.80079286422200
    // 2.018/1.009 * 64 = 128
    // 1.596/1.009 * 64 = 101.2329038652131
    // 0.813/1.009 * 64 = 51.56788899900892


1:
.set n_repeat, 2
.set ofs, 0
.rept n_repeat
    ldrb r3, [r1, #1+ofs*4]   // r3: Y      // 2
    subs r3, #16                            // 1
    muls r3, r7                             // 1

    ldrb r4, [r1, #0+ofs*4]   // r4: Cb     // 2
    subs r4, #128                           // 1

    // B
    @ movs r5, #129                           // 1
    @ muls r5, r4                             // 1
    lsls r5, r4, 7                          // 1
    adds r5, r3                             // 1
    str r5, [r2, #ACCUM0_OFFS + INTERP1]    // 1
    ldr r5, [r2, #PEEK0_OFFS + INTERP1]     // 1
    lsrs r5, #9                             // 1
    str r5, [r2, #BASE2_OFFS + INTERP0]     // 1

    //
    ldrb r5, [r1, #2+ofs*4]   // r5: Cr     // 2
    subs r5, #128                           // 1

    // R
    movs r6, #101                           // 1
    muls r6, r5                             // 1
    adds r6, r3                             // 1
    str r6, [r2, #ACCUM0_OFFS + INTERP1]    // 1
    ldr r6, [r2, #PEEK0_OFFS + INTERP1]     // 1
    lsls r6, #16                            // 1
    str r6, [r2, #ACCUM0_OFFS + INTERP0]    // 1

    // G
    movs r6, #25                            // 1
    muls r4, r6                             // 1
    movs r6, #52                            // 1
    muls r5, r6                             // 1
    subs r3, r4                             // 1
    subs r3, r5                             // 1
    str r3, [r2, #ACCUM0_OFFS + INTERP1]    // 1
    ldr r3, [r2, #PEEK0_OFFS + INTERP1]     // 1
    str r3, [r2, #ACCUM1_OFFS + INTERP0]    // 1

    ldr r3, [r2, #PEEK2_OFFS + INTERP0]     // 1
    strh r3, [r0, #ofs*2]                   // 2
.set ofs, ofs+1
.endr
    // 35

    adds r0, 2 * n_repeat                   // 1
    adds r1, 4 * n_repeat                   // 1

    cmp r0, ip                              // 1
    bne 1b                                  // 2

    // (35 * 2 + 5)/2 = 37.50 cycle/pix
    // (35 * 3 + 5)/3 = 36.66 cycle/pix
    // (35 * 4 + 5)/4 = 36.25 cycle/pix

    // 10.875+37.5 = 48.375
    // 250000000/(320*263*60) = 49.50887198986058
    // 250000000/(320*263*59.94) = 49.55843042028087

	pop {r4, r5, r6, r7, pc}

