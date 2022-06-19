#include "common.h.S"

/////
// resize YCrCb420 stream

decl_func resizeYCrCb420
    

/////
// YCrCb420 to rgb565
// arg0: dst begin
// arg1: dst end
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
    // 0vyu みたいな 32bit ピクセル列ができる

.macro do_pixel wr0, wr1
    ldr \wr0, [r3, #POP2_OFFS]  // 1
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
    do_pixel r4, r5
    do_pixel r5, r6
    stmia r0!, { r4, r5 }
.endr

    cmp r0, ip
    bne 1b

	pop {r4, r5, r6, r7, pc}
