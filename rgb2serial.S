#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

#define ACCUM0_OFFS     (SIO_INTERP0_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_OFFS     (SIO_INTERP0_ACCUM1_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)
#define ACCUM1_ADD_OFFS (SIO_INTERP0_ACCUM1_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define BASE0_OFFS      (SIO_INTERP0_BASE0_OFFSET      - SIO_INTERP0_ACCUM0_OFFSET)
#define BASE1_OFFS      (SIO_INTERP0_BASE1_OFFSET      - SIO_INTERP0_ACCUM0_OFFSET)
#define BASE2_OFFS      (SIO_INTERP0_BASE2_OFFSET      - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK0_OFFS      (SIO_INTERP0_PEEK_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK1_OFFS      (SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)
#define PEEK2_OFFS      (SIO_INTERP0_PEEK_FULL_OFFSET  - SIO_INTERP0_ACCUM0_OFFSET)
#define POP0_OFFS       (SIO_INTERP0_POP_LANE0_OFFSET  - SIO_INTERP0_ACCUM0_OFFSET)
#define POP1_OFFS       (SIO_INTERP0_POP_LANE1_OFFSET  - SIO_INTERP0_ACCUM0_OFFSET)
#define POP2_OFFS       (SIO_INTERP0_POP_FULL_OFFSET   - SIO_INTERP0_ACCUM0_OFFSET)
#define INTERP0         0
#define INTERP1         (SIO_INTERP1_ACCUM0_OFFSET     - SIO_INTERP0_ACCUM0_OFFSET)

.syntax unified
.cpu cortex-m0plus
.thumb

.macro decl_func_x name
//.section .scratch_x.\name, "ax"
.section .time_critical.\name, "ax"
.global \name
.type \name,%function
.thumb_func
\name:
.endm

.macro decl_func_y name
.section .scratch_y.\name, "ax"
.global \name
.type \name,%function
.thumb_func
\name:
.endm

#define decl_func decl_func_x

.macro do_pixel ch_peek_ofs
    str r6, [r5, #ACCUM0_OFFS + INTERP0]    // 1
    ldr r6, [r5, #\ch_peek_ofs + INTERP0]   // 1
    ldr r7, [r6, #0]                        // 2
    str r7, [r5, #BASE1_OFFS + INTERP0]     // 1
    ldr r7, [r6, #4]                        // 2
    str r7, [r5, #BASE0_OFFS + INTERP1]     // 1
    ldr r7, [r6, #8]                        // 2
    str r7, [r5, #BASE1_OFFS + INTERP1]     // 1
                                            // 11 cycles
.endm

.macro load_and_shift ofs, odd
    .if \odd
    ldr r6, [r0, \ofs]
    .else
    ldrh r6, [r0, \ofs]
    lsls r6, #16
    .endif
.endm

/////
// rgb565 to LED serial data (2x2)
// 
// arg 0: Source Line 0
// arg 1: Source Line 1
// arg 2: Source Line 2
// arg 3: Source Line 3
// arg 4: Output buffer begin
// arg 5: Output buffer end
decl_func convertRGB565toLEDSerial
    b 0f
1:
.set odd, 0
.rept 10
    // * Interpolator 0
    //   - RGB ch extract
    //     result0 = base0 + ((accum0 >> a) & b)
    //   - serial data (bit11:8 x 8) accum
    //     result1 = base1 + (result1 >> 1)
    // * Interpolator 1
    //   - serial data (bit7:4 x 8) accum
    //     result0 = base0 + (result0 >> 1)
    //   - serial data (bit3:0 x 8) accum
    //     result1 = base1 + (result1 >> 1)
    // 
    // pop でステート進むので、無意味な read を入れる必要がある
    // Interpolator 0 の無駄 read を省くために ch extract の read を POP にし、
    // 前のステップの POP を省く。
    // lut は 3 word で、[11:8], [7:4], [3:0]のbitがreverseして0x80808080倍されている。
    // 1011 -> 10000000_10000000_00000000_10000000

    mov r5, ip
    movs r6, #0
    str r6, [r5, #ACCUM1_OFFS + INTERP0]
    str r6, [r5, #ACCUM0_OFFS + INTERP1]
    str r6, [r5, #ACCUM1_OFFS + INTERP1]

    // 1 pixel あたり 15 cycle を単純に必要ピクセル数倍した時の 1sec あたりの cycle 数
    // 15 * 8 * 10 * 16 * 60 * 60 * 3        = 207 360 000 cycles
    // (14 * 2 + 15) * 8 * 10 * 16 * 60 * 60 = 198 144 000 cycles

    load_and_shift #0, odd              // 3
    do_pixel PEEK0_OFFS                 // 11
    ldr r7, [r5, #POP0_OFFS + INTERP1]  // 1
                                        // 15 cycles
    load_and_shift r1, odd
    do_pixel POP0_OFFS
    ldr r7, [r5, #POP0_OFFS + INTERP1]

    load_and_shift r2, odd
    do_pixel POP0_OFFS
    ldr r7, [r5, #POP0_OFFS + INTERP1]

    load_and_shift r3, odd
    do_pixel POP0_OFFS
    ldr r7, [r5, #POP0_OFFS + INTERP1]

    adds r0, #160
    adds r0, #160

    load_and_shift #0, odd
    do_pixel POP0_OFFS
    ldr r7, [r5, #POP0_OFFS + INTERP1]

    load_and_shift r1, odd
    do_pixel POP0_OFFS
    ldr r7, [r5, #POP0_OFFS + INTERP1]

    load_and_shift r2, odd
    do_pixel POP0_OFFS
    ldr r7, [r5, #POP0_OFFS + INTERP1]

    load_and_shift r3, odd
    do_pixel POP0_OFFS

    adds r4, #4
    ldr r6, [r5, #PEEK0_OFFS + INTERP1]
    ldr r7, [r5, #PEEK1_OFFS + INTERP1]
    ldr r5, [r5, #PEEK1_OFFS + INTERP0]
    stmia r4!, { r5, r6, r7 }

    subs r0, #(160-16)
    subs r0, #(160-16)

    @ ; 16 + 15 * 8 = 136 cycle/8 pix
    @ ; 136 / 8 * 320*240*60*3 = 235 008 000

    @ ; 16 + 14 * 8 = 128 cycle/8 pix
    @ ; (128+136)*160*240*60*3 = 1824768000
    @ ; (128+136)/2=132
    @ ; (132+128*2)*320*240*60 = 1787904000
    @ ; 4k + 2k = 6KB

//.set odd, 1 - odd
.endr

    subs r0, #(16*10-1)
    subs r0, #(16*10-1)
2:
//	cmp r4, ip
//	bne 1b
    ldr r6, [sp, #24]
    cmp r4, r6
    beq 3f
    b 1b
3:
	pop {r4, r5, r6, r7, pc}
    // ret

0:
    // entry
	push {r4, r5, r6, r7, lr}
    ldr r4, [sp, #20] // dst
    subs r1, r0
    subs r2, r0
    subs r3, r0
	ldr r5, =(SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET)
	mov ip, r5
    b 2b
