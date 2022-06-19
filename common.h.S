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
