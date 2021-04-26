/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 *
 * Copyright (C) 2017-21 Corellium LLC
 * All rights reserved.
 *
 */

#define SMP_DEBUG 0

.section ".start", "ax"

.global smpentry
smpentry:
    b smpentry
    .asciz "PREBOOT"

    .align 7
rvbar:
    .long 0xDEADB070, 0xDEADB070, 0xDEADB070, 0xDEADB070
    .long 0xDEADB070, 0xDEADB070, 0xDEADB070, 0xDEADB070
    .long 0xDEADB070, 0xDEADB070, 0xDEADB070, 0xDEADB070
    .long 0xDEADB070, 0xDEADB070, 0xDEADB070, 0xDEADB070

    .align 8
.global entry
entry:
    b start

start:
    mov x24, x0

    mov x0, #0x3d2b0000 	// WDT address
    orr x0, x0, #0x200000000	// WDT address
    str wzr, [x0, #0x1c]

    mov x0, #0x800000000
    orr x0, x0, #0x80000000
    sub x0, x0, #16
    mov sp, x0			// initialize stack

    bl enable_all_clocks

    adr x0, smpentry
    add x0, x0, #0x20000
    mov x1, x24
    adr x2, smpentry
    adr x3, rvbar
    bl loader_main

    adr x0, smpentry		// memcpy
    mov x1, #0x800000000
    orr x1, x1, #0x80000000
    add x1, x1, #0x40000
    add x2, x0, #0xc0000
    ldr x2, [x2, #16]
    add x2, x2, #0xc0000
    add x2, x2, #15
    bic x2, x2, #15
1:  ldp x8, x9, [x0], #16
    stp x8, x9, [x1], #16
    sub x2, x2, #16
    cbnz x2, 1b
    isb

    adr x0, smpentry		// set up physical address
    adr x1, 1f
    sub x1, x1, x0
    mov x0, #0x800000000
    orr x0, x0, #0x80000000
    add x0, x0, #0x40000
    add x1, x1, x0
    br x1
1:  isb

    mov x0, #0x800000000 	// jump to kernel
    orr x0, x0, #0x80000000
    add x0, x0, #0x60000
    mov x1, #0
    mov x2, #0
    mov x3, #0
    add x18, x0, #0xa0000
    br x18

.align 4
.pool

.section ".text", "ax"

.global udelay
udelay:
    mov x1, #24
    mul x0, x0, x1
    mrs x1, cntvct_el0
    add x1, x1, x0
1:  mrs x0, cntvct_el0
    cmp x0, x1
    ble 1b
    ret

.global tmr_now
tmr_now:
    mrs x0, cntvct_el0
    ret
