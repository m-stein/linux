/* arch/arm/mach-imx/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "imx-uart.h"
#include <genode_tz_vmm_defs.h>

/*
 * FIXME: This is a copy of IMX_IO_P2V in hardware.h, and needs to
 * stay sync with that.  It's hard to maintain, and should be fixed
 * globally for multi-platform build to use a fixed virtual address
 * for low-level debug uart port across platforms.
 */
#define IMX_IO_P2V(x)	(						\
			(((x) & 0x80000000) >> 7) |			\
			(0xf4000000 +					\
			(((x) & 0x50000000) >> 6) +			\
			(((x) & 0x0b000000) >> 4) +			\
			(((x) & 0x000fffff))))

#define UART_VADDR	IMX_IO_P2V(UART_PADDR)

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =UART_PADDR	@ physical
		ldr	\rv, =UART_VADDR	@ virtual
		.endm

		.macro	senduart,rd,rx

#ifdef GENODE_TZ_VMM
		.arch_extension sec
		str r0, 1f
		str r1, 2f
		str r2, 3f
		b 4f
		1: .space 4
		2: .space 4
		3: .space 4
		4:
		mov r0, #SMC_SERIAL
		mov r1, #SMC_SERIAL_SEND
		mov r2, \rd
		smc 0
		ldr r0, 1b
		ldr r1, 2b
		ldr r2, 3b
#else
		str	\rd, [\rx, #0x40]	@ TXDATA
#endif /* GENODE_TZ_VMM */

		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
#ifndef GENODE_TZ_VMM
1002:		ldr	\rd, [\rx, #0x98]	@ SR2
		tst	\rd, #1 << 3		@ TXDC
		beq	1002b			@ wait until transmit done
#endif /* GENODE_TZ_VMM */
		.endm
