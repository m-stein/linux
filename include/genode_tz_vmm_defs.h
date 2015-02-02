/*
 * \brief  Utilities for running in a Trustzone VM monitored by Genode
 * \author Martin Stein <martin.stein@genode-labs.com>
 * \date   2015-11-03
 */

/*
 * Copyright (C) 2015 Genode Labs GmbH
 *
 * This file is part of the Genode OS framework, which is distributed
 * under the terms of the GNU General Public License version 2.
 */

#ifndef _GENODE_TZ_VMM_DEFS_H_
#define _GENODE_TZ_VMM_DEFS_H_

/* whether running in a Trustzone VM monitored by Genode */
#define  GENODE_TZ_VMM
#ifdef   GENODE_TZ_VMM

/* shortcuts for arguments of SMC functions */
#define SMC_1_1_ARGS               long arg_0
#define SMC_2_1_ARGS SMC_1_1_ARGS, long arg_1
#define SMC_3_1_ARGS SMC_2_1_ARGS, long arg_2
#define SMC_4_1_ARGS SMC_3_1_ARGS, long arg_3
#define SMC_5_1_ARGS SMC_4_1_ARGS, long arg_4
#define SMC_5_2_ARGS SMC_5_1_ARGS, long * ret_1
#define SMC_6_1_ARGS SMC_5_1_ARGS, long arg_5
#define SMC_7_1_ARGS SMC_6_1_ARGS, long arg_6
#define SMC_8_1_ARGS SMC_7_1_ARGS, long arg_7
#define SMC_9_1_ARGS SMC_8_1_ARGS, long arg_8

/* shortcuts for return statements in SMC functions */
#define SMC_X_1_RETURN return   arg_0_reg;
#define SMC_X_2_RETURN *ret_1 = arg_1_reg; SMC_X_1_RETURN

/* shortcuts for argument to register mappings in SMC functions */
#define SMC_1_X_REGS              register long arg_0_reg asm("r0") = arg_0;
#define SMC_2_X_REGS SMC_1_X_REGS register long arg_1_reg asm("r1") = arg_1;
#define SMC_3_X_REGS SMC_2_X_REGS register long arg_2_reg asm("r2") = arg_2;
#define SMC_4_X_REGS SMC_3_X_REGS register long arg_3_reg asm("r3") = arg_3;
#define SMC_5_X_REGS SMC_4_X_REGS register long arg_4_reg asm("r4") = arg_4;
#define SMC_6_X_REGS SMC_5_X_REGS register long arg_5_reg asm("r5") = arg_5;
#define SMC_7_X_REGS SMC_6_X_REGS register long arg_6_reg asm("r6") = arg_6;
#define SMC_8_X_REGS SMC_7_X_REGS register long arg_7_reg asm("r7") = arg_7;
#define SMC_9_X_REGS SMC_8_X_REGS register long arg_8_reg asm("r8") = arg_8;

/* shortcuts for inline assembly arguments in SMC functions */
#define SMC_1_X_ASM ".arch_extension sec\nsmc #0\n" : "+r" (arg_0_reg)
#define SMC_2_X_ASM SMC_1_X_ASM,                      "+r" (arg_1_reg)
#define SMC_3_X_ASM SMC_2_X_ASM,                      "+r" (arg_2_reg)
#define SMC_4_X_ASM SMC_3_X_ASM,                      "+r" (arg_3_reg)
#define SMC_5_X_ASM SMC_4_X_ASM,                      "+r" (arg_4_reg)
#define SMC_6_X_ASM SMC_5_X_ASM,                      "+r" (arg_5_reg)
#define SMC_7_X_ASM SMC_6_X_ASM,                      "+r" (arg_6_reg)
#define SMC_8_X_ASM SMC_7_X_ASM,                      "+r" (arg_7_reg)
#define SMC_9_X_ASM SMC_8_X_ASM,                      "+r" (arg_8_reg)

/* device and function identifiers for the paravirtualized block API */
#define SMC_BLOCK                3
#define SMC_BLOCK_DEVICE_COUNT   0
#define SMC_BLOCK_BLOCK_COUNT    1
#define SMC_BLOCK_BLOCK_SIZE     2
#define SMC_BLOCK_WRITEABLE      3
#define SMC_BLOCK_QUEUE_SIZE     4
#define SMC_BLOCK_IRQ            5
#define SMC_BLOCK_START_CALLBACK 6
#define SMC_BLOCK_NEW_REQUEST    7
#define SMC_BLOCK_SUBMIT_REQUEST 8
#define SMC_BLOCK_COLLECT_REPLY  9
#define SMC_BLOCK_BUFFER         10
#define SMC_BLOCK_NAME           11

/* device and function identifiers for the paravirtualized serial API */
#define SMC_SERIAL      2
#define SMC_SERIAL_SEND 0

#endif /*  GENODE_TZ_VMM    */
#endif /* _GENODE_TZ_VMM_DEFS_H_ */
