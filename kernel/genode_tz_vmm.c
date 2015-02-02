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

#include <genode_tz_vmm.h>

long secure_monitor_call_1_1(SMC_1_1_ARGS)
{
	SMC_1_X_REGS
	asm volatile (SMC_1_X_ASM);
	SMC_X_1_RETURN
}

long secure_monitor_call_2_1(SMC_2_1_ARGS)
{
	SMC_2_X_REGS
	asm volatile (SMC_2_X_ASM);
	SMC_X_1_RETURN
}

long secure_monitor_call_3_1(SMC_3_1_ARGS)
{
	SMC_3_X_REGS
	asm volatile (SMC_3_X_ASM);
	SMC_X_1_RETURN
}

long secure_monitor_call_4_1(SMC_4_1_ARGS)
{
	SMC_4_X_REGS
	asm volatile (SMC_4_X_ASM);
	SMC_X_1_RETURN
}

long secure_monitor_call_5_2(SMC_5_2_ARGS)
{
	SMC_5_X_REGS
	asm volatile (SMC_5_X_ASM);
	SMC_X_2_RETURN
}

long secure_monitor_call_7_1(SMC_7_1_ARGS)
{
	SMC_7_X_REGS
	asm volatile (SMC_7_X_ASM);
	SMC_X_1_RETURN
}

long secure_monitor_call_8_1(SMC_8_1_ARGS)
{
	SMC_8_X_REGS
	asm volatile (SMC_8_X_ASM);
	SMC_X_1_RETURN
}

long secure_monitor_call_9_1(SMC_9_1_ARGS)
{
	SMC_9_X_REGS
	asm volatile (SMC_9_X_ASM);
	SMC_X_1_RETURN
}
