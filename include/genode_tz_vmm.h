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

#ifndef _GENODE_TZ_VMM_H_
#define _GENODE_TZ_VMM_H_

#include <genode_tz_vmm_defs.h>

/* secure monitor calls with different amount of arguments and return values */
long secure_monitor_call_1_1(SMC_1_1_ARGS);
long secure_monitor_call_2_1(SMC_2_1_ARGS);
long secure_monitor_call_3_1(SMC_3_1_ARGS);
long secure_monitor_call_4_1(SMC_4_1_ARGS);
long secure_monitor_call_5_2(SMC_5_2_ARGS);
long secure_monitor_call_7_1(SMC_7_1_ARGS);
long secure_monitor_call_8_1(SMC_8_1_ARGS);
long secure_monitor_call_9_1(SMC_9_1_ARGS);

#endif /* _GENODE_TZ_VMM_H_ */
