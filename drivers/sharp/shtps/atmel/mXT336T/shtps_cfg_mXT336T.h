/* drivers/sharp/shtps/atmel/mXT336T/shtps_cfg_mXT336T.h
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __SHTPS_CFG_MXT336T_H__
#define __SHTPS_CFG_MXT336T_H__

/* ===================================================================================
 * [ Debug ]
 */
#define SHTPS_DEVELOP_MODE_ENABLE

#ifdef	SHTPS_DEVELOP_MODE_ENABLE
	#define SHTPS_LOG_DEBUG_ENABLE
	#define	SHTPS_LOG_EVENT_ENABLE
	#define	SHTPS_MODULE_PARAM_ENABLE
	#define	SHTPS_DEBUG_VARIABLE_DEFINES
	#define SHTPS_CREATE_KOBJ_ENABLE
#endif

#define	SHTPS_LOG_ERROR_ENABLE

#ifdef SHTPS_LOG_EVENT_ENABLE
	#define SHTPS_LOG_OUTPUT_SWITCH_ENABLE
#endif /* #if defined( SHTPS_LOG_EVENT_ENABLE ) */

/* ===================================================================================
 * [ Diag ]
 */

/* ===================================================================================
 * [ Model specifications ]
 */
#define SHTPS_LPWG_MODE_ENABLE


#define SHTPS_QOS_LATENCY_DEF_VALUE	 			34

/* ===================================================================================
 * [ Firmware control ]
 */

/* ===================================================================================
 * [ Hardware specifications ]
 */
#define SHTPS_PROXIMITY_SUPPORT_ENABLE

/* ===================================================================================
 * [ Performance ]
 */
#define SHTPS_CPU_CLOCK_CONTROL_ENABLE
#define SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE

#define SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE

/* ===================================================================================
 * [ Standard ]
 */
#define SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE

/* ===================================================================================
 * [ Host functions ]
 */
#if defined(SHTPS_LPWG_MODE_ENABLE)
	#define SHTPS_LPWG_GRIP_SUPPORT_ENABLE
#endif /* SHTPS_LPWG_MODE_ENABLE */

#endif /* __SHTPS_CFG_MXT336T_H__ */
