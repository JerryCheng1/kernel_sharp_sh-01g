/* drivers/sharp/atmel/shtps_mxt.h
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
#ifndef __SHTPS_ATMEL_MXT_H__
#define __SHTPS_ATMEL_MXT_H__
/* --------------------------------------------------------------------------- */
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>

#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/cpuidle.h>

#include <sharp/proximity.h>
#include <sharp/shtps_dev.h>

#include "shtps_cfg.h"
#include "shtps_fwctl.h"
#include "shtps_mxt_debug.h"

/* ===================================================================================
 * Debug
 */
#define	SHTPS_DEVELOP_MODE_ENABLE

/* ===================================================================================
 * Common
 */

/* ===================================================================================
 * enum
 */
enum{
	SHTPS_STATE_IDLE,
	SHTPS_STATE_ACTIVE,
	SHTPS_STATE_SLEEP,
};

enum{
	SHTPS_EVENT_TU,
	SHTPS_EVENT_TD,
	SHTPS_EVENT_DRAG,
	SHTPS_EVENT_MTDU,
};

#if defined(SHTPS_LPWG_MODE_ENABLE)
enum{
	SHTPS_LPWG_STATE_OFF		= 0x00,
	SHTPS_LPWG_STATE_ON			= 0x01,
	SHTPS_LPWG_STATE_GRIP_ONLY	= 0x02,
};
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
enum{
	SHTPS_PROXIMITY_ENABLE = SH_PROXIMITY_ENABLE,
	SHTPS_PROXIMITY_DISABLE = SH_PROXIMITY_DISABLE,
};

enum{
	SHTPS_PROXIMITY_NEAR = SH_PROXIMITY_NEAR,
	SHTPS_PROXIMITY_FAR = SH_PROXIMITY_FAR,
};
#endif

/* ===================================================================================
 * Structure
 */
struct shtps_state_info{
	int							state;
	int							mode;
	int							starterr;
	unsigned long				starttime;
};

struct shtps_lpwg_ctrl{		//**********	SHTPS_LPWG_MODE_ENABLE
	u8							lpwg_switch;
	u8							lpwg_state;
	u8							notify_enable;

	struct wake_lock			wake_lock;
	struct pm_qos_request		pm_qos_lock_idle;
	signed long				pm_qos_idle_value;
	struct delayed_work			notify_interval_delayed_work;
	unsigned long				wakeup_time;
	
	#if defined( SHTPS_LPWG_GRIP_SUPPORT_ENABLE )
		u8							grip_state;
	#endif /* SHTPS_LPWG_GRIP_SUPPORT_ENABLE */
};

struct shtps_diag_info {
	int							input_event_detect;
	int							poll_stop_request;
	wait_queue_head_t			input_event_wait;
};

struct shtps_mxt {
	struct device *dev;
	int (*resume)(struct device *dev);
	int (*suspend)(struct device *dev);
	
	struct shtps_input_event_info	input_event_info;
	struct shtps_diag_info			diag;
	struct kobject					*kobj;

	struct shtps_state_info		state_mgr;

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
		char						phys[32];
		struct input_dev*			input_key;
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		struct shtps_lpwg_ctrl		lpwg;
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		struct shtps_cpu_clock_ctrl_info					*cpu_clock_ctrl_p;
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		struct shtps_cpu_idle_sleep_ctrl_info				*cpu_idle_sleep_ctrl_p;
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	struct shtps_fwctl_info	*fwctl_p;
};
extern struct shtps_mxt*	gShtps_mxt;


/* ===================================================================================
 * Function
 */
/* atmel_mxt_ts */
int mxt_power_enable(struct device *dev);
int mxt_power_disable(struct device *dev);
int mxt_hard_reset(struct device *dev);
int mxt_set_sleep(struct device *dev, int on);
int mxt_set_lpwg(struct device *dev, int on, struct shtps_lpwg_info *info);


void shtps_mutex_lock_ctrl(void);
void shtps_mutex_unlock_ctrl(void);
void shtps_diag_input_event_poll_stop_request(void);
int shtps_diag_input_event_poll(struct shtps_input_event_info **input_event_info_pp);
void shtps_sleep(struct shtps_mxt *ts, int sleep);

void shtps_irq_disable(struct shtps_mxt *ts);
void shtps_irq_enable(struct shtps_mxt *ts);

#if defined(SHTPS_LPWG_MODE_ENABLE)
	void shtps_set_lpwg_mode_on(struct shtps_mxt *ts);
	void shtps_set_lpwg_mode_off(struct shtps_mxt *ts);
	int shtps_is_lpwg_active(struct shtps_mxt *ts);
	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		void shtps_notify_cancel_wakeup_event(struct shtps_mxt *ts);
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE */

int shtps_get_logflag(void);

#endif /* __SHTPS_ATMEL_MXT_H__ */
