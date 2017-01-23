/* include/sharp/shub_driver.h  (Shub Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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

#ifndef SHUB_DRIVER_H
#define SHUB_DRIVER_H

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHUB_STOP_PED_TYPE_VIB,
    SHUB_STOP_PED_TYPE_TPS,
    NUM_SHUB_STOP_PED_TYPE
};


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shub_api_stop_pedometer_func (int type);
int shub_api_restart_pedometer_func (int type);


#endif /* SHUB_DRIVER_H */
