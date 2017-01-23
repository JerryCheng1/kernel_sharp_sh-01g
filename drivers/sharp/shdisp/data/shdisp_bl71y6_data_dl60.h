/* drivers/sharp/shdisp/data/shdisp_bl71y6_data_dl60.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_BL71Y6_DATA_DL60_H
#define SHDISP_BL71Y6_DATA_DL60_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "../shdisp_bl71y6.h"
#include "./shdisp_bl71y6_data_dl6x.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_TRI_LED_COLOR_TBL_NUM                (11)
#define SHDISP_COL_VARI_KIND                        (3)
#define SHDISP_HANDSET_COLOR_WHITE                  (0x01)
#define SHDISP_HANDSET_COLOR_PINK                   (0x02)
#define SHDISP_HANDSET_COLOR_BLACK                  (0x06)
#define SHDISP_TRI_LED_ANIME_2PAGE                  (2)
#define SHDISP_TRI_LED_ANIME_3PAGE                  (3)
/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static const struct shdisp_bdic_led_color_index shdisp_triple_led_color_index_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM] = {
    {0, 0, 0,  0},
    {1, 0, 0,  1},
    {0, 1, 0,  2},
    {1, 1, 0,  3},
    {0, 0, 1,  4},
    {1, 0, 1,  5},
    {0, 1, 1,  6},
    {1, 1, 1,  7},
    {2, 1, 1,  8},
    {2, 0, 0,  9},
    {0, 2, 0, 10}
};

static const unsigned char shdisp_clrvari_index[SHDISP_COL_VARI_KIND] = {
    SHDISP_HANDSET_COLOR_BLACK,
    SHDISP_HANDSET_COLOR_PINK,
    SHDISP_HANDSET_COLOR_WHITE
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F },
    { 0x0C, 0x06, 0x06 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F },
    { 0x0C, 0x06, 0x06 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F },
    { 0x0C, 0x06, 0x06 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 }
  }
};

static const unsigned char shdisp_triple_led_anime_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_ANIME_2PAGE][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F },
        { 0x0C, 0x06, 0x06 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 }
    }
  },
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F },
        { 0x0C, 0x06, 0x06 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 }
    }
  },
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F },
        { 0x0C, 0x06, 0x06 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 }
    }
  }
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

#endif /* SHDISP_BL71Y6_DATA_DL60_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
