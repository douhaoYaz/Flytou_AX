/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef __SENSOR_SC1345_SETTINGS_H__
#define __SENSOR_SC1345_SETTINGS_H__

#include "ax_sensor_struct.h"
#include "isp_sensor_internal.h"

#define SC1345_SLAVE_ADDR       (0x30) /**< i2c slave address of the SC1345 camera sensor */
#define SC1345_SENSOR_CHIP_ID   (0xda23)
#define SC1345_ADDR_BYTE        (2)
#define SC1345_DATA_BYTE        (1)
#define SC1345_SWAP_BYTES       (1)

/* AEC registers */
#define     SC1345_INT_H               (0x3E01)  /*  intigration time[7:0]  */
#define     SC1345_INT_L               (0x3E02)  /*  intigration time[7:4]  */

/* AGC registers */
#define     SC1345_AGAIN               (0x3E08)  /* ANA GAIN register */
#define     SC1345_AGAIN_FINE          (0x3E09)  /* ANA FINE GAIN register */
#define     SC1345_DGAIN               (0x3E06)  /* DIG GAIN register */
#define     SC1345_DGAIN_FINE          (0x3E07)  /* DIG FINE GAIN register */

#define     SC1345_AGAIN_CALIB_0       (0x363A)  /* calib from again */
#define     SC1345_AGAIN_CALIB_1       (0x3622)  /* calib from again */


typedef struct SENSOR_SC1345_S {
    AX_U32 int_l;
    AX_U32 int_s;
    AX_U32 vts;
    AX_F32 fps;
    AX_F32 line_period;
    AX_U32 line_period_fixednum;    /* U22.10 */
} SNSSC1345_OBJ;

static camera_i2c_reg_array SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_25fps_RGGB[] = {

    {0x0103, 0x01},
    {0x0100, 0x00},
    {0x36e9, 0x80},
    {0x301f, 0x04},
    {0x320c, 0x06},
    {0x320d, 0x90},
    {0x320e, 0x03}, //25fps
    {0x320f, 0x84}, //25fps
    {0x3211, 0x03},
    {0x3213, 0x03},
    {0x3253, 0x0a},
    {0x3301, 0x04},
    {0x3306, 0x30},
    {0x3309, 0x48},
    {0x330b, 0xb0},
    {0x330e, 0x18},
    {0x331f, 0x41},
    {0x3320, 0x05},
    {0x3333, 0x10},
    {0x3364, 0x17},
    {0x3390, 0x08},
    {0x3391, 0x18},
    {0x3392, 0x38},
    {0x3393, 0x06},
    {0x3394, 0x10},
    {0x3395, 0x40},
    {0x3620, 0x08},
    {0x3622, 0xc6},
    {0x3630, 0xc0},
    {0x3633, 0x33},
    {0x3637, 0x14},
    {0x3638, 0x0e},
    {0x363a, 0x00},
    {0x363c, 0x05},
    {0x3670, 0x1e},
    {0x3674, 0x90},
    {0x3675, 0x90},
    {0x3676, 0x90},
    {0x3677, 0x82},
    {0x3678, 0x86},
    {0x3679, 0x8b},
    {0x367c, 0x18},
    {0x367d, 0x38},
    {0x367e, 0x18},
    {0x367f, 0x38},
    {0x3690, 0x33},
    {0x3691, 0x33},
    {0x3692, 0x32},
    {0x369c, 0x08},
    {0x369d, 0x38},
    {0x36a4, 0x08},
    {0x36a5, 0x18},
    {0x36a8, 0x02},
    {0x36a9, 0x04},
    {0x36aa, 0x0e},
    {0x36ea, 0xc7},
    {0x36eb, 0x88},
    {0x36ec, 0x1d},
    {0x36ed, 0x07},
    {0x3e00, 0x00},
    {0x3e01, 0x2e},
    {0x3e02, 0x00},
    {0x3e08, 0x03},
    {0x3e09, 0x20},
    {0x4509, 0x20},
    {0x36e9, 0x23},
    {0x0100, 0x01},
/*
    //[gain<2]
    {0x363a, 0x0c},
    {0x3622, 0xc6},
    //[2=<gain<4]
    {0x363a, 0x16},
    {0x3622, 0xc6},
    //[4=<gain<8]
    {0x363a, 0x16},
    {0x3622, 0xd6},
    //[gain>=8]
    {0x363a, 0x1d},
    {0x3622, 0xd6},
*/
    {0x0000,  0x00},  // end flag
};

static camera_i2c_reg_array SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_30fps_RGGB[] = {

    {0x0103, 0x01},
    {0x0100, 0x00},
    {0x36e9, 0x80},
    {0x301f, 0x04},
    {0x320c, 0x06},
    {0x320d, 0x90},
    {0x320e, 0x02}, //30fps
    {0x320f, 0xee}, //30fps
    {0x3211, 0x03},
    {0x3213, 0x03},
    {0x3253, 0x0a},
    {0x3301, 0x04},
    {0x3306, 0x30},
    {0x3309, 0x48},
    {0x330b, 0xb0},
    {0x330e, 0x18},
    {0x331f, 0x41},
    {0x3320, 0x05},
    {0x3333, 0x10},
    {0x3364, 0x17},
    {0x3390, 0x08},
    {0x3391, 0x18},
    {0x3392, 0x38},
    {0x3393, 0x06},
    {0x3394, 0x10},
    {0x3395, 0x40},
    {0x3620, 0x08},
    {0x3622, 0xc6},
    {0x3630, 0xc0},
    {0x3633, 0x33},
    {0x3637, 0x14},
    {0x3638, 0x0e},
    {0x363a, 0x00},
    {0x363c, 0x05},
    {0x3670, 0x1e},
    {0x3674, 0x90},
    {0x3675, 0x90},
    {0x3676, 0x90},
    {0x3677, 0x82},
    {0x3678, 0x86},
    {0x3679, 0x8b},
    {0x367c, 0x18},
    {0x367d, 0x38},
    {0x367e, 0x18},
    {0x367f, 0x38},
    {0x3690, 0x33},
    {0x3691, 0x33},
    {0x3692, 0x32},
    {0x369c, 0x08},
    {0x369d, 0x38},
    {0x36a4, 0x08},
    {0x36a5, 0x18},
    {0x36a8, 0x02},
    {0x36a9, 0x04},
    {0x36aa, 0x0e},
    {0x36ea, 0xc7},
    {0x36eb, 0x88},
    {0x36ec, 0x1d},
    {0x36ed, 0x07},
    {0x3e00, 0x00},
    {0x3e01, 0x2e},
    {0x3e02, 0x00},
    {0x3e08, 0x03},
    {0x3e09, 0x20},
    {0x4509, 0x20},
    {0x36e9, 0x23},
    {0x0100, 0x01},
/*
    //[gain<2]
    {0x363a, 0x0c},
    {0x3622, 0xc6},
    //[2=<gain<4]
    {0x363a, 0x16},
    {0x3622, 0xc6},
    //[4=<gain<8]
    {0x363a, 0x16},
    {0x3622, 0xd6},
    //[gain>=8]
    {0x363a, 0x1d},
    {0x3622, 0xd6},
*/
    {0x0000, 0x00},  // end flag
};

typedef enum {
    e_SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_25fps_RGGB,
    e_SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_30fps_RGGB,
    SC1345_SETTTING_SEL_MAX
} SC1345_SETTING_SEL_E;

static camera_i2c_reg_array *sc1345_settings_table[] = {
    SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_25fps_RGGB,
    SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_30fps_RGGB,
    NULL
};


AX_S32 sc1345_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum);
AX_S32 sc1345_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr);
AX_S32 sc1345_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data);
AX_S32 sc1345_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex);
AX_F32 sc1345_dgain2value(float gain_value, AX_U8 *gain, AX_U8 *gain_fine);
AX_F32 sc1345_again2value(float gain_value, AX_U8 *gain, AX_U8 *gain_fine);
AX_U32 sc1345_get_vts(ISP_PIPE_ID nPipeId);
AX_U32 sc1345_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts);
AX_U32 sc1345_set_int_t_l(ISP_PIPE_ID nPipeId, AX_U32 int_t_l);
AX_S32 sc1345_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params);
AX_S32 sc1345_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum);

AX_U32 sc1345_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue);
AX_S32 sc1345_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo);
AX_U32 sc1345_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId);

#endif
