/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef __SENSOR_IMX327_SETTINGS_H__
#define __SENSOR_IMX327_SETTINGS_H__

#include "ax_sensor_struct.h"
#include "isp_sensor_internal.h"

#define IMX327_SLAVE_ADDR       (0x1a) /**< i2c slave address of the imx327 camera sensor */
#define IMX327_SENSOR_CHIP_ID   (0x0327)
#define IMX327_ADDR_BYTE        (2)
#define IMX327_DATA_BYTE        (1)
#define IMX327_SWAP_BYTES       (1)

#define     IMX327_STANDBY              (0x3000)  /* 1h:standby,  0h:Operating  */
#define     IMX327_XMSTA                (0x3002)  /* 1h:master operation stop,  0h:master operation start  */

#define     IMX327_FDG_SEL              (0x3009)  /* SHR1[4:4] */   /* 1:HCG,  0:LCG  */

/* Exposure control related registers */
#define     IMX327_SHS1_L               (0x3020)  /* SHR1[7:0] */
#define     IMX327_SHS1_M               (0x3021)  /* SHR1[7:0] */
#define     IMX327_SHS1_H               (0x3022)  /* SHR1[3:0] */

#define     IMX327_SHS2_L               (0x3024)  /* SHR2[7:0] */
#define     IMX327_SHS2_M               (0x3025)  /* SHR2[7:0] */
#define     IMX327_SHS2_H               (0x3026)  /* SHR2[3:0] */

#define     IMX327_RHS1_L               (0x3030)  /* RHS1[7:0] */
#define     IMX327_RHS1_M               (0x3031)  /* RHS1[7:0] */
#define     IMX327_RHS1_H               (0x3032)  /* RHS1[3:0] */

#define     IMX327_GAIN                 (0x3014)  /* GAIN[7:0] */
#define     IMX327_GAIN1                (0x30F2)  /* GAIN1[7:0] */

#define     IMX327_VMAX_L               (0x3018)  /* VMAX[7:0] */
#define     IMX327_VMAX_M               (0x3019)  /* VMAX[7:0] */
#define     IMX327_VMAX_H               (0x301a)  /* VMAX[1:0] */

#define     IMX327_HMAX_L               (0x301c)  /* HMAX[7:0] */
#define     IMX327_HMAX_H               (0x301d)  /* HMAX[7:0] */

#define     IMX327_L2S_LINEGAP_CALC(a)  ((a - 1) / 2)  /* (rhs1 - 1)/2 */

typedef enum {
    e_imx327_4line_1920x1080_linear_rggb_12bit_25fps,
    e_imx327_4line_1920x1080_linear_rggb_12bit_30fps,
    IMX327_SETTTING_SEL_MAX
} IMX327_SETTING_SEL_E;

static camera_i2c_reg_array imx327_4line_1920x1080_linear_rggb_12bit_25fps[] = {
    /* IMX327 4lane 1920x1080 Linear ADC12Bit 25fps */

    {0x3000,  0x01},  // Standby mode
    {0x3002,  0x01},  // Master mode stop
    {0x3005,  0x01},  // ADBIT,12Bit;0x00->10Bit
    {0x3007,  0x00},  // Full HD 1080P
    {0x3009,  0x12},  // FRSEL,LCG,30fps
    {0x300a,  0xf0},
    {0x3018,  0x65},  // VMAX
    {0x3019,  0x04},  //
    {0x301c,  0xa0},  // HMAX;25fps;0x50->50fps
    {0x301d,  0x14},  // HMAX;25fps;0x0a->50fps
    {0x3046,  0x01},  // MIPI 4lane,12Bit
    {0x3048,  0x10},  // XVSLNG

    {0x305C,  0x18},  // INCKSEL1,1080P,CSI-2,37.125MHz;74.25MHz->0x0C
    {0x305D,  0x03},  // INCKSEL2,1080P,CSI-2,37.125MHz;74.25MHz->0x03
    {0x305E,  0x20},  // INCKSEL3,1080P,CSI-2,37.125MHz;74.25MHz->0x10
    {0x305F,  0x01},  // INCKSEL4,1080P,CSI-2,37.125MHz;74.25MHz->0x01

    {0x3011,  0x0a},  // -
    {0x3012,  0x64},  // -
    {0x3013,  0x00},  // -
    {0x309e,  0x4a},  // -
    {0x309f,  0x4a},  // -
    {0x30d2,  0x19},  // -
    {0x30d7,  0x03},  // -

//chip id:03
    {0x3128,  0x04},  // -
    {0x313b,  0x41},  // -
    {0x3129,  0x00},  // ADBIT1,12Bit;0x1d->10bit
    {0x315e,  0x1a},  // INCKSEL5,37.125MHz;0x1b->74.25MHz
    {0x3164,  0x1a},  // INCKSEL6,37.125MHz;0x1b->74.25MHz
    {0x317c,  0x00},  // ADBIT2,12Bit
    {0x31ec,  0x0e},  // ADBIT3,12Bit
    {0x3480,  0x49},  // INCKSEL7,37.125MHz;0x92->74.25MHz

//CHIP ID 06 For MIPI I/F
    {0x3404,  0x01},  // -
    {0x3405,  0x20},  // 30/25fps,222.75Mbps/lane
    {0x3407,  0x03},  // -
    {0x3414,  0x0a},  // OPB_SIZE_V
    {0x3418,  0x49},  // -
    {0x3419,  0x04},  // -
    {0x3441,  0x0c},  // CSI_DT_FMT,12Bit
    {0x3442,  0x0c},  // -
    {0x3443,  0x03},  // CSI_LANE_MODE,4Lane
    {0x3444,  0x20},  // EXTCK_FREQ,37.125MHz;0x40->74.25MHz
    {0x3445,  0x25},  // EXTCK_FREQ,37.125MHz;0x4A->74.25MHz

    {0x3446,  0x47},  // TCLKPOST
    {0x3447,  0x00},  // -
    {0x3448,  0x1f},  // THSZERO
    {0x3449,  0x00},  // -
    {0x344A,  0x17},  // THSPREPARE
    {0x344B,  0x00},  // -
    {0x344C,  0x0F},  // TCLKTRAIL
    {0x344D,  0x00},  // -
    {0x344E,  0x17},  // THSTRAIL
    {0x344F,  0x00},  // -
    {0x3450,  0x47},  // TCLKZERO
    {0x3451,  0x00},  // -
    {0x3452,  0x0F},  // TCLKPREPARE
    {0x3453,  0x00},  // -
    {0x3454,  0x0f},  // TLPX
    {0x3455,  0x00},  // -
    {0x3472,  0x9C},  // X_OUT_SIZE
    {0x3473,  0x07},  // -

// AE register setting
    {0x3020,  0x00},  // exposure time setting, Maximum
    {0x3021,  0x00},  // -
    {0x3014,  0x00},  // Gain register setting 0DB

    {0x304b,  0x0a},  // XVS/XHS output

    {0x0000,  0x00},  // end flag

};

static camera_i2c_reg_array imx327_4line_1920x1080_linear_rggb_12bit_30fps[] = {
    /* IMX327 4lane 1920x1080 Linear ADC12Bit 30fps */

    {0x3000,  0x01}, // Standby mode
    {0x3002,  0x01}, // Master mode stop
    {0x3005,  0x01}, // ADBIT,12Bit;0x00->10Bit
    {0x3007,  0x00}, // Full HD 1080P
    {0x3009,  0x12}, // FRSEL,HCG,30fps
    {0x300a,  0xf0}, //
    {0x3018,  0x65}, // VMAX
    {0x3019,  0x04}, //
    {0x301c,  0x30}, // HMAX;30fps;0xA0->25fps
    {0x301d,  0x11}, // HMAX;30fps;0x14->25fps
    {0x3046,  0x01}, // MIPI 4lane,12Bit
    {0x3048,  0x10}, // XVSLNG

    {0x305C,  0x18}, // INCKSEL1,1080P,CSI-2,37.125MHz;74.25MHz->0x0C
    {0x305D,  0x03}, // INCKSEL2,1080P,CSI-2,37.125MHz;74.25MHz->0x03
    {0x305E,  0x20}, // INCKSEL3,1080P,CSI-2,37.125MHz;74.25MHz->0x10
    {0x305F,  0x01}, // INCKSEL4,1080P,CSI-2,37.125MHz;74.25MHz->0x01

    {0x3011,  0x02}, //
    {0x3013,  0x00}, //
    {0x309e,  0x4a}, // -
    {0x309f,  0x4a}, // -

    {0x30d2,  0x19}, // -
    {0x30d7,  0x03}, // -

//chip id:03
    {0x3128,  0x04}, // -
    {0x313b,  0x61}, // -
    {0x3129,  0x00}, // ADBIT1,12Bit;0x1d->10bit
    {0x315e,  0x1a}, // INCKSEL5,37.125MHz;0x1b->74.25MHz
    {0x3164,  0x1a}, // INCKSEL6,37.125MHz;0x1b->74.25MHz
    {0x317c,  0x00}, // ADBIT2,12Bit
    {0x31ec,  0x0e}, // ADBIT3,12Bit
    {0x3480,  0x49}, // INCKSEL7,37.125MHz;0x92->74.25MHz

//CHIP ID 06 For MIPI I/F
    {0x3405,  0x20}, // 30/25fps,222.75Mbps/lane
    {0x3407,  0x03}, // -
    {0x3414,  0x0a}, // OPB_SIZE_
    {0x3418,  0x49}, // -
    {0x3419,  0x04}, // -
    {0x3441,  0x0c}, // CSI_DT_FMT��12Bit
    {0x3442,  0x0c}, // -
    {0x3443,  0x03}, // CSI_LANE_MODE,4Lane
    {0x3444,  0x20}, // EXTCK_FREQ,37.125MHz;0x40->74.25MHz
    {0x3445,  0x25}, // EXTCK_FREQ,37.125MHz;0x4A->74.25MHz

    {0x3446,  0x47}, // TCLKPOST
    {0x3447,  0x00}, // -
    {0x3448,  0x1f}, // THSZERO
    {0x3449,  0x00}, // -
    {0x344A,  0x17}, // THSPREPARE
    {0x344B,  0x00}, // -
    {0x344C,  0x0F}, // TCLKTRAIL
    {0x344D,  0x00}, // -
    {0x344E,  0x17}, // THSTRAIL
    {0x344F,  0x00}, // -
    {0x3450,  0x47}, // TCLKZERO
    {0x3451,  0x00}, // -
    {0x3452,  0x0F}, // TCLKPREPARE
    {0x3453,  0x00}, // -
    {0x3454,  0x0f}, // TLPX
    {0x3455,  0x00}, // -
    {0x3472,  0x9C}, // X_OUT_SIZE
    {0x3473,  0x07}, // -

// AE register setting
    {0x3020,  0x00}, // exposure time setting, Maximum
    {0x3021,  0x00}, // -
    {0x3014,  0x00}, // Gain register setting 0DB

    {0x304b,  0x0a}, // XVS/XHS output

    {0x0000,  0x00},  // end flag

};

static camera_i2c_reg_array *imx327_settings_table[] = {
    imx327_4line_1920x1080_linear_rggb_12bit_25fps,
    imx327_4line_1920x1080_linear_rggb_12bit_30fps,
    NULL
};

AX_S32 imx327_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum);
AX_S32 imx327_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr);
AX_S32 imx327_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data);
AX_S32 imx327_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex);
AX_U32 imx327_get_hmax(ISP_PIPE_ID nPipeId);
AX_U32 imx327_get_vmax(ISP_PIPE_ID nPipeId);
AX_U32 imx327_set_vmax(ISP_PIPE_ID nPipeId, AX_U32 vmax);
AX_S32 imx327_set_rhs1(ISP_PIPE_ID nPipeId, AX_U32 rhs1);
AX_U32 imx327_get_rhs1(ISP_PIPE_ID nPipeId);
AX_S32 imx327_set_shs2(ISP_PIPE_ID nPipeId, AX_U32 shs2);
AX_U32 imx327_get_shs2(ISP_PIPE_ID nPipeId);
AX_S32 imx327_set_shs1(ISP_PIPE_ID nPipeId, AX_U32 shs1);
AX_U32 imx327_get_shs1(ISP_PIPE_ID nPipeId);

AX_U32 imx327_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue);
AX_S32 imx327_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo);
AX_U32 imx327_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId);

#endif

