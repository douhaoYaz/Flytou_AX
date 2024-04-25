/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "i2c.h"
#include "ax_sensor_struct.h"
#include "ax_base_type.h"
#include "sc230ai_settings.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gSc230aiBusInfo[DEF_VIN_PIPE_MAX_NUM];
extern SNSSC230AI_OBJ sns_sc230aiparams[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])

AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

static AX_SNS_DRV_DELAY_TABLE_T gSc230aiAeRegsTable[] = {

    /* nRegAddr */         /* regs value */    /* Delay Frame Number */
    {SC230AI_INT_L_0,               0,                  0},
    {SC230AI_INT_L_1,               0,                  0},
    {SC230AI_INT_L_2,               0,                  0},
    {SC230AI_INT_S_0,               0,                  0},
    {SC230AI_INT_S_1,               0,                  0},
    {SC230AI_INT_S_2,               0,                  0},
    {SC230AI_AGAIN_L,               0,                  0},
    {SC230AI_AGAIN_S,               0,                  0},
    {SC230AI_DGAIN_L,               0,                  0},
    {SC230AI_DGAIN_S,               0,                  0},
    {SC230AI_DGAIN_FINE_L,          0,                  0},
    {SC230AI_DGAIN_FINE_S,          0,                  0},
};

typedef struct _sc230ai_GAIN_TABLE_T_ {
    float gain;
    AX_U8 gain_in;
    AX_U8 gain_de;
} sc230ai_GAIN_TABLE_T;

const sc230ai_GAIN_TABLE_T sc230ai_again_table[] = {
    {1.000, 0x00},
    {2.000, 0x01},
    {3.391, 0x40},
    {6.782, 0x48},
    {13.564, 0x49},
    {27.128, 0x4B},
    {54.256, 0x4F},
    {108.512, 0x5F},
};

const sc230ai_GAIN_TABLE_T sc230ai_dgain_table[] = {

    {1.000, 0x00, 0x80},
    {1.016, 0x00, 0x82},
    {1.031, 0x00, 0x84},
    {1.047, 0x00, 0x86},
    {1.063, 0x00, 0x88},
    {1.078, 0x00, 0x8a},
    {1.094, 0x00, 0x8c},
    {1.109, 0x00, 0x8e},
    {1.125, 0x00, 0x90},
    {1.141, 0x00, 0x92},
    {1.156, 0x00, 0x94},
    {1.172, 0x00, 0x96},
    {1.188, 0x00, 0x98},
    {1.203, 0x00, 0x9a},
    {1.219, 0x00, 0x9c},
    {1.234, 0x00, 0x9e},
    {1.250, 0x00, 0xa0},
    {1.266, 0x00, 0xa2},
    {1.281, 0x00, 0xa4},
    {1.297, 0x00, 0xa6},
    {1.313, 0x00, 0xa8},
    {1.328, 0x00, 0xaa},
    {1.344, 0x00, 0xac},
    {1.359, 0x00, 0xae},
    {1.375, 0x00, 0xb0},
    {1.391, 0x00, 0xb2},
    {1.406, 0x00, 0xb4},
    {1.422, 0x00, 0xb6},
    {1.438, 0x00, 0xb8},
    {1.453, 0x00, 0xba},
    {1.469, 0x00, 0xbc},
    {1.484, 0x00, 0xbe},
    {1.500, 0x00, 0xc0},
    {1.516, 0x00, 0xc2},
    {1.531, 0x00, 0xc4},
    {1.547, 0x00, 0xc6},
    {1.563, 0x00, 0xc8},
    {1.578, 0x00, 0xca},
    {1.594, 0x00, 0xcc},
    {1.609, 0x00, 0xce},
    {1.625, 0x00, 0xd0},
    {1.641, 0x00, 0xd2},
    {1.656, 0x00, 0xd4},
    {1.672, 0x00, 0xd6},
    {1.688, 0x00, 0xd8},
    {1.703, 0x00, 0xda},
    {1.719, 0x00, 0xdc},
    {1.734, 0x00, 0xde},
    {1.750, 0x00, 0xe0},
    {1.766, 0x00, 0xe2},
    {1.781, 0x00, 0xe4},
    {1.797, 0x00, 0xe6},
    {1.813, 0x00, 0xe8},
    {1.828, 0x00, 0xea},
    {1.844, 0x00, 0xec},
    {1.859, 0x00, 0xee},
    {1.875, 0x00, 0xf0},
    {1.891, 0x00, 0xf2},
    {1.906, 0x00, 0xf4},
    {1.922, 0x00, 0xf6},
    {1.938, 0x00, 0xf8},
    {1.953, 0x00, 0xfa},
    {1.969, 0x00, 0xfc},
    {1.984, 0x00, 0xfe},
    {2.000, 0x01, 0x80},
    {2.031, 0x01, 0x82},
    {2.063, 0x01, 0x84},
    {2.094, 0x01, 0x86},
    {2.125, 0x01, 0x88},
    {2.156, 0x01, 0x8a},
    {2.188, 0x01, 0x8c},
    {2.219, 0x01, 0x8e},
    {2.250, 0x01, 0x90},
    {2.281, 0x01, 0x92},
    {2.313, 0x01, 0x94},
    {2.344, 0x01, 0x96},
    {2.375, 0x01, 0x98},
    {2.406, 0x01, 0x9a},
    {2.438, 0x01, 0x9c},
    {2.469, 0x01, 0x9e},
    {2.500, 0x01, 0xa0},
    {2.531, 0x01, 0xa2},
    {2.563, 0x01, 0xa4},
    {2.594, 0x01, 0xa6},
    {2.625, 0x01, 0xa8},
    {2.656, 0x01, 0xaa},
    {2.688, 0x01, 0xac},
    {2.719, 0x01, 0xae},
    {2.750, 0x01, 0xb0},
    {2.781, 0x01, 0xb2},
    {2.813, 0x01, 0xb4},
    {2.844, 0x01, 0xb6},
    {2.875, 0x01, 0xb8},
    {2.906, 0x01, 0xba},
    {2.938, 0x01, 0xbc},
    {2.969, 0x01, 0xbe},
    {3.000, 0x01, 0xc0},
    {3.031, 0x01, 0xc2},
    {3.063, 0x01, 0xc4},
    {3.094, 0x01, 0xc6},
    {3.125, 0x01, 0xc8},
    {3.156, 0x01, 0xca},
    {3.188, 0x01, 0xcc},
    {3.219, 0x01, 0xce},
    {3.250, 0x01, 0xd0},
    {3.281, 0x01, 0xd2},
    {3.313, 0x01, 0xd4},
    {3.344, 0x01, 0xd6},
    {3.375, 0x01, 0xd8},
    {3.406, 0x01, 0xda},
    {3.438, 0x01, 0xdc},
    {3.469, 0x01, 0xde},
    {3.500, 0x01, 0xe0},
    {3.531, 0x01, 0xe2},
    {3.563, 0x01, 0xe4},
    {3.594, 0x01, 0xe6},
    {3.625, 0x01, 0xe8},
    {3.656, 0x01, 0xea},
    {3.688, 0x01, 0xec},
    {3.719, 0x01, 0xee},
    {3.750, 0x01, 0xf0},
    {3.781, 0x01, 0xf2},
    {3.813, 0x01, 0xf4},
    {3.844, 0x01, 0xf6},
    {3.875, 0x01, 0xf8},
    {3.906, 0x01, 0xfa},
    {3.938, 0x01, 0xfc},
    {3.969, 0x01, 0xfe},
    {4.000, 0x03, 0x80},
    {4.063, 0x03, 0x82},
    {4.125, 0x03, 0x84},
    {4.188, 0x03, 0x86},
    {4.250, 0x03, 0x88},
    {4.313, 0x03, 0x8a},
    {4.375, 0x03, 0x8c},
    {4.438, 0x03, 0x8e},
    {4.500, 0x03, 0x90},
    {4.563, 0x03, 0x92},
    {4.625, 0x03, 0x94},
    {4.688, 0x03, 0x96},
    {4.750, 0x03, 0x98},
    {4.813, 0x03, 0x9a},
    {4.875, 0x03, 0x9c},
    {4.938, 0x03, 0x9e},
    {5.000, 0x03, 0xa0},
    {5.063, 0x03, 0xa2},
    {5.125, 0x03, 0xa4},
    {5.188, 0x03, 0xa6},
    {5.250, 0x03, 0xa8},
    {5.313, 0x03, 0xaa},
    {5.375, 0x03, 0xac},
    {5.438, 0x03, 0xae},
    {5.500, 0x03, 0xb0},
    {5.563, 0x03, 0xb2},
    {5.625, 0x03, 0xb4},
    {5.688, 0x03, 0xb6},
    {5.750, 0x03, 0xb8},
    {5.813, 0x03, 0xba},
    {5.875, 0x03, 0xbc},
    {5.938, 0x03, 0xbe},
    {6.000, 0x03, 0xc0},
    {6.063, 0x03, 0xc2},
    {6.125, 0x03, 0xc4},
    {6.188, 0x03, 0xc6},
    {6.250, 0x03, 0xc8},
    {6.313, 0x03, 0xca},
    {6.375, 0x03, 0xcc},
    {6.438, 0x03, 0xce},
    {6.500, 0x03, 0xd0},
    {6.563, 0x03, 0xd2},
    {6.625, 0x03, 0xd4},
    {6.688, 0x03, 0xd6},
    {6.750, 0x03, 0xd8},
    {6.813, 0x03, 0xda},
    {6.875, 0x03, 0xdc},
    {6.938, 0x03, 0xde},
    {7.000, 0x03, 0xe0},
    {7.063, 0x03, 0xe2},
    {7.125, 0x03, 0xe4},
    {7.188, 0x03, 0xe6},
    {7.250, 0x03, 0xe8},
    {7.313, 0x03, 0xea},
    {7.375, 0x03, 0xec},
    {7.438, 0x03, 0xee},
    {7.500, 0x03, 0xf0},
    {7.563, 0x03, 0xf2},
    {7.625, 0x03, 0xf4},
    {7.688, 0x03, 0xf6},
    {7.750, 0x03, 0xf8},
    {7.813, 0x03, 0xfa},
    {7.875, 0x03, 0xfc},
    {7.938, 0x03, 0xfe},
    {8.000, 0x07, 0x80},
    {8.125, 0x07, 0x82},
    {8.250, 0x07, 0x84},
    {8.375, 0x07, 0x86},
    {8.500, 0x07, 0x88},
    {8.625, 0x07, 0x8a},
    {8.750, 0x07, 0x8c},
    {8.875, 0x07, 0x8e},
    {9.000, 0x07, 0x90},
    {9.125, 0x07, 0x92},
    {9.250, 0x07, 0x94},
    {9.375, 0x07, 0x96},
    {9.500, 0x07, 0x98},
    {9.625, 0x07, 0x9a},
    {9.750, 0x07, 0x9c},
    {9.875, 0x07, 0x9e},
    {10.000, 0x07, 0xa0},
    {10.125, 0x07, 0xa2},
    {10.250, 0x07, 0xa4},
    {10.375, 0x07, 0xa6},
    {10.500, 0x07, 0xa8},
    {10.625, 0x07, 0xaa},
    {10.750, 0x07, 0xac},
    {10.875, 0x07, 0xae},
    {11.000, 0x07, 0xb0},
    {11.125, 0x07, 0xb2},
    {11.250, 0x07, 0xb4},
    {11.375, 0x07, 0xb6},
    {11.500, 0x07, 0xb8},
    {11.625, 0x07, 0xba},
    {11.750, 0x07, 0xbc},
    {11.875, 0x07, 0xbe},
    {12.000, 0x07, 0xc0},
    {12.125, 0x07, 0xc2},
    {12.250, 0x07, 0xc4},
    {12.375, 0x07, 0xc6},
    {12.500, 0x07, 0xc8},
    {12.625, 0x07, 0xca},
    {12.750, 0x07, 0xcc},
    {12.875, 0x07, 0xce},
    {13.000, 0x07, 0xd0},
    {13.125, 0x07, 0xd2},
    {13.250, 0x07, 0xd4},
    {13.375, 0x07, 0xd6},
    {13.500, 0x07, 0xd8},
    {13.625, 0x07, 0xda},
    {13.750, 0x07, 0xdc},
    {13.875, 0x07, 0xde},
    {14.000, 0x07, 0xe0},
    {14.125, 0x07, 0xe2},
    {14.250, 0x07, 0xe4},
    {14.375, 0x07, 0xe6},
    {14.500, 0x07, 0xe8},
    {14.625, 0x07, 0xea},
    {14.750, 0x07, 0xec},
    {14.875, 0x07, 0xee},
    {15.000, 0x07, 0xf0},
    {15.125, 0x07, 0xf2},
    {15.250, 0x07, 0xf4},
    {15.375, 0x07, 0xf6},
    {15.500, 0x07, 0xf8},
    {15.625, 0x07, 0xfa},
    {15.750, 0x07, 0xfc},
    {15.875, 0x07, 0xfe},
};

AX_U32 sc230ai_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gSc230aiAeRegsTable) / sizeof(gSc230aiAeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gSc230aiAeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}



AX_F32 sc230ai_again2value(float gain, AX_U8 *again_in)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!again_in)
        return -1;

    count = sizeof(sc230ai_again_table) / sizeof(sc230ai_again_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > sc230ai_again_table[i].gain) {
            continue;
        } else {
            *again_in = sc230ai_again_table[i].gain_in;
            SNS_DBG("again=%f, again_in=0x%x\n", gain, *again_in);
            return sc230ai_again_table[i].gain;
        }
    }

    return -1;
}

AX_F32 sc230ai_dgain2value(float gain, AX_U8 *dgain_in, AX_U8 *dgain_de)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!dgain_in || !dgain_de)
        return -1;

    count = sizeof(sc230ai_dgain_table) / sizeof(sc230ai_dgain_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > sc230ai_dgain_table[i].gain) {
            continue;
        } else {
            *dgain_in = sc230ai_dgain_table[i].gain_in;
            *dgain_de = sc230ai_dgain_table[i].gain_de;
            SNS_DBG("dgain=%f, dgain_in=0x%x, dgain_de=0x%x\n", gain, *dgain_in, *dgain_de);

            return sc230ai_dgain_table[i].gain;
        }
    }
    return -1;
}

AX_S32 sc230ai_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i;
    AX_S32 ret = 0;
    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(sc230ai_again_table) / sizeof(sc230ai_again_table[0]);
    params->nDgainTableSize = sizeof(sc230ai_dgain_table) / sizeof(sc230ai_dgain_table[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        nAgainTable[i] = sc230ai_again_table[i].gain;
        params->pAgainTable = nAgainTable;
    }

    for (i = 0; i < params->nDgainTableSize ; i++) {
        nDgainTable[i] = sc230ai_dgain_table[i].gain;
        params->pDgainTable = nDgainTable;
    }

    return ret;
}

AX_U32 sc230ai_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = sc230ai_reg_read(nPipeId, 0x320E);
    vts_l = sc230ai_reg_read(nPipeId, 0x320F);

    vts = (AX_U32)(((vts_h & 0x7F) << 8) | (AX_U32)((vts_l & 0xFF) << 0));

    return vts;
}

AX_U32 sc230ai_set_vts_s(ISP_PIPE_ID nPipeId, AX_U32 vts_s)
{
    AX_U8 h;
    AX_U8 l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = vts_s & 0xFF;
    h = (vts_s & 0xFF00) >> 8;

    result |= sc230ai_reg_write(nPipeId, 0x3E24, l);
    result |= sc230ai_reg_write(nPipeId, 0x3E23, h);

    return result;
}

AX_U32 sc230ai_get_vts_s(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_s_h;
    AX_U8 vts_s_l;
    AX_U32 vts_s;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_s_h = sc230ai_reg_read(nPipeId, 0x3E23);
    vts_s_l = sc230ai_reg_read(nPipeId, 0x3E24);

    vts_s = (AX_U32)(((vts_s_h & 0x7F) << 8) | (AX_U32)((vts_s_l & 0xFF) << 0));

    return vts_s;
}

AX_U32 sc230ai_get_vts_s_r1(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 l = 0;
    AX_U32 r = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    h = sc230ai_reg_read(nPipeId, 0x3206);
    l = sc230ai_reg_read(nPipeId, 0x3207);

    r = h << 8 | l;

    return r;
}

AX_U32 sc230ai_get_vts_s_r2(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 l = 0;
    AX_U32 r = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    h = sc230ai_reg_read(nPipeId, 0x3202);
    l = sc230ai_reg_read(nPipeId, 0x3203);

    r = h << 8 | l;

    return r;
}

AX_U32 sc230ai_set_int_t_l(ISP_PIPE_ID nPipeId, AX_U32 int_t_l)
{
    AX_U8 int_t_l_h;
    AX_U8 int_t_l_m;
    AX_U8 int_t_l_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    int_t_l_h = (int_t_l & 0xF000) >> 12;
    int_t_l_m = (int_t_l & 0xFF0) >> 4;
    int_t_l_l = (int_t_l & 0xF) << 4;

    result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_INT_L_0, int_t_l_h);
    result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_INT_L_1, int_t_l_m);
    result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_INT_L_2, int_t_l_l);

    return result;
}

AX_U32 sc230ai_set_int_t_s(ISP_PIPE_ID nPipeId, AX_U32 int_t_s)
{
    AX_U8 int_t_s_h;
    AX_U8 int_t_s_m;
    AX_U8 int_t_s_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    int_t_s_h = (int_t_s & 0xF000) >> 12;
    int_t_s_m = (int_t_s & 0xFF0) >> 4;
    int_t_s_l = (int_t_s & 0xF) << 4;

    result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_INT_S_0, int_t_s_h);
    result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_INT_S_1, int_t_s_m);
    result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_INT_S_2, int_t_s_l);

    return result;
}

static AX_S32 sc230ai_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
{
    FILE *fp = NULL;
    char file_name[50];
    char buf[10];

    sprintf(file_name, "/sys/class/gpio/gpio%d", gpio_num);
    SNS_DBG("file_name = %s\n", file_name);

    sprintf(file_name, "/sys/class/gpio/gpio%d/direction", gpio_num);
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    fprintf(fp, "out");
    fclose(fp);

    sprintf(file_name, "/sys/class/gpio/gpio%d/value", gpio_num);
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    if (gpio_out_val) {
        strcpy(buf, "1");
    } else {
        strcpy(buf, "0");
    }
    fprintf(fp, "%s", buf);
    fclose(fp);

    return 0;
}


/* sensor reset : only for AX620 demo/evb board */
AX_S32 sc230ai_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    FILE *fp = NULL;
    char file_name[50];
    int resetPinBase = 496;
    int nGpioNum = 0;

    nGpioNum = resetPinBase + i2cDevNum;

    strcpy(file_name, "/sys/class/gpio/export");
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    fprintf(fp, "%d", nGpioNum);
    fclose(fp);

    sc230ai_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    sc230ai_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}

AX_S32 sc230ai_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    const camera_i2c_reg_array *default_setting = NULL;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    default_setting = sc230ai_settings_table[setindex];

    while ((default_setting + reg_count)->addr != 0x0000) {
        reg_count++;
    }

    for (i = 0; i < reg_count; i++) {

        sc230ai_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);
        SNS_DBG(" addr: 0x%04x value:0x%02x \r\n", (default_setting + i)->addr, (default_setting + i)->value);

        rBuf[0] = sc230ai_reg_read(nPipeId, (default_setting + i)->addr);

        if ((rBuf[0] != (default_setting + i)->value)) {
            errnum++;
            SNS_ERR("%s  addr: 0x%04x write:0x%02x read:0x%02x \r\n", __func__,
                    (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
        }
#endif
        usleep(500);
    }

    return 0;
}

AX_U32 sc230ai_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    AX_U32 rhs1 = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gSc230aiAeRegsTable) / sizeof(gSc230aiAeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gSc230aiAeRegsTable[i].nRegValue = sc230ai_reg_read(nPipeId, gSc230aiAeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gSc230aiAeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gSc230aiAeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gSc230aiAeRegsTable[i].nRegAddr, gSc230aiAeRegsTable[i].nRegValue);
    }

    /* update linegap to table */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = sc230ai_get_vts_s(nPipeId) / 2;
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGapTime[HDR_LONG_FRAME_IDX] =
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] * sns_sc230aiparams[nPipeId].line_period + 1;

        SNS_DBG(" szLineGap %d, szLineGapTime %d\n", sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX],
                  sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGapTime[HDR_LONG_FRAME_IDX]);
    }

    return SNS_SUCCESS;
}

AX_S32 sc230ai_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
{
    AX_S32 i = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptSnsRegsInfo);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);

    if ((AX_FALSE == sns_obj->bSyncInit) || (AX_FALSE == ptSnsRegsInfo->bConfig)) {
        /* sync config */
        SNS_DBG(" bSyncInit %d, bConfig %d\n", sns_obj->bSyncInit, ptSnsRegsInfo->bConfig);
        sns_obj->sztRegsInfo[0].eSnsType = ISP_SNS_CONNECT_I2C_TYPE;
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gSc230aiBusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gSc230aiAeRegsTable) / sizeof(gSc230aiAeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = SC230AI_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gSc230aiAeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = SC230AI_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gSc230aiAeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = SC230AI_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gSc230aiAeRegsTable[i].nDelayFrmNum;
            if(IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
                sns_obj->sztRegsInfo[0].sztI2cData[i].nIntPos = AX_SNS_L_FSOF;
            } else {
                sns_obj->sztRegsInfo[0].sztI2cData[i].nIntPos = AX_SNS_S_FSOF;
            }
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gSc230aiAeRegsTable[i].nRegAddr, gSc230aiAeRegsTable[i].nRegValue);
        }

        sns_obj->bSyncInit = AX_TRUE;
    } else {
        SNS_DBG(" bSyncInit %d, bConfig %d\n", sns_obj->bSyncInit, ptSnsRegsInfo->bConfig);
        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            if (sns_obj->sztRegsInfo[0].sztI2cData[i].nData == sns_obj->sztRegsInfo[1].sztI2cData[i].nData) {
                sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_FALSE;
            } else {
                sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
                SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr, sns_obj->sztRegsInfo[0].sztI2cData[i].nData);
            }
        }
    }

    ptSnsRegsInfo->bConfig = AX_FALSE;
    memcpy(ptSnsRegsInfo, &sns_obj->sztRegsInfo[0], sizeof(AX_SNS_REGS_CFG_TABLE_T));
    /* Save the current register table */
    memcpy(&sns_obj->sztRegsInfo[1], &sns_obj->sztRegsInfo[0], sizeof(AX_SNS_REGS_CFG_TABLE_T));

    return nRet;
}


