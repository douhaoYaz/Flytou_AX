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
#include "i2c.h"
#include "ax_base_type.h"
#include "sc530ai_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern AX_SNS_COMMBUS_T g_SC530AIBusInfo[DEF_VIN_PIPE_MAX_NUM];
extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])

static AX_SNS_DRV_DELAY_TABLE_T gSC530AIAeRegsTable[] = {
    /* nRegAddr */          /*regs value*/  /*Delay Frame Num*/
    {SC530AI_INT_L_H,              0,             0},
    {SC530AI_INT_L_M,              0,             0},
    {SC530AI_INT_L_L,              0,             0},
    {SC530AI_INT_S_H,              0,             0},
    {SC530AI_INT_S_M,              0,             0},
    {SC530AI_INT_S_L,              0,             0},
    {SC530AI_LONG_AGAIN,           0,             0},
    {SC530AI_LONG_DGAIN_H,         0,             0},
    {SC530AI_LONG_DGAIN_L,         0,             0},
    {SC530AI_SHORT_AGAIN,          0,             0},
    {SC530AI_SHORT_DGAIN_H,        0,             0},
    {SC530AI_SHORT_DGAIN_L,        0,             0},
    {SC530AI_VTS_L_H,              0,             0},
    {SC530AI_VTS_L_L,              0,             0},
    {SC530AI_VTS_S_H,              0,             0},
    {SC530AI_VTS_S_L,              0,             0},
};

int gain_compare(float a, float b)
{
    float Express = 0.000001;

    if (a - b > Express)
        return 1;
    else
        return 0;
}

static AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
static AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

typedef struct _sc530ai_GAIN_TABLE_T_ {
    float gain;
    AX_U8 gain_in;
    AX_U8 gain_de;
} sc530ai_GAIN_TABLE_T;

const sc530ai_GAIN_TABLE_T sc530ai_again_table[] = {
    {1.000, 0x00},
    {2.000, 0x01},
    {2.390, 0x40},
    {4.780, 0x48},
    {9.560, 0x49},
    {19.12, 0x4B},
    {38.24, 0x4F},
    {76.48, 0x5F},
};

const sc530ai_GAIN_TABLE_T sc530ai_dgain_table[] = {
    {1.000, 0x00, 0x80},
    {1.031, 0x00, 0x84},
    {1.063, 0x00, 0x88},
    {1.094, 0x00, 0x8c},
    {1.125, 0x00, 0x90},
    {1.156, 0x00, 0x94},
    {1.188, 0x00, 0x98},
    {1.219, 0x00, 0x9c},
    {1.250, 0x00, 0xa0},
    {1.281, 0x00, 0xa4},
    {1.313, 0x00, 0xa8},
    {1.344, 0x00, 0xac},
    {1.375, 0x00, 0xb0},
    {1.406, 0x00, 0xb4},
    {1.438, 0x00, 0xb8},
    {1.469, 0x00, 0xbc},
    {1.500, 0x00, 0xc0},
    {1.531, 0x00, 0xc4},
    {1.563, 0x00, 0xc8},
    {1.594, 0x00, 0xcc},
    {1.625, 0x00, 0xd0},
    {1.656, 0x00, 0xd4},
    {1.688, 0x00, 0xd8},
    {1.719, 0x00, 0xdc},
    {1.750, 0x00, 0xe0},
    {1.781, 0x00, 0xe4},
    {1.813, 0x00, 0xe8},
    {1.844, 0x00, 0xec},
    {1.875, 0x00, 0xf0},
    {1.906, 0x00, 0xf4},
    {1.938, 0x00, 0xf8},
    {1.969, 0x00, 0xfc},
    {2.000, 0x01, 0x80},
    {2.063, 0x01, 0x84},
    {2.125, 0x01, 0x88},
    {2.188, 0x01, 0x8c},
    {2.250, 0x01, 0x90},
    {2.313, 0x01, 0x94},
    {2.375, 0x01, 0x98},
    {2.438, 0x01, 0x9c},
    {2.500, 0x01, 0xa0},
    {2.563, 0x01, 0xa4},
    {2.625, 0x01, 0xa8},
    {2.688, 0x01, 0xac},
    {2.750, 0x01, 0xb0},
    {2.813, 0x01, 0xb4},
    {2.875, 0x01, 0xb8},
    {2.938, 0x01, 0xbc},
    {3.000, 0x01, 0xc0},
    {3.063, 0x01, 0xc4},
    {3.125, 0x01, 0xc8},
    {3.188, 0x01, 0xcc},
    {3.250, 0x01, 0xd0},
    {3.313, 0x01, 0xd4},
    {3.375, 0x01, 0xd8},
    {3.438, 0x01, 0xdc},
    {3.500, 0x01, 0xe0},
    {3.563, 0x01, 0xe4},
    {3.625, 0x01, 0xe8},
    {3.688, 0x01, 0xec},
    {3.750, 0x01, 0xf0},
    {3.813, 0x01, 0xf4},
    {3.875, 0x01, 0xf8},
    {3.938, 0x01, 0xfc},

};

AX_F32 sc530ai_again2value(float gain, AX_U8 *again_in)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!again_in)
        return -1;

    count = sizeof(sc530ai_again_table) / sizeof(sc530ai_again_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > sc530ai_again_table[i].gain) {
            continue;
        } else {
            *again_in = sc530ai_again_table[i].gain_in;
            SNS_DBG("again=%f, again_in=0x%x\n", gain, *again_in);
            return sc530ai_again_table[i].gain;
        }
    }

    return -1;
}

AX_F32 sc530ai_dgain2value(float gain, AX_U8 *dgain_in, AX_U8 *dgain_de)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!dgain_in || !dgain_de)
        return -1;

    count = sizeof(sc530ai_dgain_table) / sizeof(sc530ai_dgain_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > sc530ai_dgain_table[i].gain) {
            continue;
        } else {
            *dgain_in = sc530ai_dgain_table[i].gain_in;
            *dgain_de = sc530ai_dgain_table[i].gain_de;
            SNS_DBG("dgain=%f, dgain_in=0x%x, dgain_de=0x%x\n", gain, *dgain_in, *dgain_de);

            return sc530ai_dgain_table[i].gain;
        }
    }
    return -1;
}

AX_S32 sc530ai_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i;
    AX_S32 ret = 0;
    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(sc530ai_again_table) / sizeof(sc530ai_again_table[0]);
    params->nDgainTableSize = sizeof(sc530ai_dgain_table) / sizeof(sc530ai_dgain_table[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        nAgainTable[i] = sc530ai_again_table[i].gain;
        params->pAgainTable = nAgainTable;
    }

    for (i = 0; i < params->nDgainTableSize ; i++) {
        nDgainTable[i] = sc530ai_dgain_table[i].gain;
        params->pDgainTable = nDgainTable;
    }

    return ret;
}

AX_S32 sc530ai_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    const camera_i2c_reg_array *default_setting = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("sc530ai setitng index: %d\n", setindex);

    default_setting = sc530ai_settings_table[setindex];

    while ((default_setting + reg_count)->addr != 0x0000) {
        reg_count++;
    }

    SNS_DBG("sc530ai setitng index: %d, reg_count %d\n", setindex, reg_count);
    for (i = 0; i < reg_count; i++) {
        sc530ai_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);

        rBuf[0] = sc530ai_reg_read(nPipeId, (default_setting + i)->addr);
        SNS_DBG(" addr: 0x%04x write:0x%02x read:0x%02x \r\n",
                (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
#endif
        usleep(500);
    }

    return 0;
}

AX_U32 sc530ai_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = sc530ai_reg_read(nPipeId, SC530AI_VTS_L_H);
    vts_l = sc530ai_reg_read(nPipeId, SC530AI_VTS_L_L);

    vts = (AX_U32)(((vts_h & 0x7F) << 8) | (AX_U32)((vts_l & 0xFF) << 0));

    return vts;
}

AX_U32 sc530ai_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_l = vts & 0xFF;
    vts_h = (vts & 0x7F00) >> 8;

    result |= sc530ai_reg_write(0, SC530AI_VTS_L_H, vts_h);
    result |= sc530ai_reg_write(0, SC530AI_VTS_L_L, vts_l);

    return result;
}

AX_U32 sc530ai_get_vts_s(ISP_PIPE_ID nPipeId)
{
    AX_U32 vts_s_h = 0;
    AX_U32 vts_s_l = 0;
    AX_U32 vts_s = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_s_h = sc530ai_reg_read(nPipeId, SC530AI_VTS_S_H);
    vts_s_l = sc530ai_reg_read(nPipeId, SC530AI_VTS_S_L);

    vts_s = (AX_U32)(((vts_s_h & 0x00FF) << 8) | (AX_U32)((vts_s_l & 0xFF) << 0));

    return vts_s;
}

AX_U32 sc530ai_set_vts_s(ISP_PIPE_ID nPipeId, AX_U32 vts_s)
{
    AX_U8 h;
    AX_U8 l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = vts_s & 0xFF;
    h = (vts_s & 0xFF00) >> 8;

    result |= sc530ai_reg_write(nPipeId, SC530AI_VTS_S_L, l);
    result |= sc530ai_reg_write(nPipeId, SC530AI_VTS_S_H, h);

    return result;
}

AX_U32 sc530ai_get_vts_s_r1(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 l = 0;
    AX_U32 r = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    h = sc530ai_reg_read(nPipeId, 0x3206);
    l = sc530ai_reg_read(nPipeId, 0x3207);

    r = h << 8 | l;

    return r;
}

AX_U32 sc530ai_get_vts_s_r2(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 l = 0;
    AX_U32 r = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    h = sc530ai_reg_read(nPipeId, 0x3202);
    l = sc530ai_reg_read(nPipeId, 0x3203);

    r = h << 8 | l;

    return r;
}

AX_U32 sc530ai_set_int_t_l(ISP_PIPE_ID nPipeId, AX_U32 int_t_l)
{
    AX_U8 int_t_l_h;
    AX_U8 int_t_l_m;
    AX_U8 int_t_l_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    int_t_l_h = (int_t_l & 0xF000) >> 12;
    int_t_l_m = (int_t_l & 0xFF0) >> 4;
    int_t_l_l = (int_t_l & 0xF) << 4;

    result |= sc530ai_sns_update_regs_table(nPipeId, SC530AI_INT_L_H, int_t_l_h);
    result |= sc530ai_sns_update_regs_table(nPipeId, SC530AI_INT_L_M, int_t_l_m);
    result |= sc530ai_sns_update_regs_table(nPipeId, SC530AI_INT_L_L, int_t_l_l);

    return result;
}

AX_U32 sc530ai_set_int_t_s(ISP_PIPE_ID nPipeId, AX_U32 int_t_s)
{
    AX_U8 int_t_s_h;
    AX_U8 int_t_s_m;
    AX_U8 int_t_s_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    int_t_s_h = (int_t_s & 0xF000) >> 12;
    int_t_s_m = (int_t_s & 0x0FF0) >> 4;
    int_t_s_l = (int_t_s & 0x000F) << 4;

    result |= sc530ai_sns_update_regs_table(nPipeId, SC530AI_INT_S_H, int_t_s_h);
    result |= sc530ai_sns_update_regs_table(nPipeId, SC530AI_INT_S_M, int_t_s_m);
    result |= sc530ai_sns_update_regs_table(nPipeId, SC530AI_INT_S_L, int_t_s_l);

    return result;
}

AX_S32 sc530ai_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
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
AX_S32 sc530ai_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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


    sc530ai_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    sc530ai_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}

AX_U32 sc530ai_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    if (AX_FALSE == sns_obj->bSyncInit) {
        return SNS_SUCCESS;
    }

    nNum = sizeof(gSC530AIAeRegsTable) / sizeof(gSC530AIAeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}

AX_U32 sc530ai_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gSC530AIAeRegsTable) / sizeof(gSC530AIAeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gSC530AIAeRegsTable[i].nRegValue = sc530ai_reg_read(nPipeId, gSC530AIAeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gSC530AIAeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gSC530AIAeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gSC530AIAeRegsTable[i].nRegAddr, gSC530AIAeRegsTable[i].nRegValue);
    }

    /* linegap default vaule */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = sc530ai_get_vts_s(nPipeId) / 2;
        SNS_DBG("init szLineGap 0x%x\n", sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX]);
    }

    return SNS_SUCCESS;
}

AX_S32 sc530ai_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = g_SC530AIBusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gSC530AIAeRegsTable) / sizeof(gSC530AIAeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = SC530AI_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gSC530AIAeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = SC530AI_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gSC530AIAeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = SC530AI_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gSC530AIAeRegsTable[i].nDelayFrmNum;
            if(IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
                sns_obj->sztRegsInfo[0].sztI2cData[i].nIntPos = AX_SNS_L_FSOF;
            } else {
                sns_obj->sztRegsInfo[0].sztI2cData[i].nIntPos = AX_SNS_S_FSOF;
            }
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gSC530AIAeRegsTable[i].nRegAddr, gSC530AIAeRegsTable[i].nRegValue);
        }

        sns_obj->bSyncInit = AX_TRUE;
    } else {
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
