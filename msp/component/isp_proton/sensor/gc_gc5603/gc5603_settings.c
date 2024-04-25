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
#include "ax_base_type.h"
#include "gc5603_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

static AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
static AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

extern SNS_STATE_OBJ *g_szGc5603Ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gGc5603BusInfo[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = g_szGc5603Ctx[dev])


static AX_SNS_DRV_DELAY_TABLE_T gGc5603AeRegsTable[] = {
    /*    nRegAddr         nRegValue nDelayFrmNum */
    {GC5603_LONG_EXP_LINE_H,    0,         0},
    {GC5603_LONG_EXP_LINE_L,    0,         0},

    {GC5603_LONG_AGAIN_BUF0,    0,         0},
    {GC5603_LONG_AGAIN0,        0,         0},
    {GC5603_LONG_AGAIN1,        0,         0},
    {GC5603_LONG_AGAIN2,        0,         0},
    {GC5603_LONG_AGAIN_BUF1,    0,         0},
    {GC5603_LONG_AGAIN3,        0,         0},
    {GC5603_LONG_AGAIN4,        0,         0},
    {GC5603_LONG_AGAIN5,        0,         0},
    {GC5603_LONG_AGAIN6,        0,         0},

    {GC5603_LONG_DGAIN0,        0,         0},
    {GC5603_LONG_DGAIN1,        0,         0},

    {GC5603_VTS_H,              0,         0},
    {GC5603_VTS_L,              0,         0},
};

typedef struct gc5603_again_table_s {
    float gain;
    AX_U8 again_param[7];
} gc5603_again_table_s;

/* frome GC */
const gc5603_again_table_s gc5603_again_table[] = {
               //0614, 0615, 0225, 1467  1468, 00b8, 00b9
    { 1.0,	    {0x00 ,0x00 ,0x04 ,0x19 ,0x19 ,0x01 ,0x00}},
    { 1.15625,	{0x90 ,0x02 ,0x04 ,0x1b ,0x1b ,0x01 ,0x0A}},
    { 1.28125,	{0x00 ,0x00 ,0x00 ,0x19 ,0x19 ,0x01 ,0x12}},
    { 1.5,	    {0x90 ,0x02 ,0x00 ,0x1b ,0x1b ,0x01 ,0x20}},
    { 1.75,	    {0x01 ,0x00 ,0x00 ,0x19 ,0x19 ,0x01 ,0x30}},
    { 2.078125,	{0x91 ,0x02 ,0x00 ,0x1b ,0x1b ,0x02 ,0x05}},
    { 2.390625,	{0x02 ,0x00 ,0x00 ,0x1a ,0x1a ,0x02 ,0x19}},
    { 2.984375,	{0x92 ,0x02 ,0x00 ,0x1c ,0x1c ,0x02 ,0x3F}},
    { 3.5,	    {0x03 ,0x00 ,0x00 ,0x1a ,0x1a ,0x03 ,0x20}},
    { 4.15625,	{0x93 ,0x02 ,0x00 ,0x1d ,0x1d ,0x04 ,0x0A}},
    { 5.03125,	{0x00 ,0x00 ,0x01 ,0x1d ,0x1d ,0x05 ,0x02}},
    { 5.890625,	{0x90 ,0x02 ,0x01 ,0x20 ,0x20 ,0x05 ,0x39}},
    { 6.9375,	{0x01 ,0x00 ,0x01 ,0x1e ,0x1e ,0x06 ,0x3C}},
    { 8.203125,	{0x91 ,0x02 ,0x01 ,0x20 ,0x20 ,0x08 ,0x0D}},
    { 9.515625,	{0x02 ,0x00 ,0x01 ,0x20 ,0x20 ,0x09 ,0x21}},
    { 11.234375,{0x92 ,0x02 ,0x01 ,0x22 ,0x22 ,0x0B ,0x0F}},
    { 13.359375,{0x03 ,0x00 ,0x01 ,0x21 ,0x21 ,0x0D ,0x17}},
    { 15.796875,{0x93 ,0x02 ,0x01 ,0x23 ,0x23 ,0x0F ,0x33}},
    { 18.75,	{0x04 ,0x00 ,0x01 ,0x23 ,0x23 ,0x12 ,0x30}},
    { 22.25,	{0x94 ,0x02 ,0x01 ,0x25 ,0x25 ,0x16 ,0x10}},
    { 26.390625,{0x05 ,0x00 ,0x01 ,0x24 ,0x24 ,0x1A ,0x19}},
    { 31.296875,{0x95 ,0x02 ,0x01 ,0x26 ,0x26 ,0x1F ,0x13}},
    { 37.125,	{0x06 ,0x00 ,0x01 ,0x26 ,0x26 ,0x25 ,0x08}},
    { 44.046875,{0x96 ,0x02 ,0x01 ,0x28 ,0x28 ,0x2C ,0x03}},
    { 52.234375,{0xb6 ,0x04 ,0x01 ,0x28 ,0x28 ,0x34 ,0x0F}},
    { 61.953125,{0x86 ,0x06 ,0x01 ,0x2a ,0x2a ,0x3D ,0x3D}},
    { 73.484375,{0x06 ,0x08 ,0x01 ,0x2b ,0x2b ,0x49 ,0x1f}},
};


AX_F32 gc5603_again2value(float gain, AX_U8 *again_val)
{
    AX_U32 i;
    AX_U32 count;

    count = sizeof(gc5603_again_table) / sizeof(gc5603_again_table[0]);

    for (i = 0; i < count; i++) {
        if (AXSNS_CAMPARE_FLOAT(gain, gc5603_again_table[i].gain)) {
            continue;
        } else {
            if (again_val) {
                memcpy(again_val, &gc5603_again_table[i].again_param, 7);
            }
            //printf("%s: alg gain=%f,  ret again=%f \n", __func__, gain, gc5603_again_table[i].gain);
            return gc5603_again_table[i].gain;
        }
    }

    return -1;
}

AX_F32 gc5603_value2again(AX_U8 *again_val)
{
    AX_U32 i;
    AX_U32 count;

    count = sizeof(gc5603_again_table) / sizeof(gc5603_again_table[0]);

    for (i = 0; i < count; i++) {
        if (memcmp(again_val, gc5603_again_table[i].again_param, 7) == 0) {
            return gc5603_again_table[i].gain;
        }
    }

    SNS_ERR("no correct again found\n");

    return -1;
}

/*
  for gc5603, dgain value should be 1 to 16
 */
void gc5603_dgain2value(float gain, AX_U8 *dgain_int_reg, AX_U8 *dgain_sec_reg)
{
    AX_U32 int_val;
    AX_U32 floor_c;
    AX_U8 val;
    AX_U32 ret_value = 0;

    int_val = floor(gain);
    *dgain_int_reg = int_val;
    val = (AX_U8)((gain - int_val) * 64);
    *dgain_sec_reg = (val & 0x3F) << 2;
}

AX_S32 gc5603_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i = 0;
    AX_U32 j = 0;
    AX_S32 ret = 0;
    int counter = 0;
    int d_min = 0;

    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(gc5603_again_table) / sizeof(gc5603_again_table[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        nAgainTable[i] = gc5603_again_table[i].gain;
        params->pAgainTable = nAgainTable;
    }

    for (i = 1; i < 16 ; i++) {
        for (j = 0; j < 64 ; j++) {
            if (counter > (SENSOR_MAX_GAIN_STEP - 1)) {
                SNS_ERR("%s: sensor Dgain count overflow\n", __func__);
                break;
            }
            nDgainTable[counter] = i + (j * 0.015625);
            counter ++;
        }
    }
    params->nDgainTableSize = counter;
    params->pDgainTable = nDgainTable;

    return ret;
}


AX_S32 gc5603_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
{
    FILE *fp = NULL;
    char file_name[50];
    char buf[10];

    sprintf(file_name, "/sys/class/gpio/export");
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    fprintf(fp, "%d", gpio_num);
    fclose(fp);

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

AX_S32 gc5603_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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

    gc5603_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    gc5603_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}


AX_S32 gc5603_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    const camera_i2c_reg_array *default_setting = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("gc5603 setitng index: %d\n", setindex);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    switch (setindex) {
    case MODE_GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_30FPS:
        default_setting = &GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_30FPS[0];
        reg_count = sizeof(GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    case MODE_GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_25FPS:
        default_setting = &GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_25FPS[0];
        reg_count = sizeof(GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case MODE_GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_20FPS:
        default_setting = &GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_15FPS[0];
        reg_count = sizeof(GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_15FPS) / sizeof(camera_i2c_reg_array);
        break;

    case MODE_GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_15FPS:
        default_setting = &GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_15FPS[0];
        reg_count = sizeof(GC5603_2Lane_2944_1664_846Mbps_27MHz_LINEAR_RGGB_10BIT_15FPS) / sizeof(camera_i2c_reg_array);
        break;

    case MODE_GC5603_2Lane_2944_1664_846Mbps_27MHz_2STAGGER_HDR_RGGB_10BIT_15FPS:
        default_setting = &GC5603_2Lane_2944_1664_846Mbps_27MHz_2STAGGER_HDR_RGGB_10BIT_15FPS[0];
        reg_count = sizeof(GC5603_2Lane_2944_1664_846Mbps_27MHz_2STAGGER_HDR_RGGB_10BIT_15FPS) / sizeof(camera_i2c_reg_array);
        break;

    default:
        SNS_ERR("it's not supported. pipe=%d, setting mode=%d] \n", nPipeId, setindex);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("setindex:%d, reg_count:%d\n", setindex, reg_count);

    for (i = 0; i < reg_count; i++) {
        gc5603_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);

        rBuf[0] = gc5603_reg_read(nPipeId, (default_setting + i)->addr);
        SNS_DBG(" addr: 0x%04x write:0x%02x read:0x%02x \r\n",
                (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
#endif
    }
    usleep(80000);
    return 0;
}

AX_U32 gc5603_get_hts(ISP_PIPE_ID nPipeId)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_U32 hts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts_l = gc5603_reg_read(nPipeId, GC5603_HTS_L);
    hts_h = gc5603_reg_read(nPipeId, GC5603_HTS_H);

    hts = (AX_U32)(((hts_h & 0xF) << 8) | (AX_U32)(hts_l << 0));
    hts = hts * 2;

    return hts;
}

AX_U32 gc5603_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = gc5603_reg_read(nPipeId, GC5603_VTS_H);
    vts_l = gc5603_reg_read(nPipeId, GC5603_VTS_L);

    vts = (AX_U32)(((vts_h & 0xFF) << 8) | (AX_U32)(vts_l << 0));

    return vts;
}

AX_U32 gc5603_set_hts(ISP_PIPE_ID nPipeId, AX_U32 hts)
{
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts = hts / 2;
    result |= gc5603_sns_update_regs_table(nPipeId, GC5603_HTS_H_IDX, (hts >> 8) & 0xF);
    result |= gc5603_sns_update_regs_table(nPipeId, GC5603_HTS_L_IDX, hts & 0xFF);

    return result;
}

AX_U32 gc5603_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts)
{
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result |= gc5603_sns_update_regs_table(nPipeId, GC5603_VTS_H_IDX, (vts >> 8) & 0xFF);
    result |= gc5603_sns_update_regs_table(nPipeId, GC5603_VTS_L_IDX, vts & 0xFF);

    return result;
}

AX_U32 gc5603_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U8 nRegIdx, AX_U8 nRegValue)
{
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    nNum = sizeof(gGc5603AeRegsTable) / sizeof(gGc5603AeRegsTable[0]);
    SNS_CHECK_VALUE_RANGE_VALID(nRegIdx, 0, nNum - 1);

    sns_obj->sztRegsInfo[0].sztI2cData[nRegIdx].nData = nRegValue;
    gGc5603AeRegsTable[nRegIdx].nRegValue = nRegValue;
    sns_obj->sztRegsInfo[0].sztI2cData[nRegIdx].bUpdate = AX_TRUE;

    SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n",
        nRegIdx, sns_obj->sztRegsInfo[0].sztI2cData[nRegIdx].nRegAddr, nRegValue);

    return SNS_SUCCESS;
}

AX_U32 gc5603_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    nNum = sizeof(gGc5603AeRegsTable) / sizeof(gGc5603AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gGc5603AeRegsTable[i].nRegValue = gc5603_reg_read(nPipeId, gGc5603AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gGc5603AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gGc5603AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gGc5603AeRegsTable[i].nRegAddr, gGc5603AeRegsTable[i].nRegValue);
    }

    return SNS_SUCCESS;
}

AX_S32 gc5603_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
{
    AX_S32 i = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptSnsRegsInfo);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if ((AX_FALSE == sns_obj->bSyncInit) || (AX_FALSE == ptSnsRegsInfo->bConfig)) {
        /* sync config */
        SNS_DBG(" bSyncInit %d, bConfig %d\n", sns_obj->bSyncInit, ptSnsRegsInfo->bConfig);
        sns_obj->sztRegsInfo[0].eSnsType = ISP_SNS_CONNECT_I2C_TYPE;
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gGc5603BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gGc5603AeRegsTable) / sizeof(gGc5603AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = GC5603_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gGc5603AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = GC5603_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gGc5603AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = GC5603_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gGc5603AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gGc5603AeRegsTable[i].nRegAddr, gGc5603AeRegsTable[i].nRegValue);
        }

        sns_obj->bSyncInit = AX_TRUE;
    }

    ptSnsRegsInfo->bConfig = AX_FALSE;
    memcpy(ptSnsRegsInfo, &sns_obj->sztRegsInfo[0], sizeof(AX_SNS_REGS_CFG_TABLE_T));
    /* Save the current register table */
    memcpy(&sns_obj->sztRegsInfo[1], &sns_obj->sztRegsInfo[0], sizeof(AX_SNS_REGS_CFG_TABLE_T));

    for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
        sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_FALSE;
    }

    return nRet;
}



