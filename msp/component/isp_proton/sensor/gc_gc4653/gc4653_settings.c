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
#include "gc4653_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

static AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
static AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gGc4653BusInfo[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])


static AX_SNS_DRV_DELAY_TABLE_T gGc4653AeRegsTable[] = {
    /* nRegAddr */          /*regs value*/  /*Delay Frame Num*/
    {GC4653_LONG_EXP_LINE_H,        0,          0},
    {GC4653_LONG_EXP_LINE_L,        0,          0},

    {GC4653_LONG_AGAIN0,            0,          0},
    {GC4653_LONG_AGAIN1,            0,          0},
    {GC4653_LONG_AGAIN2,            0,          0},
    {GC4653_LONG_AGAIN3,            0,          0},
    {GC4653_LONG_AGAIN4,            0,          0},
    {GC4653_LONG_AGAIN5,            0,          0},
    {GC4653_LONG_AGAIN6,            0,          0},

    {GC4653_LONG_DGAIN0,            0,          0},
    {GC4653_LONG_DGAIN1,            0,          0},
};

typedef struct gc4653_again_table_s {
    float gain;
    AX_U8 again_param[7];
} gc4653_again_table_s;


const gc4653_again_table_s gc4653_again_table[] = {
              /* 2b3    2b4    2b8    2b9    515    519    2d9 */
    { 1.0,      {0x00,  0x00,  0x01,  0x00,  0x30,  0x1e,  0x5C}},
    { 1.171,    {0x20,  0x00,  0x01,  0x0B,  0x30,  0x1e,  0x5C}},
    { 1.39,     {0x01,  0x00,  0x01,  0x19,  0x30,  0x1d,  0x5B}},
    { 1.656,    {0x21,  0x00,  0x01,  0x2A,  0x30,  0x1e,  0x5C}},
    { 2.0,      {0x02,  0x00,  0x02,  0x00,  0x30,  0x1e,  0x5C}},
    { 2.359,    {0x22,  0x00,  0x02,  0x17,  0x30,  0x1d,  0x5B}},
    { 2.797,    {0x03,  0x00,  0x02,  0x33,  0x20,  0x16,  0x54}},
    { 3.3125,   {0x23,  0x00,  0x03,  0x14,  0x20,  0x17,  0x55}},
    { 4.0,      {0x04,  0x00,  0x04,  0x00,  0x20,  0x17,  0x55}},
    { 4.7343,   {0x24,  0x00,  0x04,  0x2F,  0x20,  0x19,  0x57}},
    { 5.5937,   {0x05,  0x00,  0x05,  0x26,  0x20,  0x19,  0x57}},
    { 6.625,    {0x25,  0x00,  0x06,  0x28,  0x20,  0x1b,  0x59}},
    { 8.0,      {0x0c,  0x00,  0x08,  0x00,  0x20,  0x1d,  0x5B}},
    { 9.4687,   {0x2C,  0x00,  0x09,  0x1E,  0x20,  0x1f,  0x5D}},
    { 11.1875,  {0x0D,  0x00,  0x0B,  0x0C,  0x20,  0x21,  0x5F}},
    { 13.266,   {0x2D,  0x00,  0x0D,  0x11,  0x20,  0x24,  0x62}},
    { 16.0,     {0x1C,  0x00,  0x10,  0x00,  0x20,  0x26,  0x64}},
    { 18.953,   {0x3C,  0x00,  0x12,  0x3D,  0x18,  0x2a,  0x68}},
    { 22.391,   {0x5C,  0x00,  0x16,  0x19,  0x18,  0x2c,  0x6A}},
    { 26.531,   {0x7C,  0x00,  0x1A,  0x22,  0x18,  0x2e,  0x6C}},
    { 32.0,     {0x9C,  0x00,  0x20,  0x00,  0x18,  0x32,  0x70}},
    { 37.906,   {0xBC,  0x00,  0x25,  0x3A,  0x18,  0x35,  0x73}},
    { 44.797,   {0xDC,  0x00,  0x2C,  0x33,  0x10,  0x36,  0x74}},
    { 53.078,   {0xFC,  0x00,  0x35,  0x05,  0x10,  0x38,  0x76}},
    { 64.0,     {0x1C,  0x01,  0x40,  0x00,  0x10,  0x3c,  0x7A}},
    { 75.828,   {0x3C,  0x01,  0x4B,  0x35,  0x10,  0x42,  0x80}},
};


AX_F32 gc4653_again2value(float gain, AX_U8 *again_val)
{
    AX_U32 i;
    AX_U32 count;

    count = sizeof(gc4653_again_table) / sizeof(gc4653_again_table[0]);

    for (i = 0; i < count; i++) {
        if (AXSNS_CAMPARE_FLOAT(gain, gc4653_again_table[i].gain)) {
            continue;
        } else {
            if (again_val) {
                memcpy(again_val, &gc4653_again_table[i].again_param, 7);
            }
            return gc4653_again_table[i].gain;
        }
    }

    return -1;
}

/* for gc4653, dgain value should be 1 to 16 */
void gc4653_dgain2value(float gain, AX_U8 *dgain_int_reg, AX_U8 *dgain_sec_reg)
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

AX_S32 gc4653_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i = 0;
    AX_U32 j = 0;
    AX_S32 ret = 0;
    int counter = 0;
    int d_min = 0;

    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(gc4653_again_table) / sizeof(gc4653_again_table[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        nAgainTable[i] = gc4653_again_table[i].gain;
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


AX_S32 gc4653_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
{
    FILE *fp = NULL;
    char file_name[50];
    char buf[10];

    sprintf(file_name, "/sys/class/gpio/gpio%d", gpio_num);
    if (0 != access(file_name, F_OK)) {
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
    }

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
AX_S32 gc4653_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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


    gc4653_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    gc4653_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}



AX_S32 gc4653_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    const camera_i2c_reg_array *default_setting = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("gc4653 setitng index: %d\n", setindex);

    default_setting = gc4653_settings_table[setindex];

    while ((default_setting + reg_count)->addr != 0x0000) {
        reg_count++;
    }

    SNS_DBG("gc4653 setitng index: %d, reg_count %d\n", setindex, reg_count);
    for (i = 0; i < reg_count; i++) {
        gc4653_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);

        rBuf[0] = gc4653_reg_read(nPipeId, (default_setting + i)->addr);
        SNS_DBG(" addr: 0x%04x write:0x%02x read:0x%02x \r\n",
                (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
#endif
        usleep(500);
    }

    return 0;
}

AX_U32 gc4653_get_hts(ISP_PIPE_ID nPipeId)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_U32 hts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts_h = gc4653_reg_read(nPipeId, 0x342);
    hts_l = gc4653_reg_read(nPipeId, 0x343);

    hts = (AX_U32)(((hts_h & 0xF) << 8) | (AX_U32)(hts_l << 0));
    hts = hts * 2;

    return hts;
}

AX_U32 gc4653_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = gc4653_reg_read(nPipeId, 0x340);
    vts_l = gc4653_reg_read(nPipeId, 0x341);

    vts = (AX_U32)(((vts_h & 0xF) << 8) | (AX_U32)(vts_l << 0));

    return vts;
}

AX_U32 gc4653_set_hts(ISP_PIPE_ID nPipeId, AX_U32 hts)
{
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts = hts / 2;
    result |= gc4653_reg_write(nPipeId, 0x342, (hts >> 8) & 0xF);
    result |= gc4653_reg_write(nPipeId, 0x343, hts & 0xFF);

    return result;
}

AX_U32 gc4653_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts)
{
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result |= gc4653_reg_write(nPipeId, 0x340, (vts >> 8) & 0x3F);
    result |= gc4653_reg_write(nPipeId, 0x341, vts & 0xFF);

    return result;
}

AX_U32 gc4653_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gGc4653AeRegsTable) / sizeof(gGc4653AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gGc4653AeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}

AX_U32 gc4653_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gGc4653AeRegsTable) / sizeof(gGc4653AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gGc4653AeRegsTable[i].nRegValue = gc4653_reg_read(nPipeId, gGc4653AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gGc4653AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gGc4653AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gGc4653AeRegsTable[i].nRegAddr, gGc4653AeRegsTable[i].nRegValue);
    }

    return SNS_SUCCESS;
}

AX_S32 gc4653_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gGc4653BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gGc4653AeRegsTable) / sizeof(gGc4653AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = GC4653_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gGc4653AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = GC4653_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gGc4653AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = GC4653_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gGc4653AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gGc4653AeRegsTable[i].nRegAddr, gGc4653AeRegsTable[i].nRegValue);
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



