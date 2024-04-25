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
#include "imx327_settings.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern AX_SNS_COMMBUS_T g_imx327BusInfo[DEF_VIN_PIPE_MAX_NUM];
extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])

static AX_SNS_DRV_DELAY_TABLE_T gImx327AeRegsTable[] = {

    /* nRegAddr */       /* regs value */    /* Delay Frame Number */
    {IMX327_SHS1_L,             0,                  0},
    {IMX327_SHS1_M,             0,                  0},
    {IMX327_SHS1_H,             0,                  0},
    {IMX327_SHS2_L,             0,                  0},
    {IMX327_SHS2_M,             0,                  0},
    {IMX327_SHS2_H,             0,                  0},

    {IMX327_RHS1_L,             0,                  0},
    {IMX327_RHS1_M,             0,                  0},
    {IMX327_RHS1_H,             0,                  0},

    {IMX327_GAIN,               0,                  0},
    {IMX327_GAIN1,              0,                  0},

    {IMX327_FDG_SEL,            0,                  1},
};

AX_U32 imx327_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gImx327AeRegsTable) / sizeof(gImx327AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            gImx327AeRegsTable[i].nRegValue = nRegsValue;
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

AX_U32 imx327_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;

    nNum = sizeof(gImx327AeRegsTable) / sizeof(gImx327AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gImx327AeRegsTable[i].nRegValue = imx327_reg_read(nPipeId, gImx327AeRegsTable[i].nRegAddr);
        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gImx327AeRegsTable[i].nRegAddr, gImx327AeRegsTable[i].nRegValue);
    }

    return SNS_SUCCESS;
}


AX_U32 imx327_get_shs1(ISP_PIPE_ID nPipeId)
{
    AX_U32 shs1_h;
    AX_U32 shs1_m;
    AX_U32 shs1_l;
    AX_U32 shs1;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shs1_l = imx327_reg_read(nPipeId, IMX327_SHS1_L);
    shs1_m = imx327_reg_read(nPipeId, IMX327_SHS1_M);
    shs1_h = imx327_reg_read(nPipeId, IMX327_SHS1_H);

    shs1 = (shs1_h & 0xF) << 16 | shs1_m << 8 | shs1_l;

    SNS_DBG("get shs1: 0x%x (0x%x, 0x%x, 0x%x)\n", shs1, shs1_h, shs1_m, shs1_l);

    return shs1;
}

AX_S32 imx327_set_shs1(ISP_PIPE_ID nPipeId, AX_U32 shs1)
{
    AX_U8 shs1_h;
    AX_U8 shs1_m;
    AX_U8 shs1_l;

    AX_S32 result = 0;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shs1_l = shs1 & 0xFF;
    shs1_m = (shs1 & 0xFF00) >> 8;
    shs1_h = (shs1 & 0xF0000) >> 16;

    result |= imx327_sns_update_regs_table(nPipeId, IMX327_SHS1_L, shs1_l);
    result |= imx327_sns_update_regs_table(nPipeId, IMX327_SHS1_M, shs1_m);
    result |= imx327_sns_update_regs_table(nPipeId, IMX327_SHS1_H, shs1_h);

    SNS_DBG("set shs1: 0x%x (0x%x, 0x%x, 0x%x)\n", shs1, shs1_h, shs1_m, shs1_l);

    return result;
}

AX_U32 imx327_get_shs2(ISP_PIPE_ID nPipeId)
{
    AX_U32 shs2_h;
    AX_U32 shs2_m;
    AX_U32 shs2_l;
    AX_U32 shs2;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shs2_l = imx327_reg_read(nPipeId, IMX327_SHS2_L);
    shs2_m = imx327_reg_read(nPipeId, IMX327_SHS2_M);
    shs2_h = imx327_reg_read(nPipeId, IMX327_SHS2_H);

    shs2 = (shs2_h & 0xF) << 16 | shs2_m << 8 | shs2_l;

    SNS_DBG("get shs2: 0x%x (0x%x, 0x%x, 0x%x)\n", shs2, shs2_h, shs2_m, shs2_l);

    return shs2;
}

AX_S32 imx327_set_shs2(ISP_PIPE_ID nPipeId, AX_U32 shs2)
{
    AX_U8 shs2_h;
    AX_U8 shs2_m;
    AX_U8 shs2_l;

    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shs2_l = shs2 & 0xFF;
    shs2_m = (shs2 & 0xFF00) >> 8;
    shs2_h = (shs2 & 0xF0000) >> 16;

    result |= imx327_sns_update_regs_table(nPipeId, IMX327_SHS2_L, shs2_l);
    result |= imx327_sns_update_regs_table(nPipeId, IMX327_SHS2_M, shs2_m);
    result |= imx327_sns_update_regs_table(nPipeId, IMX327_SHS2_H, shs2_h);

    SNS_DBG("set shs2: 0x%x (0x%x, 0x%x, 0x%x)\n", shs2, shs2_h, shs2_m, shs2_l);

    return result;
}



AX_U32 imx327_get_rhs1(ISP_PIPE_ID nPipeId)
{
    AX_U32 rhs1_h;
    AX_U32 rhs1_m;
    AX_U32 rhs1_l;
    AX_U32 rhs1;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    rhs1_l = imx327_reg_read(nPipeId, IMX327_RHS1_L);
    rhs1_m = imx327_reg_read(nPipeId, IMX327_RHS1_M);
    rhs1_h = imx327_reg_read(nPipeId, IMX327_RHS1_H);

    rhs1 = (rhs1_h & 0xF) << 16 | rhs1_m << 8 | rhs1_l;

    SNS_DBG("get rhs1: 0x%x (0x%x, 0x%x, 0x%x)\n", rhs1, rhs1_h, rhs1_m, rhs1_l);

    return rhs1;
}

AX_S32 imx327_set_rhs1(ISP_PIPE_ID nPipeId, AX_U32 rhs1)
{
    AX_U8 rhs1_h;
    AX_U8 rhs1_m;
    AX_U8 rhs1_l;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    rhs1_l = rhs1 & 0xFF;
    rhs1_m = (rhs1 & 0xFF00) >> 8;
    rhs1_h = (rhs1 & 0xF0000) >> 16;

    result |= imx327_sns_update_regs_table(nPipeId, IMX327_RHS1_L, rhs1_l);
    result |= imx327_sns_update_regs_table(nPipeId, IMX327_RHS1_M, rhs1_m);
    result |= imx327_sns_update_regs_table(nPipeId, IMX327_RHS1_H, rhs1_h);

    SNS_DBG("set rhs1: 0x%x (0x%x, 0x%x, 0x%x)\n", rhs1, rhs1_h, rhs1_m, rhs1_l);

    return result;
}

AX_U32 imx327_get_vmax(ISP_PIPE_ID nPipeId)
{
    AX_U32 vmax_h;
    AX_U32 vmax_m;
    AX_U32 vmax_l;

    AX_U32 vmax;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vmax_l = imx327_reg_read(nPipeId, IMX327_VMAX_L);
    vmax_m = imx327_reg_read(nPipeId, IMX327_VMAX_M);
    vmax_h = imx327_reg_read(nPipeId, IMX327_VMAX_H);

    vmax = (vmax_h & 0x3) << 16 | vmax_m << 8 | vmax_l;

    SNS_DBG("get vmax: 0x%x (0x%x, 0x%x, 0x%x)\n", vmax, vmax_h, vmax_m, vmax_l);

    return vmax;
}

AX_U32 imx327_get_hmax(ISP_PIPE_ID nPipeId)
{
    AX_U32 hmax_h;
    AX_U32 hmax_l;
    AX_U32 hmax;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hmax_l = imx327_reg_read(nPipeId, IMX327_HMAX_L);
    hmax_h = imx327_reg_read(nPipeId, IMX327_HMAX_H);

    hmax = hmax_h << 8 | hmax_l;

    SNS_DBG("get hmax: 0x%x (0x%x, 0x%x)\n", hmax, hmax_h, hmax_l);

    return hmax;
}

AX_U32 imx327_set_vmax(ISP_PIPE_ID nPipeId, AX_U32 vmax)
{
    AX_U8 vmax_h;
    AX_U8 vmax_m;
    AX_U8 vmax_l;
    AX_U8 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vmax_l = vmax & 0xFF;
    vmax_m = (vmax & 0xFF00) >>8;
    vmax_h = (vmax & 0x30000)>>16;

    result |= imx327_reg_write(nPipeId, IMX327_VMAX_L, vmax_l);
    result |= imx327_reg_write(nPipeId, IMX327_VMAX_M, vmax_m);
    result |= imx327_reg_write(nPipeId, IMX327_VMAX_H, vmax_h);

    SNS_DBG("set vmax: 0x%x (0x%x, 0x%x, 0x%x)\n", vmax, vmax_h, vmax_m, vmax_l);

    return 0;
}


AX_S32 imx327_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
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

AX_S32 imx327_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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

    imx327_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    imx327_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}


AX_S32 imx327_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    const camera_i2c_reg_array *default_setting = NULL;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    default_setting = imx327_settings_table[setindex];

    while ((default_setting + reg_count)->addr != 0x0000) {
        reg_count++;
    }
    printf("reg_count: %d\n", reg_count);
    for (i = 0; i < reg_count; i++) {

        imx327_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);
        SNS_DBG(" addr: 0x%04x value:0x%02x \r\n", (default_setting + i)->addr, (default_setting + i)->value);

        rBuf[0] = imx327_reg_read(nPipeId, (default_setting + i)->addr);

        if ((rBuf[0] != (default_setting + i)->value)) {
            errnum++;
            SNS_ERR("%s  addr: 0x%04x write:0x%02x read:0x%02x \r\n", __func__,
                    (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
        }
#endif
    }

    return 0;
}

AX_S32 imx327_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = g_imx327BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gImx327AeRegsTable) / sizeof(gImx327AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = IMX327_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gImx327AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = IMX327_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gImx327AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = IMX327_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gImx327AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gImx327AeRegsTable[i].nRegAddr, gImx327AeRegsTable[i].nRegValue);
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


