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
#include "imx678_settings.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern SNS_STATE_OBJ *g_szImx678Ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gImx678BusInfo[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = g_szImx678Ctx[dev])


static AX_SNS_DRV_DELAY_TABLE_T gImx678AeRegsTable[] = {

    /* nRegAddr */       /* regs value */    /* Delay Frame Number */
    {IMX678_SHR0_L,             0,                  0},
    {IMX678_SHR0_M,             0,                  0},
    {IMX678_SHR0_H,             0,                  0},
    {IMX678_SHR1_L,             0,                  0},
    {IMX678_SHR1_M,             0,                  0},
    {IMX678_SHR1_H,             0,                  0},
    {IMX678_RHS1_L,             0,                  0},
    {IMX678_RHS1_M,             0,                  0},
    {IMX678_RHS1_H,             0,                  0},

    {IMX678_GAIN0_L,            0,                  0},
    {IMX678_GAIN0_H,            0,                  0},
    {IMX678_GAIN1_L,            0,                  0},
    {IMX678_GAIN1_H,            0,                  0},

    {IMX678_FDG_SEL0,           0,                  0},
    {IMX678_FDG_SEL1,           0,                  0},

    {IMX678_VMAX_L,             0,                  0},
    {IMX678_VMAX_M,             0,                  0},
    {IMX678_VMAX_H,             0,                  0},
};

AX_U32 imx678_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    nNum = sizeof(gImx678AeRegsTable) / sizeof(gImx678AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gImx678AeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}

AX_S32 imx678_set_shr0(ISP_PIPE_ID nPipeId, AX_U32 shr0)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = shr0 & 0xFF;
    m = (shr0 & 0xFF00) >> 8;
    h = (shr0 & 0xF0000) >> 16;

    result |= imx678_sns_update_regs_table(nPipeId, IMX678_SHR0_L, l);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_SHR0_M, m);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_SHR0_H, h);

    //SNS_DBG("set shr0: 0x%x [0x%x, 0x%x, 0x%x]\n", shr0, h, m, l);

    return result;
}

AX_U32 imx678_get_shr0(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = imx678_reg_read(nPipeId, IMX678_SHR0_L);
    m = imx678_reg_read(nPipeId, IMX678_SHR0_M);
    h = imx678_reg_read(nPipeId, IMX678_SHR0_H);

    val = (AX_U32)(((h & 0xF) << 16) | (AX_U32)(m << 8) | (AX_U32)(l << 0));

    //SNS_DBG("get shr0: 0x%x [0x%x, 0x%x, 0x%x]\n", val, h, m, l);

    return val;
}


AX_S32 imx678_set_shr1(ISP_PIPE_ID nPipeId, AX_U32 shr1)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = shr1 & 0xFF;
    m = (shr1 & 0xFF00) >> 8;
    h = (shr1 & 0xF0000) >> 16;

    result |= imx678_sns_update_regs_table(nPipeId, IMX678_SHR1_L, l);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_SHR1_M, m);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_SHR1_H, h);

    //SNS_DBG("set shr1: 0x%x [0x%x, 0x%x, 0x%x]\n", shr1, h, m, l);

    return result;
}

AX_U32 imx678_get_shr1(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = imx678_reg_read(nPipeId, IMX678_SHR1_L);
    m = imx678_reg_read(nPipeId, IMX678_SHR1_M);
    h = imx678_reg_read(nPipeId, IMX678_SHR1_H);

    val = (AX_U32)(((h & 0xF) << 16) | (AX_U32)(m << 8) | (AX_U32)(l << 0));

    //SNS_DBG("get shr0: 0x%x [0x%x, 0x%x, 0x%x]\n", val, h, m, l);

    return val;
}


AX_S32 imx678_set_rhs1(ISP_PIPE_ID nPipeId, AX_U32 rhs1)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = rhs1 & 0xFF;
    m = (rhs1 & 0xFF00) >> 8;
    h = (rhs1 & 0xF0000) >> 16;

    result |= imx678_sns_update_regs_table(nPipeId, IMX678_RHS1_L, l);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_RHS1_M, m);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_RHS1_H, h);

    //SNS_DBG("set rhs1: 0x%x [0x%x, 0x%x, 0x%x]\n", rhs1, h, m, l);

    return result;
}

AX_U32 imx678_get_rhs1(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = imx678_reg_read(nPipeId, IMX678_RHS1_L);
    m = imx678_reg_read(nPipeId, IMX678_RHS1_M);
    h = imx678_reg_read(nPipeId, IMX678_RHS1_H);

    val = (AX_U32)(((h & 0xF) << 16) | (AX_U32)(m << 8) | (AX_U32)(l << 0));

    //SNS_DBG("get rhs1: 0x%x [0x%x, 0x%x, 0x%x]\n", val, h, m, l);

    return val;
}

AX_U32 imx678_set_vmax(ISP_PIPE_ID nPipeId, AX_U32 vmax)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = vmax & 0xFF;
    m = (vmax & 0xFF00) >> 8;
    h = (vmax & 0xF0000) >> 16;

    result |= imx678_sns_update_regs_table(nPipeId, IMX678_VMAX_L, l);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_VMAX_M, m);
    result |= imx678_sns_update_regs_table(nPipeId, IMX678_VMAX_H, h);

    //SNS_DBG("set vmax: 0x%x [0x%x, 0x%x, 0x%x]\n", vmax, h, m, l);

    return result;
}

AX_U32 imx678_get_vmax(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 m = 0;
    AX_U8 l = 0;
    AX_U32 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = imx678_reg_read(nPipeId, IMX678_VMAX_L);
    m = imx678_reg_read(nPipeId, IMX678_VMAX_M);
    h = imx678_reg_read(nPipeId, IMX678_VMAX_H);

    val = (AX_U32)(((h & 0xF) << 16) | (AX_U32)(m << 8) | (AX_U32)(l << 0));

    //SNS_DBG("get vmax: 0x%x [0x%x, 0x%x, 0x%x]\n", val, h, m, l);

    return val;
}

AX_U32 imx678_get_hmax(ISP_PIPE_ID nPipeId)
{
    AX_U8 h = 0;
    AX_U8 l = 0;
    AX_U32 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    l = imx678_reg_read(nPipeId, IMX678_HMAX_L);
    h = imx678_reg_read(nPipeId, IMX678_HMAX_H);

    val = (AX_U32)((AX_U32)(h << 8) | (AX_U32)(l << 0));

    //SNS_DBG("get hmax: 0x%x [0x%x, 0x%x, 0x%x]\n", val, h, m, l);

    return val;
}

static AX_S32 imx678_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
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
AX_S32 imx678_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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

    imx678_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    imx678_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}



AX_S32 imx678_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    const camera_i2c_reg_array *default_setting = NULL;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    switch (setindex) {
    case E_IMX678_4LANE_24MHZ_891MBPS_3840x2160_RGGB_10BIT_LINEAR_25FPS:
        default_setting = &IMX678_4LANE_24MHZ_891MBPS_3840x2160_RGGB_10BIT_LINEAR_25FPS[0];
        reg_count = sizeof(IMX678_4LANE_24MHZ_891MBPS_3840x2160_RGGB_10BIT_LINEAR_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX678_4LANE_24MHZ_891MBPS_3840x2160_RGGB_10BIT_LINEAR_30FPS:
        default_setting = &IMX678_4LANE_24MHZ_891MBPS_3840x2160_RGGB_10BIT_LINEAR_30FPS[0];
        reg_count = sizeof(IMX678_4LANE_24MHZ_891MBPS_3840x2160_RGGB_10BIT_LINEAR_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX678_4LANE_24MHZ_1188MBPS_3840x2160_RGGB_12BIT_LINEAR_25FPS:
        default_setting = &IMX678_4LANE_24MHZ_1188MBPS_3840x2160_RGGB_12BIT_LINEAR_25FPS[0];
        reg_count = sizeof(IMX678_4LANE_24MHZ_1188MBPS_3840x2160_RGGB_12BIT_LINEAR_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX678_4LANE_24MHZ_1188MBPS_3840x2160_RGGB_12BIT_LINEAR_30FPS:
        default_setting = &IMX678_4LANE_24MHZ_1188MBPS_3840x2160_RGGB_12BIT_LINEAR_30FPS[0];
        reg_count = sizeof(IMX678_4LANE_24MHZ_1188MBPS_3840x2160_RGGB_12BIT_LINEAR_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX678_4LANE_24MHZ_1440MBPS_3840x2160_RGGB_10BIT_HDR2DOL_25FPS:
        default_setting = &IMX678_4LANE_24MHZ_1440MBPS_3840x2160_RGGB_10BIT_HDR2DOL_25FPS[0];
        reg_count = sizeof(IMX678_4LANE_24MHZ_1440MBPS_3840x2160_RGGB_10BIT_HDR2DOL_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX678_4LANE_24MHZ_1440MBPS_3840x2160_RGGB_10BIT_HDR2DOL_30FPS:
        default_setting = &IMX678_4LANE_24MHZ_1440MBPS_3840x2160_RGGB_10BIT_HDR2DOL_30FPS[0];
        reg_count = sizeof(IMX678_4LANE_24MHZ_1440MBPS_3840x2160_RGGB_10BIT_HDR2DOL_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    default:
        SNS_ERR("it's not supported. pipe=%d, setting mode=%d] \n", nPipeId, setindex);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("setindex:%d, reg_count:%d\n", setindex, reg_count);

    for (i = 0; i < reg_count; i++) {
        imx678_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);
        SNS_DBG(" addr: 0x%04x value:0x%02x \r\n", (default_setting + i)->addr, (default_setting + i)->value);

        rBuf[0] = imx678_reg_read(nPipeId, (default_setting + i)->addr);
        if ((rBuf[0] != (default_setting + i)->value)) {
            errnum++;
            SNS_ERR("%s  addr: 0x%04x write:0x%02x read:0x%02x \r\n", __func__,
                    (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
        }
#endif
    }

    return 0;
}

AX_U32 imx678_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    AX_U32 rhs1 = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    nNum = sizeof(gImx678AeRegsTable) / sizeof(gImx678AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gImx678AeRegsTable[i].nRegValue = imx678_reg_read(nPipeId, gImx678AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gImx678AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gImx678AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gImx678AeRegsTable[i].nRegAddr, gImx678AeRegsTable[i].nRegValue);
    }

    /* linegap default vaule */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        /* 2DOL line gap = VBP1 = (RHS1-3)/2 */
        rhs1 = imx678_get_rhs1(nPipeId);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = IMX678_L2S_LINEGAP_CALC(rhs1);
    }

    return SNS_SUCCESS;
}

AX_S32 imx678_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gImx678BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gImx678AeRegsTable) / sizeof(gImx678AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = IMX678_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gImx678AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = IMX678_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gImx678AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = IMX678_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gImx678AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gImx678AeRegsTable[i].nRegAddr, gImx678AeRegsTable[i].nRegValue);
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


