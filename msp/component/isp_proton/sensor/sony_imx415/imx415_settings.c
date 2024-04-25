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
#include "imx415_settings.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern SNS_STATE_OBJ *g_szImx415Ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gImx415BusInfo[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = g_szImx415Ctx[dev])


static AX_SNS_DRV_DELAY_TABLE_T gImx415AeRegsTable[] = {

    /* nRegAddr */       /* regs value */    /* Delay Frame Number */
    {IMX415_SHR0_L,             0,                  0},
    {IMX415_SHR0_M,             0,                  0},
    {IMX415_SHR0_H,             0,                  0},
    {IMX415_SHR1_L,             0,                  0},
    {IMX415_SHR1_M,             0,                  0},
    {IMX415_SHR1_H,             0,                  0},
    {IMX415_RHS1_L,             0,                  0},
    {IMX415_RHS1_M,             0,                  0},
    {IMX415_RHS1_H,             0,                  0},

    {IMX415_GAIN_PGC_0_L,       0,                  0},
    {IMX415_GAIN_PGC_0_H,       0,                  0},
    {IMX415_GAIN_PGC_1_L,       0,                  0},
    {IMX415_GAIN_PGC_1_H,       0,                  0},

    {IMX415_VMAX_L,             0,                  0},
    {IMX415_VMAX_M,             0,                  0},
    {IMX415_VMAX_H,             0,                  0},
};

AX_U32 imx415_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    nNum = sizeof(gImx415AeRegsTable) / sizeof(gImx415AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gImx415AeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}

AX_U32 imx415_get_shr0(ISP_PIPE_ID nPipeId)
{
    AX_U8 shr0_h;
    AX_U8 shr0_m;
    AX_U8 shr0_l;
    AX_U32 shr0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shr0_l = imx415_reg_read(nPipeId, IMX415_SHR0_L);
    shr0_m = imx415_reg_read(nPipeId, IMX415_SHR0_M);
    shr0_h = imx415_reg_read(nPipeId, IMX415_SHR0_H);

    shr0 = (AX_U32)(((shr0_h & 0xF) << 16) | (AX_U32)(shr0_m << 8) | (AX_U32)(shr0_l << 0));

    //SNS_DBG("get shr0: 0x%x [0x%x, 0x%x, 0x%x]\n", shr0, shr0_h, shr0_m, shr0_l);

    return shr0;
}

AX_U32 imx415_set_shr0(ISP_PIPE_ID nPipeId, AX_U32 shr0)
{
    AX_U8 shr0_h;
    AX_U8 shr0_m;
    AX_U8 shr0_l;

    AX_U32 result = 0;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shr0_l = shr0 & 0xFF;
    shr0_m = (shr0 & 0xFF00) >> 8;
    shr0_h = (shr0 & 0xF0000) >> 16;

    result |= imx415_sns_update_regs_table(nPipeId, IMX415_SHR0_L, shr0_l);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_SHR0_M, shr0_m);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_SHR0_H, shr0_h);

    //SNS_DBG("set shr0: 0x%x [0x%x, 0x%x, 0x%x]\n", shr0, shr0_h, shr0_m, shr0_l);

    return result;
}

AX_U32 imx415_get_shr1(ISP_PIPE_ID nPipeId)
{
    AX_U8 shr1_h;
    AX_U8 shr1_m;
    AX_U8 shr1_l;
    AX_U32 shr1;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shr1_l = imx415_reg_read(nPipeId, IMX415_SHR1_L);
    shr1_m = imx415_reg_read(nPipeId, IMX415_SHR1_M);
    shr1_h = imx415_reg_read(nPipeId, IMX415_SHR1_H);

    shr1 = (AX_U32)(((shr1_h & 0xF) << 16) | (AX_U32)(shr1_m << 8) | (AX_U32)(shr1_l << 0));

    //SNS_DBG("get shr1: 0x%x [0x%x, 0x%x, 0x%x]\n", shr1, shr1_h, shr1_m, shr1_l);

    return shr1;
}

AX_U32 imx415_set_shr1(ISP_PIPE_ID nPipeId, AX_U32 shr1)
{
    AX_U8 shr1_h;
    AX_U8 shr1_m;
    AX_U8 shr1_l;

    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    shr1_l = shr1 & 0xFF;
    shr1_m = (shr1 & 0xFF00) >> 8;
    shr1_h = (shr1 & 0xF0000) >> 16;

    result |= imx415_sns_update_regs_table(nPipeId, IMX415_SHR1_L, shr1_l);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_SHR1_M, shr1_m);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_SHR1_H, shr1_h);

    //SNS_DBG("set shr1: 0x%x [0x%x, 0x%x, 0x%x]\n", shr1, shr1_h, shr1_m, shr1_l);

    return result;
}

AX_U32 imx415_get_rhs1(ISP_PIPE_ID nPipeId)
{
    AX_U8 rhs1_h;
    AX_U8 rhs1_m;
    AX_U8 rhs1_l;
    AX_U32 rhs1;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    rhs1_l = imx415_reg_read(nPipeId, IMX415_RHS1_L);
    rhs1_m = imx415_reg_read(nPipeId, IMX415_RHS1_M);
    rhs1_h = imx415_reg_read(nPipeId, IMX415_RHS1_H);

    rhs1 = (AX_U32)(((rhs1_h & 0xF) << 16) | (AX_U32)(rhs1_m << 8) | (AX_U32)(rhs1_l << 0));

    //SNS_DBG("get rhs1: 0x%x [0x%x, 0x%x, 0x%x]\n", rhs1, rhs1_h, rhs1_m, rhs1_l);

    return rhs1;
}

AX_U32 imx415_set_rhs1(ISP_PIPE_ID nPipeId, AX_U32 rhs1)
{
    AX_U8 rhs1_h;
    AX_U8 rhs1_m;
    AX_U8 rhs1_l;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    rhs1_l = rhs1 & 0xFF;
    rhs1_m = (rhs1 & 0xFF00) >> 8;
    rhs1_h = (rhs1 & 0xF0000) >> 16;

    result |= imx415_sns_update_regs_table(nPipeId, IMX415_RHS1_L, rhs1_l);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_RHS1_M, rhs1_m);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_RHS1_H, rhs1_h);

    //SNS_DBG("set rhs1: 0x%x [0x%x, 0x%x, 0x%x]\n", rhs1, rhs1_h, rhs1_m, rhs1_l);

    return result;
}

AX_U32 imx415_set_vmax(ISP_PIPE_ID nPipeId, AX_U32 vmax)
{
    AX_U8 vmax_h;
    AX_U8 vmax_m;
    AX_U8 vmax_l;
    AX_U32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vmax_l = vmax & 0xFF;
    vmax_m = (vmax & 0xFF00) >> 8;
    vmax_h = (vmax & 0xF0000) >> 16;

    result |= imx415_sns_update_regs_table(nPipeId, IMX415_VMAX_L, vmax_l);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_VMAX_M, vmax_m);
    result |= imx415_sns_update_regs_table(nPipeId, IMX415_VMAX_H, vmax_h);

    //SNS_DBG("set vmax: 0x%x [0x%x, 0x%x, 0x%x]\n", vmax, vmax_h, vmax_m, vmax_l);

    return result;
}

AX_U32 imx415_get_vmax(ISP_PIPE_ID nPipeId)
{
    AX_U8 vmax_h;
    AX_U8 vmax_m;
    AX_U8 vmax_l;

    AX_U32 vmax;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vmax_l = imx415_reg_read(nPipeId, IMX415_VMAX_L);
    vmax_m = imx415_reg_read(nPipeId, IMX415_VMAX_M);
    vmax_h = imx415_reg_read(nPipeId, IMX415_VMAX_H);

    vmax = (AX_U32)(((vmax_h & 0xF) << 16) | (AX_U32)(vmax_m << 8) | (AX_U32)(vmax_l << 0));

    //SNS_DBG("get vmax: 0x%x [0x%x, 0x%x, 0x%x]\n", vmax, vmax_h, vmax_m, vmax_l);

    return vmax;
}

AX_U32 imx415_get_hmax(ISP_PIPE_ID nPipeId)
{
    AX_U8 hmax_h;
    AX_U8 hmax_l;
    AX_U32 hmax;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hmax_l = imx415_reg_read(nPipeId, IMX415_HMAX_L);
    hmax_h = imx415_reg_read(nPipeId, IMX415_HMAX_H);

    hmax = (AX_U32)(((hmax_h & 0xF) << 8) | (AX_U32)(hmax_l << 0));

    //SNS_DBG("get hmax: 0x%x [0x%x, 0x%x]\n", hmax, hmax_h, hmax_l);

    return hmax;
}

static AX_S32 imx415_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
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
AX_S32 imx415_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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

    imx415_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    imx415_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}



AX_S32 imx415_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
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
    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_20FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_20FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_20FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_25FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_25FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_30FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_30FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1485MBPS_SDR_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1782MBPS_HDR2DOL_25FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1782MBPS_HDR2DOL_25FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1782MBPS_HDR2DOL_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1782MBPS_HDR2DOL_30FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1782MBPS_HDR2DOL_30FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_3864x2164_10BIT_1782MBPS_HDR2DOL_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_SDR_25FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_SDR_25FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_SDR_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_SDR_30FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_SDR_30FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_SDR_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_HDR2DOL_25FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_HDR2DOL_25FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_HDR2DOL_25FPS) / sizeof(camera_i2c_reg_array);
        break;

    case E_IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_HDR2DOL_30FPS:
        default_setting = &IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_HDR2DOL_30FPS[0];
        reg_count = sizeof(IMX415_WINDOWCROP_CSI2_4LANE_27MHZ_GBRG_2904x1620_10BIT_1485MBPS_HDR2DOL_30FPS) / sizeof(camera_i2c_reg_array);
        break;

    default:
        SNS_ERR("it's not supported. pipe=%d, setting mode=%d] \n", nPipeId, setindex);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("setindex:%d, reg_count:%d\n", setindex, reg_count);

    for (i = 0; i < reg_count; i++) {
        imx415_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);
        SNS_DBG(" addr: 0x%04x value:0x%02x \r\n", (default_setting + i)->addr, (default_setting + i)->value);

        rBuf[0] = imx415_reg_read(nPipeId, (default_setting + i)->addr);
        if ((rBuf[0] != (default_setting + i)->value)) {
            errnum++;
            SNS_ERR("%s  addr: 0x%04x write:0x%02x read:0x%02x \r\n", __func__,
                    (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
        }
#endif
    }

    return 0;
}

AX_U32 imx415_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    AX_U32 rhs1 = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    nNum = sizeof(gImx415AeRegsTable) / sizeof(gImx415AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gImx415AeRegsTable[i].nRegValue = imx415_reg_read(nPipeId, gImx415AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gImx415AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gImx415AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gImx415AeRegsTable[i].nRegAddr, gImx415AeRegsTable[i].nRegValue);
    }

    /* linegap default vaule */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        /* 2DOL line gap = VBP1 = (RHS1-3)/2 */
        rhs1 = imx415_get_rhs1(nPipeId);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = IMX415_L2S_LINEGAP_CALC(rhs1);
    }

    return SNS_SUCCESS;
}

AX_S32 imx415_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gImx415BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gImx415AeRegsTable) / sizeof(gImx415AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = IMX415_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gImx415AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = IMX415_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gImx415AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = IMX415_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gImx415AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gImx415AeRegsTable[i].nRegAddr, gImx415AeRegsTable[i].nRegValue);
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


