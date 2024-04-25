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
#include "sc1345_settings.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gSc1345BusInfo[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])


typedef struct _SC1345_GAIN_TABLE_T_ {
    AX_U8 gain;
    AX_U8 gain_fine;
    float gain_value;
} SC1345_GAIN_TABLE_T;


AX_F32 gAgainTable[SENSOR_MAX_GAIN_STEP];
AX_F32 gDgainTable[SENSOR_MAX_GAIN_STEP];

const SC1345_GAIN_TABLE_T gSc1345AgainTable[] = {
    {0x03, 0x20, 1.0000},
    {0x03, 0x21, 1.0313},
    {0x03, 0x22, 1.0625},
    {0x03, 0x23, 1.0938},
    {0x03, 0x24, 1.1250},
    {0x03, 0x25, 1.1563},
    {0x03, 0x26, 1.1875},
    {0x03, 0x27, 1.2188},
    {0x03, 0x28, 1.2500},
    {0x03, 0x29, 1.2813},
    {0x03, 0x2a, 1.3125},
    {0x03, 0x2b, 1.3438},
    {0x03, 0x2c, 1.3750},
    {0x03, 0x2d, 1.4063},
    {0x03, 0x2e, 1.4375},
    {0x03, 0x2f, 1.4688},
    {0x03, 0x30, 1.5000},
    {0x03, 0x31, 1.5313},
    {0x03, 0x32, 1.5625},
    {0x03, 0x33, 1.5938},
    {0x03, 0x34, 1.6250},
    {0x03, 0x35, 1.6563},
    {0x03, 0x36, 1.6875},
    {0x03, 0x37, 1.7188},
    {0x03, 0x38, 1.7500},
    {0x03, 0x39, 1.7813},
    {0x03, 0x3a, 1.8125},
    {0x03, 0x3b, 1.8438},
    {0x03, 0x3c, 1.8750},
    {0x03, 0x3d, 1.9063},
    {0x03, 0x3e, 1.9375},
    {0x03, 0x3f, 1.9688},
    {0x07, 0x20, 2.0000},
    {0x07, 0x21, 2.0625},
    {0x07, 0x22, 2.1250},
    {0x07, 0x23, 2.1875},
    {0x07, 0x24, 2.2500},
    {0x07, 0x25, 2.3125},
    {0x07, 0x26, 2.3750},
    {0x07, 0x27, 2.4375},
    {0x07, 0x28, 2.5000},
    {0x07, 0x29, 2.5625},
    {0x07, 0x2a, 2.6250},
    {0x07, 0x2b, 2.6875},
    {0x07, 0x2c, 2.7500},
    {0x07, 0x2d, 2.8125},
    {0x07, 0x2e, 2.8750},
    {0x07, 0x2f, 2.9375},
    {0x07, 0x30, 3.0000},
    {0x07, 0x31, 3.0625},
    {0x07, 0x32, 3.1250},
    {0x07, 0x33, 3.1875},
    {0x07, 0x34, 3.2500},
    {0x07, 0x35, 3.3125},
    {0x07, 0x36, 3.3750},
    {0x07, 0x37, 3.4375},
    {0x07, 0x38, 3.5000},
    {0x07, 0x39, 3.5625},
    {0x07, 0x3a, 3.6250},
    {0x07, 0x3b, 3.6875},
    {0x07, 0x3c, 3.7500},
    {0x07, 0x3d, 3.8125},
    {0x07, 0x3e, 3.8750},
    {0x07, 0x3f, 3.9375},
    {0x0f, 0x20, 4.0000},
    {0x0f, 0x21, 4.1250},
    {0x0f, 0x22, 4.2500},
    {0x0f, 0x23, 4.3750},
    {0x0f, 0x24, 4.5000},
    {0x0f, 0x25, 4.6250},
    {0x0f, 0x26, 4.7500},
    {0x0f, 0x27, 4.8750},
    {0x0f, 0x28, 5.0000},
    {0x0f, 0x29, 5.1250},
    {0x0f, 0x2a, 5.2500},
    {0x0f, 0x2b, 5.3750},
    {0x0f, 0x2c, 5.5000},
    {0x0f, 0x2d, 5.6250},
    {0x0f, 0x2e, 5.7500},
    {0x0f, 0x2f, 5.8750},
    {0x0f, 0x30, 6.0000},
    {0x0f, 0x31, 6.1250},
    {0x0f, 0x32, 6.2500},
    {0x0f, 0x33, 6.3750},
    {0x0f, 0x34, 6.5000},
    {0x0f, 0x35, 6.6250},
    {0x0f, 0x36, 6.7500},
    {0x0f, 0x37, 6.8750},
    {0x0f, 0x38, 7.0000},
    {0x0f, 0x39, 7.1250},
    {0x0f, 0x3a, 7.2500},
    {0x0f, 0x3b, 7.3750},
    {0x0f, 0x3c, 7.5000},
    {0x0f, 0x3d, 7.6250},
    {0x0f, 0x3e, 7.7500},
    {0x0f, 0x3f, 7.8750},
    {0x1f, 0x20, 8.0000},
    {0x1f, 0x21, 8.2500},
    {0x1f, 0x22, 8.5000},
    {0x1f, 0x23, 8.7500},
    {0x1f, 0x24, 9.0000},
    {0x1f, 0x25, 9.2500},
    {0x1f, 0x26, 9.5000},
    {0x1f, 0x27, 9.7500},
    {0x1f, 0x28, 10.0000},
    {0x1f, 0x29, 10.2500},
    {0x1f, 0x2a, 10.5000},
    {0x1f, 0x2b, 10.7500},
    {0x1f, 0x2c, 11.0000},
    {0x1f, 0x2d, 11.2500},
    {0x1f, 0x2e, 11.5000},
    {0x1f, 0x2f, 11.7500},
    {0x1f, 0x30, 12.0000},
    {0x1f, 0x31, 12.2500},
    {0x1f, 0x32, 12.5000},
    {0x1f, 0x33, 12.7500},
    {0x1f, 0x34, 13.0000},
    {0x1f, 0x35, 13.2500},
    {0x1f, 0x36, 13.5000},
    {0x1f, 0x37, 13.7500},
    {0x1f, 0x38, 14.0000},
    {0x1f, 0x39, 14.2500},
    {0x1f, 0x3a, 14.5000},
    {0x1f, 0x3b, 14.7500},
    {0x1f, 0x3c, 15.0000},
    {0x1f, 0x3d, 15.2500},
    {0x1f, 0x3e, 15.5000},
    {0x1f, 0x3f, 15.7500},
};

const SC1345_GAIN_TABLE_T gSc1345DgainTable[] = {
    {0x00, 0x80, 1.0000},
    {0x00, 0x84, 1.0313},
    {0x00, 0x88, 1.0625},
    {0x00, 0x8C, 1.0938},
    {0x00, 0x90, 1.1250},
    {0x00, 0x94, 1.1563},
    {0x00, 0x98, 1.1875},
    {0x00, 0x9C, 1.2188},
    {0x00, 0xA0, 1.2500},
    {0x00, 0xA4, 1.2813},
    {0x00, 0xA8, 1.3125},
    {0x00, 0xAC, 1.3438},
    {0x00, 0xB0, 1.3750},
    {0x00, 0xB4, 1.4063},
    {0x00, 0xB8, 1.4375},
    {0x00, 0xBC, 1.4688},
    {0x00, 0xC0, 1.5000},
    {0x00, 0xC4, 1.5313},
    {0x00, 0xC8, 1.5625},
    {0x00, 0xCC, 1.5938},
    {0x00, 0xD0, 1.6250},
    {0x00, 0xD4, 1.6563},
    {0x00, 0xD8, 1.6875},
    {0x00, 0xDC, 1.7188},
    {0x00, 0xE0, 1.7500},
    {0x00, 0xE4, 1.7813},
    {0x00, 0xE8, 1.8125},
    {0x00, 0xEC, 1.8438},
    {0x00, 0xF0, 1.8750},
    {0x00, 0xF4, 1.9063},
    {0x00, 0xF8, 1.9375},
    {0x00, 0xFC, 1.9688},
    {0x01, 0x80, 2.0000},
    {0x01, 0x84, 2.0625},
    {0x01, 0x88, 2.1250},
    {0x01, 0x8C, 2.1875},
    {0x01, 0x90, 2.2500},
    {0x01, 0x94, 2.3125},
    {0x01, 0x98, 2.3750},
    {0x01, 0x9C, 2.4375},
    {0x01, 0xA0, 2.5000},
    {0x01, 0xA4, 2.5625},
    {0x01, 0xA8, 2.6250},
    {0x01, 0xAC, 2.6875},
    {0x01, 0xB0, 2.7500},
    {0x01, 0xB4, 2.8125},
    {0x01, 0xB8, 2.8750},
    {0x01, 0xBC, 2.9375},
    {0x01, 0xC0, 3.0000},
    {0x01, 0xC4, 3.0625},
    {0x01, 0xC8, 3.1250},
    {0x01, 0xCC, 3.1875},
    {0x01, 0xD0, 3.2500},
    {0x01, 0xD4, 3.3125},
    {0x01, 0xD8, 3.3750},
    {0x01, 0xDC, 3.4375},
    {0x01, 0xE0, 3.5000},
    {0x01, 0xE4, 3.5625},
    {0x01, 0xE8, 3.6250},
    {0x01, 0xEC, 3.6875},
    {0x01, 0xF0, 3.7500},
    {0x01, 0xF4, 3.8125},
    {0x01, 0xF8, 3.8750},
    {0x01, 0xFC, 3.9375},
    {0x03, 0x80, 4.0000},
    {0x03, 0x84, 4.1250},
    {0x03, 0x88, 4.2500},
    {0x03, 0x8C, 4.3750},
    {0x03, 0x90, 4.5000},
    {0x03, 0x94, 4.6250},
    {0x03, 0x98, 4.7500},
    {0x03, 0x9C, 4.8750},
    {0x03, 0xA0, 5.0000},
    {0x03, 0xA4, 5.1250},
    {0x03, 0xA8, 5.2500},
    {0x03, 0xAC, 5.3750},
    {0x03, 0xB0, 5.5000},
    {0x03, 0xB4, 5.6250},
    {0x03, 0xB8, 5.7500},
    {0x03, 0xBC, 5.8750},
    {0x03, 0xC0, 6.0000},
    {0x03, 0xC4, 6.1250},
    {0x03, 0xC8, 6.2500},
    {0x03, 0xCC, 6.3750},
    {0x03, 0xD0, 6.5000},
    {0x03, 0xD4, 6.6250},
    {0x03, 0xD8, 6.7500},
    {0x03, 0xDC, 6.8750},
    {0x03, 0xE0, 7.0000},
    {0x03, 0xE4, 7.1250},
    {0x03, 0xE8, 7.2500},
    {0x03, 0xEC, 7.3750},
    {0x03, 0xF0, 7.5000},
    {0x03, 0xF4, 7.6250},
    {0x03, 0xF8, 7.7500},
    {0x03, 0xFC, 7.8750},
    {0x07, 0x80, 8.0000},
    {0x07, 0x84, 8.2500},
    {0x07, 0x88, 8.5000},
    {0x07, 0x8C, 8.7500},
    {0x07, 0x90, 9.0000},
    {0x07, 0x94, 9.2500},
    {0x07, 0x98, 9.5000},
    {0x07, 0x9C, 9.7500},
    {0x07, 0xA0, 10.0000},
    {0x07, 0xA4, 10.2500},
    {0x07, 0xA8, 10.5000},
    {0x07, 0xAC, 10.7500},
    {0x07, 0xB0, 11.0000},
    {0x07, 0xB4, 11.2500},
    {0x07, 0xB8, 11.5000},
    {0x07, 0xBC, 11.7500},
    {0x07, 0xC0, 12.0000},
    {0x07, 0xC4, 12.2500},
    {0x07, 0xC8, 12.5000},
    {0x07, 0xCC, 12.7500},
    {0x07, 0xD0, 13.0000},
    {0x07, 0xD4, 13.2500},
    {0x07, 0xD8, 13.5000},
    {0x07, 0xDC, 13.7500},
    {0x07, 0xE0, 14.0000},
    {0x07, 0xE4, 14.2500},
    {0x07, 0xE8, 14.5000},
    {0x07, 0xEC, 14.7500},
    {0x07, 0xF0, 15.0000},
    {0x07, 0xF4, 15.2500},
    {0x07, 0xF8, 15.5000},
    {0x07, 0xFC, 15.7500},
    {0x0f, 0x80, 16.0000},
    {0x0f, 0x84, 16.5000},
    {0x0f, 0x88, 17.0000},
    {0x0f, 0x8C, 17.5000},
    {0x0f, 0x90, 18.0000},
    {0x0f, 0x94, 18.5000},
    {0x0f, 0x98, 19.0000},
    {0x0f, 0x9C, 19.5000},
    {0x0f, 0xA0, 20.0000},
    {0x0f, 0xA4, 20.5000},
    {0x0f, 0xA8, 21.0000},
    {0x0f, 0xAC, 21.5000},
    {0x0f, 0xB0, 22.0000},
    {0x0f, 0xB4, 22.5000},
    {0x0f, 0xB8, 23.0000},
    {0x0f, 0xBC, 23.5000},
    {0x0f, 0xC0, 24.0000},
    {0x0f, 0xC4, 24.5000},
    {0x0f, 0xC8, 25.0000},
    {0x0f, 0xCC, 25.5000},
    {0x0f, 0xD0, 26.0000},
    {0x0f, 0xD4, 26.5000},
    {0x0f, 0xD8, 27.0000},
    {0x0f, 0xDC, 27.5000},
    {0x0f, 0xE0, 28.0000},
    {0x0f, 0xE4, 28.5000},
    {0x0f, 0xE8, 29.0000},
    {0x0f, 0xEC, 29.5000},
    {0x0f, 0xF0, 30.0000},
    {0x0f, 0xF4, 30.5000},
    {0x0f, 0xF8, 31.0000},
    {0x0f, 0xFC, 31.5000},
};


static AX_SNS_DRV_DELAY_TABLE_T gSc1345AeRegsTable[] = {

    /* nRegAddr */       /* regs value */    /* Delay Frame Number */
    {SC1345_INT_H,             0,                  0},
    {SC1345_INT_L,             0,                  0},
    {SC1345_AGAIN,             0,                  0},
    {SC1345_AGAIN_FINE,        0,                  0},
    {SC1345_DGAIN,             0,                  0},
    {SC1345_DGAIN_FINE,        0,                  0},
    {SC1345_AGAIN_CALIB_0,     0,                  0},
    {SC1345_AGAIN_CALIB_1,     0,                  0},
};

AX_U32 sc1345_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gSc1345AeRegsTable) / sizeof(gSc1345AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gSc1345AeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}


static AX_S32 sc1345_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
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
AX_S32 sc1345_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
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

    sc1345_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    sc1345_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}



AX_S32 sc1345_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    const camera_i2c_reg_array *default_setting = NULL;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    default_setting = sc1345_settings_table[setindex];

    while ((default_setting + reg_count)->addr != 0x0000) {
        reg_count++;
    }

    for (i = 0; i < reg_count; i++) {

        sc1345_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);
        SNS_DBG(" addr: 0x%04x value:0x%02x \r\n", (default_setting + i)->addr, (default_setting + i)->value);

        rBuf[0] = sc1345_reg_read(nPipeId, (default_setting + i)->addr);

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


AX_U32 sc1345_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = sc1345_reg_read(nPipeId, 0x320E);
    vts_l = sc1345_reg_read(nPipeId, 0x320F);

    vts = (AX_U32)(((vts_h & 0x7F) << 8) | (AX_U32)((vts_l & 0xFF) << 0));

    return vts;
}

AX_U32 sc1345_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_l = vts & 0xFF;
    vts_h = (vts & 0x7F00) >> 8;

    result |= sc1345_reg_write(nPipeId, 0x320E, vts_h);
    result |= sc1345_reg_write(nPipeId, 0x320F, vts_l);

    return result;
}

AX_U32 sc1345_set_int_t_l(ISP_PIPE_ID nPipeId, AX_U32 int_t_l)
{
    AX_U8 int_t_l_h;
    AX_U8 int_t_l_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    int_t_l_h = (int_t_l & 0xFF0) >> 4;
    int_t_l_l = (int_t_l & 0xF) << 4;

    result |= sc1345_sns_update_regs_table(nPipeId, SC1345_INT_H, int_t_l_h);
    result |= sc1345_sns_update_regs_table(nPipeId, SC1345_INT_L, int_t_l_l);

    return result;
}

AX_F32 sc1345_again2value(float gain_value, AX_U8 *gain, AX_U8 *gain_fine)
{
    AX_U32 i = 0;
    AX_U32 count = 0;
    AX_U32 ret_value = 0;

    if (!gain || !gain_fine)
        return -1;

    count = sizeof(gSc1345AgainTable) / sizeof(gSc1345AgainTable[0]);

    for (i = 0; i < count; i++) {
        if (gain_value > gSc1345AgainTable[i].gain_value) {
            continue;
        } else {
            *gain = gSc1345AgainTable[i].gain;
            *gain_fine = gSc1345AgainTable[i].gain_fine;
            return gSc1345AgainTable[i].gain_value;
        }
    }

    return -1;
}

AX_F32 sc1345_dgain2value(float gain_value, AX_U8 *gain, AX_U8 *gain_fine)
{
    AX_U32 i = 0;
    AX_U32 count = 0;
    AX_U32 ret_value = 0;

    if (!gain || !gain_fine)
        return -1;

    count = sizeof(gSc1345DgainTable) / sizeof(gSc1345DgainTable[0]);

    for (i = 0; i < count; i++) {
        if (gain_value > gSc1345DgainTable[i].gain_value) {
            continue;
        } else {
            *gain = gSc1345DgainTable[i].gain;
            *gain_fine = gSc1345DgainTable[i].gain_fine;
            return gSc1345DgainTable[i].gain_value;
        }
    }

    return -1;
}

AX_S32 sc1345_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i;
    AX_S32 ret = 0;
    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(gSc1345AgainTable) / sizeof(gSc1345AgainTable[0]);
    params->nDgainTableSize = sizeof(gSc1345DgainTable) / sizeof(gSc1345DgainTable[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        gAgainTable[i] = gSc1345AgainTable[i].gain_value;
    }

    for (i = 0; i < params->nDgainTableSize ; i++) {
        gDgainTable[i] = gSc1345DgainTable[i].gain_value;
    }

    params->pAgainTable = gAgainTable;
    params->pDgainTable = gDgainTable;

    return ret;
}

AX_U32 sc1345_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    AX_U32 rhs1 = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gSc1345AeRegsTable) / sizeof(gSc1345AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gSc1345AeRegsTable[i].nRegValue = sc1345_reg_read(nPipeId, gSc1345AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gSc1345AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gSc1345AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gSc1345AeRegsTable[i].nRegAddr, gSc1345AeRegsTable[i].nRegValue);
    }

    return SNS_SUCCESS;
}

AX_S32 sc1345_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gSc1345BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gSc1345AeRegsTable) / sizeof(gSc1345AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = SC1345_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gSc1345AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = SC1345_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gSc1345AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = SC1345_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gSc1345AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gSc1345AeRegsTable[i].nRegAddr, gSc1345AeRegsTable[i].nRegValue);
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


