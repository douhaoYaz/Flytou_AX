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
#include "os08a20_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];
extern AX_SNS_COMMBUS_T gOs08a20BusInfo[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])


static AX_SNS_DRV_DELAY_TABLE_T gOs08a20AeRegsTable[] = {
    /* nRegAddr */          /*regs value*/  /*Delay Frame Num*/
    {OS08A20_LONG_EXP_LINE_H,       0,          0},
    {OS08A20_LONG_EXP_LINE_L,       0,          0},
    {OS08A20_LONG_AGAIN_H,          0,          0},
    {OS08A20_LONG_AGAIN_L,          0,          0},
    {OS08A20_LONG_DGAIN_H,          0,          0},
    {OS08A20_LONG_DGAIN_L,          0,          0},

    {OS08A20_SHORT_EXP_LINE_H,      0,          0},
    {OS08A20_SHORT_EXP_LINE_L,      0,          0},
    {OS08A20_SHORT_AGAIN_H,         0,          0},
    {OS08A20_SHORT_AGAIN_L,         0,          0},
    {OS08A20_SHORT_DGAIN_H,         0,          0},
    {OS08A20_SHORT_DGAIN_L,         0,          0},
    {OS08A20_VTS_H,                 0,          0},
    {OS08A20_VTS_L,                 0,          0},
};

int gain_compare(float a, float b)
{
    float Express = 0.000001;

    if (a - b > Express)
        return 1;
    else
        return 0;
}

AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

typedef struct _OS08A20_GAIN_TABLE_T_ {
    float gain;
    AX_U8 again_in;
    AX_U8 again_de;
    AX_U8 dgain_in;
    AX_U8 dgain_de;
    AX_U8 dgain_de2;
} OS08A20_GAIN_TABLE_T;

const OS08A20_GAIN_TABLE_T os08a20_gain_table[] = {
   /* gain   ag_h  ag_l  dg_h  dg_l */
    {1,      0x00, 0x80, 0x04, 0x00},
    {1.0625, 0x00, 0x88, 0x04, 0x40},
    {1.125,  0x00, 0x90, 0x04, 0x80},
    {1.1875, 0x00, 0x98, 0x04, 0xC0},
    {1.25,   0x00, 0xA0, 0x05, 0x00},
    {1.3125, 0x00, 0xA8, 0x05, 0x40},
    {1.375,  0x00, 0xB0, 0x05, 0x80},
    {1.4375, 0x00, 0xB8, 0x05, 0xC0},
    {1.5,    0x00, 0xC0, 0x06, 0x00},
    {1.5625, 0x00, 0xC8, 0x06, 0x40},
    {1.625,  0x00, 0xD0, 0x06, 0x80},
    {1.6875, 0x00, 0xD8, 0x06, 0xC0},
    {1.75,   0x00, 0xE0, 0x07, 0x00},
    {1.8125, 0x00, 0xE8, 0x07, 0x40},
    {1.875,  0x00, 0xF0, 0x07, 0x80},
    {1.9375, 0x00, 0xF8, 0x07, 0xC0},
    {2,      0x01, 0x00, 0x08, 0x00},
    {2.125,  0x01, 0x10, 0x08, 0x80},
    {2.25,   0x01, 0x20, 0x09, 0x00},
    {2.375,  0x01, 0x30, 0x09, 0x80},
    {2.5,    0x01, 0x40, 0x0A, 0x00},
    {2.625,  0x01, 0x50, 0x0A, 0x80},
    {2.75,   0x01, 0x60, 0x0B, 0x00},
    {2.875,  0x01, 0x70, 0x0B, 0x80},
    {3,      0x01, 0x80, 0x0C, 0x00},
    {3.125,  0x01, 0x90, 0x0C, 0x80},
    {3.25,   0x01, 0xA0, 0x0D, 0x00},
    {3.375,  0x01, 0xB0, 0x0D, 0x80},
    {3.5,    0x01, 0xC0, 0x0E, 0x00},
    {3.625,  0x01, 0xD0, 0x0E, 0x80},
    {3.75,   0x01, 0xE0, 0x0F, 0x00},
    {3.875,  0x01, 0xF0, 0x0F, 0x80},
    {4,      0x02, 0x00, 0x10, 0x00},
    {4.25,   0x02, 0x20, 0x11, 0x00},
    {4.5,    0x02, 0x40, 0x12, 0x00},
    {4.75,   0x02, 0x60, 0x13, 0x00},
    {5,      0x02, 0x80, 0x14, 0x00},
    {5.25,   0x02, 0xA0, 0x15, 0x00},
    {5.5,    0x02, 0xC0, 0x16, 0x00},
    {5.75,   0x02, 0xE0, 0x17, 0x00},
    {6,      0x03, 0x00, 0x18, 0x00},
    {6.25,   0x03, 0x20, 0x19, 0x00},
    {6.5,    0x03, 0x40, 0x1A, 0x00},
    {6.75,   0x03, 0x60, 0x1B, 0x00},
    {7,      0x03, 0x80, 0x1C, 0x00},
    {7.25,   0x03, 0xA0, 0x1D, 0x00},
    {7.5,    0x03, 0xC0, 0x1E, 0x00},
    {7.75,   0x03, 0xE0, 0x1F, 0x00},
    {8,      0x04, 0x00, 0x20, 0x00},
    {8.5,    0x04, 0x40, 0x22, 0x00},
    {9,      0x04, 0x80, 0x24, 0x00},
    {9.5,    0x04, 0xC0, 0x26, 0x00},
    {10,     0x05, 0x00, 0x28, 0x00},
    {10.5,   0x05, 0x40, 0x2A, 0x00},
    {11,     0x05, 0x80, 0x2C, 0x00},
    {11.5,   0x05, 0xC0, 0x2E, 0x00},
    {12,     0x06, 0x00, 0x30, 0x00},
    {12.5,   0x06, 0x40, 0x32, 0x00},
    {13,     0x06, 0x80, 0x34, 0x00},
    {13.5,   0x06, 0xC0, 0x36, 0x00},
    {14,     0x07, 0x00, 0x38, 0x00},
    {14.5,   0x07, 0x40, 0x3A, 0x00},
    {15,     0x07, 0x80, 0x3C, 0x00},
    {15.5,   0x07, 0xC0, 0x3E, 0x00},
    {15.99,  0X07, 0xFF, 0x3F, 0xFF},
};


AX_F32 os08a20_again2value(float gain, AX_U8 *again_in, AX_U8 *again_de)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!again_in || !again_de)
        return -1;

    count = sizeof(os08a20_gain_table) / sizeof(os08a20_gain_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > os08a20_gain_table[i].gain) {
            continue;
        } else {
            if (again_in)
                *again_in = os08a20_gain_table[i].again_in;
            if (again_de)
                *again_de = os08a20_gain_table[i].again_de;
            SNS_DBG("again=%f, again_in=0x%x, again_de=0x%x\n", gain, *again_in, *again_de);
            return os08a20_gain_table[i].gain;
        }
    }

    return -1;
}

AX_F32 os08a20_dgain2value(float gain, AX_U8 *dgain_in, AX_U8 *dgain_de, AX_U8 *dgain_de2)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!dgain_in || !dgain_de)
        return -1;

    count = sizeof(os08a20_gain_table) / sizeof(os08a20_gain_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > os08a20_gain_table[i].gain) {
            continue;
        } else {
            *dgain_in = os08a20_gain_table[i].dgain_in;
            *dgain_de = os08a20_gain_table[i].dgain_de;

            SNS_DBG("dgain=%f, dgain_in=0x%x, dgain_de=0x%x\n", gain, *dgain_in, *dgain_de);

            return os08a20_gain_table[i].gain;
        }
    }

    return -1;
}


AX_S32 os08a20_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i;
    AX_S32 ret = 0;
    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(os08a20_gain_table) / sizeof(os08a20_gain_table[0]);
    params->nDgainTableSize = sizeof(os08a20_gain_table) / sizeof(os08a20_gain_table[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        nAgainTable[i] = os08a20_gain_table[i].gain;
        params->pAgainTable = nAgainTable;
    }

    for (i = 0; i < params->nDgainTableSize ; i++) {
        nDgainTable[i] = os08a20_gain_table[i].gain;
        params->pDgainTable = nDgainTable;
    }

    return ret;
}

AX_S32 os08a20_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
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
        SNS_ERR("Cannot open %s.\n", file_name);
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

AX_S32 os08a20_reset(ISP_PIPE_ID nPipeId)
{
    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

#ifdef AX630A_demo
    if (0 == nPipeId) {
        os08a20_hw_reset(50, 0);
        usleep(5 * 1000);
        os08a20_hw_reset(50, 1);
    } else {
        /* FIXME: project demo dual camera reset use same pin. */
        //os08a20_hw_reset(50, 0);
        //usleep(5 * 1000);
        //os08a20_hw_reset(50, 1);
    }
#elif ((defined AX630A_cp40) || (defined AX630A_evb))
    os08a20_hw_reset(20, 0);
    usleep(5 * 1000);
    os08a20_hw_reset(20, 1);
#endif

    return 0;
}

AX_S32 os08a20_group_prepare(ISP_PIPE_ID nPipeId)
{
    AX_S32 result = 0;

    result |= os08a20_reg_write(nPipeId, 0x320D, 0x00);
    result |= os08a20_reg_write(nPipeId, 0x3208, 0x00);
    result |= os08a20_reg_write(nPipeId, 0x0808, 0x00);
    result |= os08a20_reg_write(nPipeId, 0x3800, 0x11);
    result |= os08a20_reg_write(nPipeId, 0x3911, 0x22);
    result |= os08a20_reg_write(nPipeId, 0x3208, 0x10);

    return result;
}

AX_S32 os08a20_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    const camera_i2c_reg_array *default_setting = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("os08a20 setitng index: %d\n", setindex);

    SENSOR_GET_CTX(nPipeId, sns_obj);

    switch (setindex) {
    case e_os08a20_4lane_1440Mbps_24MHz_3840x2160_12bit_rggb_sdr_20fps:
        default_setting = &os08a20_4lane_1440Mbps_24MHz_3840x2160_12bit_rggb_sdr_20fps[0];
        reg_count = sizeof(os08a20_4lane_1440Mbps_24MHz_3840x2160_12bit_rggb_sdr_20fps) / sizeof(camera_i2c_reg_array);
        break;

    case e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr_20fps:
        default_setting = &os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr_20fps[0];
        reg_count = sizeof(os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr_20fps) / sizeof(camera_i2c_reg_array);
        break;

    case e_os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_30fps:
        default_setting = &os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_30fps[0];
        reg_count = sizeof(os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_30fps) / sizeof(camera_i2c_reg_array);
        break;

    case e_os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_25fps:
        default_setting = &os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_25fps[0];
        reg_count = sizeof(os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_25fps) / sizeof(camera_i2c_reg_array);
        break;

    case e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_25fps:
        default_setting = &os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_25fps[0];
        reg_count = sizeof(os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_25fps) / sizeof(camera_i2c_reg_array);
        break;

    case e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_30fps:
        default_setting = &os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_30fps[0];
        reg_count = sizeof(os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_30fps) / sizeof(camera_i2c_reg_array);
        break;

    case e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_sdr_60fps:
        default_setting = &os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_sdr_60fps[0];
        reg_count = sizeof(os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_sdr_60fps) / sizeof(camera_i2c_reg_array);
        break;

    default:
        SNS_ERR("it's not supported. pipe=%d, setting mode=%d] \n", nPipeId, setindex);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("setindex:%d, reg_count:%d\n", setindex, reg_count);

    SNS_DBG("os08a20 setitng index: %d, reg_count %d\n", setindex, reg_count);
    for (i = 0; i < reg_count; i++) {
        os08a20_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);

        rBuf[0] = os08a20_reg_read(nPipeId, (default_setting + i)->addr);
        SNS_DBG(" addr: 0x%04x write:0x%02x read:0x%02x \r\n",
                (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
#endif
    }

    return 0;
}

AX_U32 os08a20_get_hts(ISP_PIPE_ID nPipeId)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_U32 hts;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    hts_h = os08a20_reg_read(nPipeId, 0x380C);
    hts_l = os08a20_reg_read(nPipeId, 0x380D);

    hts = (AX_U32)(((hts_h & 0xF) << 8) | (AX_U32)(hts_l << 0));

    return hts;
}

AX_U32 os08a20_get_vs_hts(ISP_PIPE_ID nPipeId)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_U32 hts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts_h = os08a20_reg_read(nPipeId, 0x380C);
    hts_l = os08a20_reg_read(nPipeId, 0x380D);

    hts = (AX_U32)(((hts_h & 0xF) << 8) | (AX_U32)(hts_l << 0));

    return hts;
}

AX_U32 os08a20_set_hts(ISP_PIPE_ID nPipeId, AX_U32 hts)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts_l = hts & 0xFF;
    hts_h = (hts & 0xFF00) >> 8;

    result |= os08a20_reg_write(nPipeId, 0x380C, hts_h);
    result |= os08a20_reg_write(nPipeId, 0x380D, hts_l);

    return result;
}

AX_U32 os08a20_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = os08a20_reg_read(nPipeId, OS08A20_VTS_H);
    vts_l = os08a20_reg_read(nPipeId, OS08A20_VTS_L);

    vts = (AX_U32)(((vts_h & 0xF) << 8) | (AX_U32)(vts_l << 0));

    return vts;
}

AX_U32 os08a20_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_l = vts & 0xFF;
    vts_h = (vts & 0xFF00) >> 8;

    result |= os08a20_sns_update_regs_table(nPipeId, OS08A20_VTS_H, vts_h);
    result |= os08a20_sns_update_regs_table(nPipeId, OS08A20_VTS_L, vts_l);

    return result;
}

AX_F32 os08a20_get_sclk(ISP_PIPE_ID nPipeId)
{
    AX_U8 pre_div0;
    AX_U8 pre_div;
    AX_U16 multiplier;
    AX_U8 post_div;
    AX_U8 sram_div;
    AX_U8 st_div;
    AX_U8 t_div;
    float inck;
    float sclk;

    inck = OS08a20_INCK_24M;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    pre_div0 = (os08a20_reg_read(nPipeId, 0x0322) & 0x1) + 1;

    pre_div = os08a20_reg_read(nPipeId, 0x0323) & 0x7;
    if (pre_div == 0) {
        pre_div = 1;
    } else if (pre_div == 1) {
        pre_div = 1.5;
    } else if (pre_div == 2) {
        pre_div = 2;
    } else if (pre_div == 3) {
        pre_div = 2.5;
    } else if (pre_div == 4) {
        pre_div = 3;
    } else if (pre_div == 5) {
        pre_div = 4;
    } else if (pre_div == 6) {
        pre_div = 6;
    } else if (pre_div == 7) {
        pre_div = 8;
    } else {
    }

    multiplier = (os08a20_reg_read(nPipeId, 0x0324) & 0x3) << 8;
    multiplier = multiplier | ((os08a20_reg_read(nPipeId, 0x0325)) & 0xFF);

    post_div = (os08a20_reg_read(nPipeId, 0x032f) & 0xF) + 1;
    sram_div = (os08a20_reg_read(nPipeId, 0x0327) & 0xF) + 1;
    st_div = (os08a20_reg_read(nPipeId, 0x0328) & 0xF) + 1;

    t_div = os08a20_reg_read(nPipeId, 0x032a) & 0x7;
    if (t_div == 0) {
        t_div = 1;
    } else if (t_div == 1) {
        t_div = 1.5;
    } else if (t_div == 2) {
        t_div = 2;
    } else if (t_div == 3) {
        t_div = 2.5;
    } else if (t_div == 4) {
        t_div = 3;
    } else if (t_div == 5) {
        t_div = 3.5;
    } else if (t_div == 6) {
        t_div = 4;
    } else if (t_div == 7) {
        t_div = 5;
    } else {
    }
    sclk = (((((((float)(inck * 1000 * 1000) / pre_div0) / pre_div) * multiplier) / post_div) / st_div) / t_div);
    /*printf("%s pre_div0=0x%x, pre_div=0x%x, multiplier=0x%x, post_div=0x%x, sram_div=0x%x, st_div=0x%x, t_div=0x%x \n", \
          __func__, pre_div0, pre_div, multiplier, post_div, sram_div, st_div, t_div); */

    return sclk;
}

AX_U32 os08a20_get_shortframe_expline(ISP_PIPE_ID nPipeId)
{
    AX_U8 s_exp_h;
    AX_U8 s_exp_l;
    AX_U32 expline = 0;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    s_exp_h = os08a20_reg_read(nPipeId, OS08A20_SHORT_EXP_LINE_H);
    s_exp_l = os08a20_reg_read(nPipeId, OS08A20_SHORT_EXP_LINE_L);

    expline = (AX_U32)(((s_exp_h) << 8) | (AX_U32)(s_exp_l << 0));

    return expline;
}

AX_U32 os08a20_get_l2s_offset(ISP_PIPE_ID nPipeId)
{
    AX_U32 offset = 0;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    offset = os08a20_reg_read(nPipeId, 0x3797);
    offset = offset & 0x1f;
    offset += 1;

    return offset;
}

AX_U32 os08a20_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gOs08a20AeRegsTable) / sizeof(gOs08a20AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gOs08a20AeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}

AX_U32 os08a20_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gOs08a20AeRegsTable) / sizeof(gOs08a20AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gOs08a20AeRegsTable[i].nRegValue = os08a20_reg_read(nPipeId, gOs08a20AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gOs08a20AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gOs08a20AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gOs08a20AeRegsTable[i].nRegAddr, gOs08a20AeRegsTable[i].nRegValue);
    }

    /* linegap default vaule */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        /* 2DOL line gap */
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] =
            os08a20_get_shortframe_expline(nPipeId) + os08a20_get_l2s_offset(nPipeId);
    }

    return SNS_SUCCESS;
}

AX_S32 os08a20_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
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
        sns_obj->sztRegsInfo[0].tComBus.I2cDev = gOs08a20BusInfo[nPipeId].I2cDev;
        sns_obj->sztRegsInfo[0].nRegNum = sizeof(gOs08a20AeRegsTable) / sizeof(gOs08a20AeRegsTable[0]);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

        for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = OS08A20_SLAVE_ADDR;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gOs08a20AeRegsTable[i].nRegAddr;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = OS08A20_ADDR_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gOs08a20AeRegsTable[i].nRegValue;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = OS08A20_DATA_BYTE;
            sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gOs08a20AeRegsTable[i].nDelayFrmNum;
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    gOs08a20AeRegsTable[i].nRegAddr, gOs08a20AeRegsTable[i].nRegValue);
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


