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
#include <stdlib.h>
#include <math.h>
#include "i2c.h"
#include "ax_sensor_struct.h"
#include "ax_base_type.h"
#include "imx464_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "imx464_sdr.h"
#include "imx464_hdr_2x.h"


/****************************************************************************
 * golbal variables  and macro definition
 ****************************************************************************/
#define IMX464_INCLK_74POINT25M         (74.25)
#define IMX464_ALLPIXEL_BRL             (1558)
#define IMX464_MAX_VMAX                 (0xFFFFF)
#define IMX464_MAX_RATIO                (16.0f)
#define IMX464_MIN_RATIO                (1.0f)


IMX464_PARAMS gImx464Params[DEF_VIN_PIPE_MAX_NUM];
AX_SNS_COMMBUS_T gImx464BusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0} };

SNS_STATE_OBJ *g_szImx464Ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = g_szImx464Ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (g_szImx464Ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (g_szImx464Ctx[dev] = AX_NULL)


AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

/*user config*/
static AX_F32 gFpsGear[] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00, 11.00, 12.00, 13.00, 14.00, 15.00,
                            16.00, 17.00, 18.00, 19.00, 20.00, 21.00, 22.00, 23.00, 24.00, 25.00, 26.00, 27.00, 28.00, 29.00, 30.00
                           };

static AX_S32 imx464_set_aecparam(ISP_PIPE_ID nPipeId);
static AX_S32 imx464_set_exp_limit(ISP_PIPE_ID nPipeId, AX_F32 fExpRatio, AX_U32 nVmax);

/****************************************************************************
 * Internal function definition
 ****************************************************************************/
static AX_S32 sensor_ctx_init(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 ret = 0;

    SNS_DBG("os04a10 sensor_ctx_init. ret = %d\n", ret);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);

    if (AX_NULL == sns_obj) {
        sns_obj = (SNS_STATE_OBJ *)calloc(1, sizeof(SNS_STATE_OBJ));
        if (AX_NULL == sns_obj) {
            SNS_ERR("malloc g_szImx464Ctx failed\r\n");
            return SNS_ERR_CODE_NOT_MEM;
        }
    }

    memset(sns_obj, 0, sizeof(SNS_STATE_OBJ));

    SENSOR_SET_CTX(nPipeId, sns_obj);

    return SNS_SUCCESS;
}

static AX_VOID sensor_ctx_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SENSOR_GET_CTX(nPipeId, sns_obj);
    free(sns_obj);
    SENSOR_RESET_CTX(nPipeId);
}


/****************************************************************************
 * sensor control function
 ****************************************************************************/
AX_S32 imx464_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
{
    AX_U8 data;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 nRet = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (-1 == sns_obj->sns_i2c_obj.sns_i2c_fd)
        return SNS_ERR_CODE_ILLEGAL_PARAMS;

    i2c_read(sns_obj->sns_i2c_obj.sns_i2c_fd, sns_obj->sns_i2c_obj.slave_addr, addr, sns_obj->sns_i2c_obj.address_byte,
             (AX_U8 *)(&data), sns_obj->sns_i2c_obj.data_byte, sns_obj->sns_i2c_obj.swap_byte);

    return data;
}

AX_S32 imx464_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
{
    AX_U32 ret = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (-1 == sns_obj->sns_i2c_obj.sns_i2c_fd)
        return SNS_ERR_CODE_ILLEGAL_PARAMS;

    ret = i2c_write(sns_obj->sns_i2c_obj.sns_i2c_fd, sns_obj->sns_i2c_obj.slave_addr, addr,
                    sns_obj->sns_i2c_obj.address_byte,
                    (AX_U8 *)(&data), sns_obj->sns_i2c_obj.data_byte, sns_obj->sns_i2c_obj.swap_byte);

    return ret;
}


static AX_S32 sensor_i2c_init(ISP_PIPE_ID nPipeId)
{
    AX_S32 ret = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 snsId = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        ret = sensor_ctx_init(nPipeId);
        if (0 != ret) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return -1;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    sns_obj->sns_id = nPipeId;

    sns_obj->sns_i2c_obj.sns_i2c_fd = -1;
    sns_obj->sns_i2c_obj.slave_addr = IMX464_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = IMX464_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = IMX464_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = IMX464_SWAP_BYTES;

    sns_obj->sns_i2c_obj.sns_i2c_bnum = gImx464BusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

#if 0
    ret = imx464_get_chipid(nPipeId, &snsId);
    if (ret < 0) {
        SNS_ERR("can't find imx464 sensor id.\r\n");
        return ret;
    } else {
        SNS_DBG("Sensor: imx464 check chip id success.\r\n");
    }

#endif

    SNS_DBG("imx464 i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}


static void imx464_init(ISP_PIPE_ID nPipeId)
{
    AX_S32 nImagemode = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    /* 1. contex init */
    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        nRet = sensor_ctx_init(nPipeId);
        if (0 != nRet) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    /* 2. i2c init */
    sensor_i2c_init(nPipeId);

    /* 3. config settings  */
    nImagemode = sns_obj->eImgMode;
    imx464_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    imx464_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    imx464_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;

    sns_obj->sns_mode_obj.nVts = imx464_get_vmax(nPipeId);

    return;
}


static void imx464_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        SNS_ERR("os04a10_exit sns_obj null\n");
        return;
    }

    i2c_exit(sns_obj->sns_i2c_obj.sns_i2c_fd);

    sensor_ctx_exit(nPipeId);
    return;
}


static AX_S32 imx464_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    gImx464BusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 imx464_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    imx464_reset(nPipeId, gImx464BusInfo[nPipeId].I2cDev);

    return SNS_SUCCESS;
}

static AX_S32 imx464_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U16 sensor_id = 0;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= imx464_reg_read(nPipeId, 0x3107) << 8;
    sensor_id |= imx464_reg_read(nPipeId, 0x3108);

    SNS_DBG("%s: sensor imx464 id: 0x%x\n", __func__, sensor_id);

    if (sensor_id != IMX464_SENSOR_CHIP_ID) {
        SNS_ERR("%s: Failed to read sensor imx464 id\n", __func__);
        return -1;
    }
    return 0;
}

static AX_S32 imx464_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 imx464_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}


static AX_S32 imx464_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
{
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    params->nSnsMode_caps = AX_SNS_LINEAR_MODE | AX_SNS_HDR_2X_MODE | AX_SNS_HDR_3X_MODE;
    params->nSnsRawType_caps = AX_SNS_RAWTYPE_12BIT;
    params->nSnsFps_caps = AX_SNS_25FPS | AX_SNS_30FPS;
    params->nSnsResolution_caps = AX_SNS_RES_2688_1520;
    return SNS_SUCCESS;
}

static AX_S32 imx464_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
{
    AX_S32 ret = 0;
    AX_S32 width;
    AX_S32 height;
    AX_S32 hdrmode;
    AX_S32 fps;
    AX_S32 rawtype;
    AX_S32 sns_seting_index = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(sns_mode);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        ret = sensor_ctx_init(nPipeId);
        if (0 != ret) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return SNS_ERR_CODE_INIT_FAILD;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    sns_obj->bSyncInit = AX_FALSE;
    width = sns_mode->nWidth;
    height = sns_mode->nHeight;
    fps = sns_mode->nFrameRate;
    rawtype = sns_mode->eRawType;
    if ((AX_SNS_LINEAR_MODE != sns_mode->eSnsMode)
        && (AX_SNS_HDR_2X_MODE != sns_mode->eSnsMode)
        && (AX_SNS_HDR_3X_MODE != sns_mode->eSnsMode)) {
        hdrmode = AX_SNS_LINEAR_MODE;
    } else {
        hdrmode = sns_mode->eSnsMode;
    }

    if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && fps == 25 && rawtype == 10) {
        sns_seting_index = E_IMX464_ALLPIXEL_CSI2_4LANE_24MHZ_891MBPS_RGGB_2688x1520_10BIT_SDR_25FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && fps == 30 && rawtype == 10) {
        sns_seting_index = E_IMX464_ALLPIXEL_CSI2_4LANE_24MHZ_891MBPS_RGGB_2688x1520_10BIT_SDR_30FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && fps == 25 && rawtype == 12) {
        sns_seting_index = E_IMX464_ALLPIXEL_CSI2_4LANE_24MHZ_891MBPS_RGGB_2688x1520_12BIT_SDR_25FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && fps == 30 && rawtype == 12) {
        sns_seting_index = E_IMX464_ALLPIXEL_CSI2_4LANE_24MHZ_891MBPS_RGGB_2688x1520_12BIT_SDR_30FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && fps == 25 && rawtype == 10) {
        sns_seting_index = E_IMX464_ALLPIXEL_CSI2_4LANE_24MHZ_1188MBPS_RGGB_2688x1520_10BIT_HDR2DOL_25FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && fps == 30 && rawtype == 10) {
        sns_seting_index = E_IMX464_ALLPIXEL_CSI2_4LANE_24MHZ_1188MBPS_RGGB_2688x1520_10BIT_HDR2DOL_30FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && fps == 25 && rawtype == 12) {
        sns_seting_index = E_IMX464_WINDOWCROP_CSI2_4LANE_24MHZ_1188MBPS_RGGB_2688x1522_12BIT_HDR2DOL_25FPS;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && fps == 30 && rawtype == 12) {
        sns_seting_index = E_IMX464_WINDOWCROP_CSI2_4LANE_24MHZ_1188MBPS_RGGB_2688x1522_12BIT_HDR2DOL_30FPS;
    } else {
        SNS_ERR("it's not supported. mode:%d_%d_%d_%d_%d\n", width, height, hdrmode, fps, rawtype);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("sns_seting_index:%d, mode:%d_%d_%d_%d_%d\n", sns_seting_index, width, height, hdrmode, fps, rawtype);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = fps;
    sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel = sns_mode->eWithinBeyondfscSel;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return 0;
}

static AX_S32 imx464_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
{
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsMode);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        nRet = sensor_ctx_init(nPipeId);
        if (0 != nRet) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return -1;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    memcpy(pSnsMode, &sns_obj->sns_param.sns_dev_attr, sizeof(AX_SNS_ATTR_T));

    return SNS_SUCCESS;
}

static AX_S32 imx464_set_wdr_mode(ISP_PIPE_ID nPipeId, AX_SNS_HDR_MODE_E eHdrMode)
{
    return SNS_SUCCESS;
}
static AX_S32 imx464_stream_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 ret = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
        ret |= imx464_reg_write(nPipeId, 0x3002, 0x00);
        ret |= imx464_reg_write(nPipeId, 0x3000, 0x00);
    } else {
        ret |= imx464_reg_write(nPipeId, 0x3000, 0x01);
        ret |= imx464_reg_write(nPipeId, 0x3002, 0x01);
    }
    if (0 != ret) {
        return -1;
    }

    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return 0;
}

static AX_S32 imx464_mirror_flip(ISP_PIPE_ID nPipeId, AX_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
    switch (eSnsMirrorFlip) {
    default:
    case AX_SNS_MF_NORMAL:
        imx464_reg_write(nPipeId, 0x304E, 0x00);
        imx464_reg_write(nPipeId, 0x304F, 0x00);
        imx464_reg_write(nPipeId, 0x3080, 0x01);
        imx464_reg_write(nPipeId, 0x30AD, 0x02);
        imx464_reg_write(nPipeId, 0x30B6, 0x00);
        imx464_reg_write(nPipeId, 0x30B7, 0x00);
        imx464_reg_write(nPipeId, 0x3116, 0x02);
        break;

    case AX_SNS_MF_MIRROR:
        imx464_reg_write(nPipeId, 0x304E, 0x01);    //00h->01h
        break;

    case AX_SNS_MF_FLIP:
        imx464_reg_write(nPipeId, 0x304F, 0x01);    //00h->01h
        imx464_reg_write(nPipeId, 0x3080, 0xFF);    //01h->FFh
        imx464_reg_write(nPipeId, 0x30AD, 0x7E);    //02h->7Eh
        imx464_reg_write(nPipeId, 0x30B6, 0xFF);    //00h->FFh
        imx464_reg_write(nPipeId, 0x30B7, 0x01);    //00h->01h
        imx464_reg_write(nPipeId, 0x3116, 0x01);    //02h->01h
        break;

    case AX_SNS_MF_MIRROR_FLIP:
        imx464_reg_write(nPipeId, 0x304E, 0x01);
        imx464_reg_write(nPipeId, 0x304F, 0x01);
        imx464_reg_write(nPipeId, 0x3080, 0xFF);
        imx464_reg_write(nPipeId, 0x30AD, 0x7E);
        imx464_reg_write(nPipeId, 0x30B6, 0xFF);
        imx464_reg_write(nPipeId, 0x30B7, 0x01);
        imx464_reg_write(nPipeId, 0x3116, 0x01);
        break;
    }
    return SNS_SUCCESS;
}

static AX_S32 imx464_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("nPipeId:%d, testpattern on:%d\n", nPipeId, on);

    if (1 == on) {
        imx464_reg_write(nPipeId, 0x3148, 0x10);
        imx464_reg_write(nPipeId, 0x3280, 0x00);
        imx464_reg_write(nPipeId, 0x329c, 0x01);
        imx464_reg_write(nPipeId, 0x329e, 0x0a);
        imx464_reg_write(nPipeId, 0x3302, 0x00);
        imx464_reg_write(nPipeId, 0x336c, 0x00);
    } else {
        imx464_reg_write(nPipeId, 0x3148, 0x00);
        imx464_reg_write(nPipeId, 0x3280, 0x01);
        imx464_reg_write(nPipeId, 0x329c, 0x00);
        imx464_reg_write(nPipeId, 0x329e, 0x00);
        imx464_reg_write(nPipeId, 0x3302, 0x32);
        imx464_reg_write(nPipeId, 0x336c, 0x01);
    }

    return SNS_SUCCESS;
}


/****************************************************************************
 * get module default parameters function
 ****************************************************************************/
static AX_S32 imx464_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
{
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_SNS_HDR_MODE_E nHdrmode;

    SNS_CHECK_PTR_VALID(ptDftParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        nRet = sensor_ctx_init(nPipeId);
        if (0 != nRet) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return -1;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    memset(ptDftParam, 0, sizeof(AX_SENSOR_DEFAULT_PARAM_T));

    nHdrmode = sns_obj->sns_mode_obj.eHDRMode;

    switch (nHdrmode) {
    default:
    case AX_SNS_LINEAR_MODE:
        ptDftParam->ptDpc           = &dpc_param_sdr;
        ptDftParam->ptBlc           = &blc_param_sdr;
        ptDftParam->ptFpn           = &fpn_param_sdr;
        ptDftParam->ptDarkshading   = &ds_param_sdr;
        ptDftParam->ptGbl           = &gbl_param_sdr;
        ptDftParam->ptCac           = &cac_param_sdr;
        ptDftParam->ptDemosaic      = &demosaic_param_sdr;
        ptDftParam->ptCsc0          = &csc0_param_sdr;
        ptDftParam->ptCsc1          = &csc1_param_sdr;
        ptDftParam->ptGamma         = &gamma_param_sdr;
        ptDftParam->ptClc           = &clc_param_sdr;
        ptDftParam->ptPfr           = &pfr_param_sdr;
        ptDftParam->ptWbGain        = &wbGain_param_sdr;
        ptDftParam->ptRltm          = &rltm_param_sdr;
        ptDftParam->ptDehaze        = &dehaze_param_sdr;
        ptDftParam->ptNpu           = &npu_param_sdr;
        ptDftParam->ptLsc           = &lsc_param_sdr;
        ptDftParam->ptSharpen       = &lce_sharpen_param_sdr;
        ptDftParam->ptCset          = &lce_cset_param_sdr;
        ptDftParam->ptLumaNr        = &lce_luma_nr_param_sdr;
        ptDftParam->ptYcproc        = &lce_ycproc_param_sdr;
        ptDftParam->ptChromaNr      = &lce_chroma_nr_param_sdr;
        ptDftParam->ptYcrt          = &lce_ycrt_param_sdr;
        ptDftParam->ptWnr           = &wnr_param_sdr;
        ptDftParam->ptEis           = &eis_param_sdr;
        ptDftParam->ptAeDftParam    = &ae_param_sdr;
        ptDftParam->ptAwbDftParam   = &awb_param_sdr;
        break;
    case AX_SNS_HDR_2X_MODE:
        ptDftParam->ptDpc           = &dpc_param_hdr_2x;
        ptDftParam->ptBlc           = &blc_param_hdr_2x;
        ptDftParam->ptFpn           = &fpn_param_hdr_2x;
        ptDftParam->ptDarkshading   = &ds_param_hdr_2x;
        ptDftParam->ptGbl           = &gbl_param_hdr_2x;
        ptDftParam->ptCac           = &cac_param_hdr_2x;
        ptDftParam->ptDemosaic      = &demosaic_param_hdr_2x;
        ptDftParam->ptCsc0          = &csc0_param_hdr_2x;
        ptDftParam->ptCsc1          = &csc1_param_hdr_2x;
        ptDftParam->ptGamma         = &gamma_param_hdr_2x;
        ptDftParam->ptClc           = &clc_param_hdr_2x;
        ptDftParam->ptPfr           = &pfr_param_hdr_2x;
        ptDftParam->ptWbGain        = &wbGain_param_hdr_2x;
        ptDftParam->ptRltm          = &rltm_param_hdr_2x;
        ptDftParam->ptDehaze        = &dehaze_param_hdr_2x;
        ptDftParam->ptNpu           = &npu_param_hdr_2x;
        ptDftParam->ptLsc           = &lsc_param_hdr_2x;
        ptDftParam->ptSharpen       = &lce_sharpen_param_hdr_2x;
        ptDftParam->ptCset          = &lce_cset_param_hdr_2x;
        ptDftParam->ptLumaNr        = &lce_luma_nr_param_hdr_2x;
        ptDftParam->ptYcproc        = &lce_ycproc_param_hdr_2x;
        ptDftParam->ptChromaNr      = &lce_chroma_nr_param_hdr_2x;
        ptDftParam->ptYcrt          = &lce_ycrt_param_hdr_2x;
        ptDftParam->ptWnr           = &wnr_param_hdr_2x;
        ptDftParam->ptEis           = &eis_param_hdr_2x;
        ptDftParam->ptAeDftParam    = &ae_param_hdr_2x;
        ptDftParam->ptAwbDftParam   = &awb_param_hdr_2x;
        break;
    case AX_SNS_HDR_3X_MODE:
        /* TODO: Users configure their own default parameters */
        break;

    case AX_SNS_HDR_4X_MODE:
        /* TODO: Users configure their own default parameters */
        break;
    }

    return SNS_SUCCESS;
}

static AX_S32 imx464_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level of linear mode */
    if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        ptBlackLevel->nBlackLevel[0] = 800;
        ptBlackLevel->nBlackLevel[1] = 800;
        ptBlackLevel->nBlackLevel[2] = 800;
        ptBlackLevel->nBlackLevel[3] = 800;
    } else {
        ptBlackLevel->nBlackLevel[0] = 800;
        ptBlackLevel->nBlackLevel[1] = 800;
        ptBlackLevel->nBlackLevel[2] = 800;
        ptBlackLevel->nBlackLevel[3] = 800;
    }

    return SNS_SUCCESS;
}


/****************************************************************************
 * exposure control function
 ****************************************************************************/

AX_S32 imx464_set_exp_limit(ISP_PIPE_ID nPipeId, AX_F32 fExpRatio, AX_U32 nVmax)
{
    AX_U32 fsc = 0;
    AX_F32 line_period = 0;
    AX_U32 line_period_fp = 0;

    AX_U32 shr0_min = 0, shr0_max = 0;
    AX_U32 shr1_min = 0, shr1_max = 0;
    AX_U32 rhs1_min = 0, rhs1_max = 0;

    AX_U32 exp_max_line = 0;
    AX_U32 lef_max_line = 0;
    AX_U32 sef1_max_line = 0;

    AX_F32 ratio = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    line_period = gImx464Params[nPipeId].line_period;
    line_period_fp = gImx464Params[nPipeId].line_period_fp;

    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = nVmax;
        shr0_min = 3;
        shr0_max = fsc - 1;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = 2 * nVmax;
        if(fsc % 4 != 0){
            SNS_ERR("hdr fsc:%d is invalid!\n", fsc);
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }

        /* SHR1_MIN */
        shr1_min = 9;
        exp_max_line = fsc - shr1_min - 9;

        /* SHR0_MIN */
        /* 1)ratio=lef/sef  2)shr0 > rhs1+9     3)2n */
        lef_max_line = exp_max_line * (fExpRatio / (1 + fExpRatio));
        SNS_DBG("lef_max_line:%d\n", lef_max_line);

        shr0_min = fsc - lef_max_line;
        SNS_DBG("limit0 shr0_min:%d\n", shr0_min);

        shr0_min = AXSNS_ALIGN_UP(shr0_min, 2);
        SNS_DBG("limitn shr0_min:%d\n", shr0_min);

        /* SHR0_MAX */
        shr0_max = AXSNS_ALIGN_DOWN(fsc - 2, 2);
        SNS_DBG("limitn shr0_max:%d\n", shr0_max);

        /* RHS1_MAX */
        /* 1)ratio=lef/sef  2)RHS1 ≤ (SHR0-9)   3)RHS1 < (2*BRL)
            4)Within FSC: RHS1 < FSC-2*BRL  5)Within FSC: linecap <= IMX464_L2S_LINEGAP_MAX  6)4n+1 */
        sef1_max_line = lef_max_line / fExpRatio;
        SNS_DBG("sef1_max_line:%d\n", sef1_max_line);

        rhs1_max = sef1_max_line + shr1_min;
        SNS_DBG("limit0 rhs1_max:%d\n", rhs1_max);

        rhs1_max = AXSNS_MIN(rhs1_max, shr0_min - 9);
        SNS_DBG("limit1 rhs1_max:%d\n", rhs1_max);

        rhs1_max = AXSNS_MIN(rhs1_max, 2 * IMX464_ALLPIXEL_BRL - 1);   // less than use max-1
        SNS_DBG("limit2 rhs1_max:%d\n", rhs1_max);

        if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
            rhs1_max = AXSNS_MIN((AX_S32)rhs1_max, (AX_S32)(fsc - 2 * IMX464_ALLPIXEL_BRL - 1) - 32);
            SNS_DBG("limit3 rhs1_max:%d\n", rhs1_max);

            rhs1_max = AXSNS_MIN(rhs1_max, (IMX464_L2S_LINEGAP_MAX - 1) * 2 + 3);
            SNS_DBG("limit4 rhs1_max:%d\n", rhs1_max);
        }

        if(0 == rhs1_max % 4){
            rhs1_max = rhs1_max - 3;   /* 4n -> 4(n-1)+1 */
        } else {
            rhs1_max = AXSNS_ALIGN_DOWN(rhs1_max, 4) + 1;   /* 4n+1/4n+2/4n+3 -> 4n+1 */
        }
        SNS_DBG("limitn rhs1_max:%d\n", rhs1_max);
        gImx464Params[nPipeId].rhs1_max = rhs1_max;

        /* SHR1_MAX */
        /* 1)2n+1   2)shr1 <= rhs1-2 */
        shr1_max = AXSNS_ALIGN_DOWN(rhs1_max - 2, 2) + 1;
        SNS_DBG("limitn shr1_max:%d\n", shr1_max);

    } else {
        SNS_ERR("not support hdr mode:%d\n", sns_obj->sns_mode_obj.eHDRMode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = (fsc - shr0_max) * line_period_fp;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (fsc - shr0_min) * line_period_fp;

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (rhs1_max - shr1_max) * line_period_fp;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (rhs1_max - shr1_min) * line_period_fp;

        ratio = (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] /
            sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX];
    }

    sns_obj->sns_param.sns_ae_limit.nInitMaxIntegrationTime =
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_limit.nInitMinIntegrationTime =
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX];

    SNS_DBG("userRatio:%.2f, userVts:0x%x, limitRatio:%.2f, limitExp:%.2f-%.2f-%.2f-%.2f, limitExpLine:%d-%d-%d-%d\n",
        fExpRatio, nVmax, ratio,
        (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024,
        fsc - shr0_min, fsc - shr0_max, rhs1_max - shr1_min, rhs1_max - shr1_max);

    return 0;
}

static AX_S32 imx464_set_aecparam(ISP_PIPE_ID nPipeId)
{
    AX_U32 vmax = 0, hmax = 0;
    AX_U32 fsc = 0;
    AX_U32 shr0 = 0;
    AX_U32 shr1 = 0;
    AX_U32 rhs1 = 0;
    AX_U32 line_period_fp = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    vmax = imx464_get_vmax(nPipeId);
    hmax = imx464_get_hmax(nPipeId);
    shr0 = imx464_get_shr0(nPipeId);
    shr1 = imx464_get_shr1(nPipeId);
    rhs1 = imx464_get_rhs1(nPipeId);

    SNS_DBG("get vmax:0x%x, hmax:0x%x, shr0:0x%x, shr1:0x%x, rhs1:0x%x\n", vmax, hmax, shr0, shr1, rhs1);

    gImx464Params[nPipeId].hmax = hmax;
    gImx464Params[nPipeId].vmax = vmax;
    gImx464Params[nPipeId].line_period = hmax / IMX464_INCLK_74POINT25M;
    gImx464Params[nPipeId].line_period_fp = ax_float_convert_to_int(gImx464Params[nPipeId].line_period, 22, 10, 0);
    line_period_fp = gImx464Params[nPipeId].line_period_fp;

    /******************integration time limit*****************/
    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = gImx464Params[nPipeId].vmax;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = 2 * gImx464Params[nPipeId].vmax;
        if(fsc % 4 != 0){
            SNS_ERR("hdr fsc:%d is invalid!\n", fsc);
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }
    } else {
        SNS_ERR("not support hdr mode:%d\n", sns_obj->sns_mode_obj.eHDRMode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    sns_obj->sns_param.sns_ae_limit.nMinratio = IMX464_MIN_RATIO;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = IMX464_MAX_RATIO;
    imx464_set_exp_limit(nPipeId, IMX464_MAX_RATIO, gImx464Params[nPipeId].vmax);
    gImx464Params[nPipeId].pre_ratio = IMX464_MAX_RATIO;
    gImx464Params[nPipeId].pre_vmax = gImx464Params[nPipeId].vmax;

    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = 1 * line_period_fp;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = 2 * line_period_fp;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_MEDIUM_FRAME_IDX] = 2 * line_period_fp;
    }

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = (fsc - shr0) * line_period_fp;
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (rhs1 - shr1) * line_period_fp;

    /******************gain limit*****************/
    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_SEPARATION;

    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = pow(10, 0);
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = pow(10, (float)72 / 20);
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = pow(10, (float)0.3 / 20);

    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = pow(10, 0);
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = pow(10, 0);
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = pow(10, (float)0.3 / 20);

    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX] = pow(10, 0);
        sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX] = pow(10, (float)72 / 20);
        sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_MEDIUM_FRAME_IDX] = pow(10, (float)0.3 / 20);

        sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX] = pow(10, 0);
        sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX] = pow(10, 0);
        sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_MEDIUM_FRAME_IDX] = pow(10, (float)0.3 / 20);

        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    }

    sns_obj->sns_param.sns_ae_limit.nMingain = sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_limit.nMaxgain = sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (float)1/256;  //U10.8

    /******************lcg2hcg ratio*****************/
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = pow(10, (float)8.3 / 20); //2.6

    /******************current fps*****************/
    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return 0;
}

/* note: integration time unit : us */
static AX_S32 imx464_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U32 vmax = 0;
    AX_U32 hmax = 0;
    AX_U32 fsc = 0;
    AX_U32 shr0 = 0;
    AX_U32 shr1 = 0;
    AX_U32 rhs1 = 0;
    AX_S32 ret = 0;
    AX_F32 ratio = 0.0f;
    AX_U32 nIntTimeFromUser = 0;
    AX_U32 line_period_fp = 0;
    AX_F32 lef = 0;
    AX_F32 sef1 = 0;

    AX_F32 lef_line = 0;
    AX_F32 sef1_line = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptIntTimeTbl);

    SNS_DBG("userExptime:%.2f-%.2f-%.2f-%.2f, Hdrratio:%f-%f-%f-%f\n",
            (float)ptIntTimeTbl->nIntTime[0]/1024, (float)ptIntTimeTbl->nIntTime[1]/1024,
            (float)ptIntTimeTbl->nIntTime[2]/1024, (float)ptIntTimeTbl->nIntTime[3]/1024,
            ptIntTimeTbl->nHdrRatio[0], ptIntTimeTbl->nHdrRatio[1], ptIntTimeTbl->nHdrRatio[2], ptIntTimeTbl->nHdrRatio[3]);

    vmax = imx464_get_vmax(nPipeId);
    hmax = imx464_get_hmax(nPipeId);
    shr0 = imx464_get_shr0(nPipeId);
    shr1 = imx464_get_shr1(nPipeId);
    rhs1 = imx464_get_rhs1(nPipeId);

    SNS_DBG("get vmax:0x%x, hmax:0x%x, shr0:0x%x, shr1:0x%x, rhs1:0x%x\n", vmax, hmax, shr0, shr1, rhs1);

    line_period_fp = gImx464Params[nPipeId].line_period_fp;

    if ((gImx464Params[nPipeId].vmax == 0U) ||
        (sns_obj->sns_mode_obj.nFrameRate == 0U) ||
        (line_period_fp == 0U)) {
        SNS_ERR("param == 0 (Division by zero !!!)\n");
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = gImx464Params[nPipeId].vmax;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = 2 * gImx464Params[nPipeId].vmax;
    } else {
        SNS_ERR("not support hdr mode:%d\n", sns_obj->sns_mode_obj.eHDRMode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ratio = ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX];
    ratio = AXSNS_CLIP3(ratio, sns_obj->sns_param.sns_ae_limit.nMinratio, sns_obj->sns_param.sns_ae_limit.nMaxratio);

    if(ratio != gImx464Params[nPipeId].pre_ratio || gImx464Params[nPipeId].vmax != gImx464Params[nPipeId].pre_vmax) {
        SNS_DBG("pre_ratio:%.2f, ratio:%.2f, pre_vmax:0x%x, vmax:0x%x\n",
            gImx464Params[nPipeId].pre_ratio, ratio, gImx464Params[nPipeId].pre_vmax, gImx464Params[nPipeId].vmax);
        imx464_set_exp_limit(nPipeId, ratio, gImx464Params[nPipeId].vmax);
        gImx464Params[nPipeId].pre_ratio = ratio;
        gImx464Params[nPipeId].pre_vmax = gImx464Params[nPipeId].vmax;
    }

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
        nIntTimeFromUser = nIntTimeFromUser / AXSNS_DIV_0_TO_1_FLOAT(ratio);
        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]);

        if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] == nIntTimeFromUser) {
            SNS_DBG("SEF1 nIntTimeFromUser:%.2f new and current is equal.\n", (float)nIntTimeFromUser/1024);
        } else {
            SNS_DBG("SEF1 nIntTimeFromUser:%.2f\n", (float)nIntTimeFromUser/1024);

            rhs1 = gImx464Params[nPipeId].rhs1_max;
            SNS_DBG("limit0 rhs1:0x%x\n", rhs1);

            sef1_line = (AX_F32)nIntTimeFromUser / line_period_fp;
            SNS_DBG("sef1_line:%.2f\n", sef1_line);

            shr1 = rhs1 - (AX_U32)sef1_line;
            SNS_DBG("limit0 shr1:0x%x\n", shr1);

            shr1 = AXSNS_MAX(shr1, 9);
            SNS_DBG("limit1 shr1:0x%x\n", shr1);

            /* shr1 2n+1 */
            shr1 = AXSNS_ALIGN_DOWN(shr1, 2) + 1;
            SNS_DBG("limitn shr1:0x%x\n", shr1);

            ret = imx464_set_shr1(nPipeId, shr1);
            ret = imx464_set_rhs1(nPipeId, rhs1);
        }
    }

    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] == nIntTimeFromUser ) {
        SNS_DBG("LEF nIntTimeFromUser:%.2f new and current is equal.\n", (float)nIntTimeFromUser/1024);
    } else {
        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

        lef_line = (AX_F32)nIntTimeFromUser / line_period_fp;
        SNS_DBG("lef_line:%.2f\n", lef_line);

        shr0 = fsc - (AX_U32)lef_line;
        SNS_DBG("limit0 shr0:0x%x\n", shr0);

        if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            shr0 = AXSNS_ALIGN_DOWN(shr0, 2);
            SNS_DBG("limitn shr0:0x%x\n", shr0);
        }

        ret = imx464_set_shr0(nPipeId, shr0);
    }

    lef = (fsc - shr0) * line_period_fp;
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = lef;
    sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX] = lef;

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sef1 = (rhs1 - shr1) * line_period_fp;
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = sef1;
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_MEDIUM_FRAME_IDX] = sef1;
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = IMX464_L2S_LINEGAP_CALC(rhs1);
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGapTime[HDR_LONG_FRAME_IDX] =
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] * gImx464Params[nPipeId].line_period + 1;

        /* Calculate the true effective exposure ratio */
        ratio = lef / AXSNS_DIV_0_TO_1_FLOAT(sef1);
    }

    SNS_DBG("set vmax:0x%x, hmax:0x%x, shr0:0x%x, shr1:0x%x, rhs1:0x%x\n",
            gImx464Params[nPipeId].vmax, gImx464Params[nPipeId].hmax, shr0, shr1, rhs1);

    SNS_DBG("current_ratio=%.2f, line_period=%.2f, LineGap[0]=%d, ExpTime=%.2f-%.2f\n",
            ratio, gImx464Params[nPipeId].line_period,
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX],
            (float)sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX]/1024,
            (float)sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_MEDIUM_FRAME_IDX]/1024);

    return 0;
}

static AX_S32 imx464_get_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    //now nothing to do
    return 0;
}

static AX_S32 imx464_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 Gainh;
    AX_U8 Gainl;
    float LocalNewGain = 0;
    AX_S32 ret = 0;
    AX_F32 local_lgain = 0.0f ;
    AX_F32 fGainFromUser = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptAnalogGainTbl);

    SNS_DBG("userAgain:%f-%f\n", ptAnalogGainTbl->nGain[0], ptAnalogGainTbl->nGain[1]);

    /* long gain  seting */
    fGainFromUser = ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX];
    fGainFromUser = AXSNS_CLIP3(fGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] - fGainFromUser) < EPS) {
        //SNS_DBG("LEF fGainFromUser:%f new and current is equal.\n", fGainFromUser);
    } else {
        LocalNewGain = 20 * log10(fGainFromUser);
        LocalNewGain = LocalNewGain * (float)10 / 3;

        Gainh = ((AX_U8)LocalNewGain & 0x700) >> 8;
        Gainl = ((AX_U8)LocalNewGain & 0x0FF);

        ret |= imx464_sns_update_regs_table(nPipeId, IMX464_GAIN_L, Gainl);
        ret |= imx464_sns_update_regs_table(nPipeId, IMX464_GAIN_H, Gainh);

        if (ret != 0) {
            SNS_ERR("LEF fGainFromUser:%f update fail.", fGainFromUser);
            return ret;
        }

        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = fGainFromUser;
    }

    /* medium gain  seting */
    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        fGainFromUser = AXSNS_CLIP3(fGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] - fGainFromUser) < EPS) {
            //SNS_DBG("SEF1 fGainFromUser:%f new and current is equal.\n", fGainFromUser);
        } else {
            LocalNewGain = 20 * log10(fGainFromUser);
            LocalNewGain = LocalNewGain * (float)10 / 3;

            Gainh = ((AX_U8)LocalNewGain & 0x700) >> 8;
            Gainl = ((AX_U8)LocalNewGain & 0x0FF);

            ret |= imx464_sns_update_regs_table(nPipeId, IMX464_GAIN1_L, Gainl);
            ret |= imx464_sns_update_regs_table(nPipeId, IMX464_GAIN1_H, Gainh);

            if (ret != 0) {
                SNS_ERR("SEF1 fGainFromUser:%f update fail.", fGainFromUser);
                return ret;
            }
            sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = fGainFromUser;
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 imx464_get_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 imx464_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_F32 fGainFromUser = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptDigitalGainTbl);

    SNS_DBG("userDgain:%f-%f\n", ptDigitalGainTbl->nGain[0], ptDigitalGainTbl->nGain[1]);

    fGainFromUser = ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX];
    fGainFromUser = AXSNS_CLIP3(fGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] - fGainFromUser) < EPS) {
        //SNS_DBG("LEF userDgain:%f new and current is equal.\n", fGainFromUser);
    } else {
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = fGainFromUser;
    }

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fGainFromUser = ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
        fGainFromUser = AXSNS_CLIP3(fGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX]);
        if (fabs(sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] - fGainFromUser) < EPS) {
            //SNS_DBG("SEF1 fGainFromUser:%f new and current is equal.\n", fGainFromUser);
        } else {
            sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = fGainFromUser;
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 imx464_get_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 imx464_get_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i = 0;
    AX_S32 ret = 0;
    int d_max = 0;
    int d_min = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    // value <=> dB:  dB = 20* log10(value),    value = 10 ^ (dB/20)
    // dB <=> d:      d = dB * (10/3) ,         dB = d * (3/10)
    // value <=>d :   d = (200/3) log10(value), value = 10 ^(d * (3/200))
    d_max = round(log10(sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3);
    d_min = round(log10(sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3);
    params->nAgainTableSize = d_max - d_min + 1;
    for (int i = d_min; i < (d_max + 1); i++) {
        nAgainTable[i] = pow(10, i * (float)3 / 200);
    }

    d_max = round(log10(sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3);
    d_min = round(log10(sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3);
    params->nDgainTableSize = d_max - d_min + 1;
    for (int i = d_min; i < (d_max + 1); i++) {
        nDgainTable[i] = pow(10, i * (float)3 / 200);
    }

    params->pAgainTable = nAgainTable;
    params->pDgainTable = nDgainTable;

    return ret;
}

static AX_S32 imx464_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 ret = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    ret = imx464_get_gain_table(nPipeId, params);

    return ret;
}


static AX_S32 imx464_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    AX_U8 hcglcg_value = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_HCG_MODE) && (nSnsHcgLcg == AX_LCG_MODE)) {

        imx464_sns_update_regs_table(nPipeId, IMX464_FDG_SEL, 0x00);
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_LCG_MODE;

    } else if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_LCG_MODE) && (nSnsHcgLcg == AX_HCG_MODE)) {

        imx464_sns_update_regs_table(nPipeId, IMX464_FDG_SEL, 0x01);
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_HCG_MODE;
    } else {
        /*SNS_DBG("current hcg/lcg mode : %d, Mode of ae library : %d \n", nSnsHcgLcg,
                sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg);*/
    }

    return SNS_SUCCESS;
}


static AX_S32 imx464_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
{
    AX_U32 vmax = 0;
    AX_U32 fsc = 0;
    AX_S32 nRet = 0;
    AX_F32 line_period = 0.0;   /* unit : us */

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if(nFps < EPS){
        SNS_ERR("userFps:%f < %f is Invalid.\n", nFps, EPS);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    line_period = gImx464Params[nPipeId].line_period;
    if ((AX_U32)nFps >= sns_obj->sns_mode_obj.nFrameRate) {
        SNS_WRN("userFps:%d > nFrameRate:%d, use fps:%d\n",
            (AX_U32)nFps, sns_obj->sns_mode_obj.nFrameRate, sns_obj->sns_mode_obj.nFrameRate);
        vmax = sns_obj->sns_mode_obj.nVts;
    } else {
        vmax = 1 * SNS_1_SECOND_UNIT_US / (line_period * nFps);
        if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            vmax = vmax / 2;
        }
    }

    if (vmax > IMX464_MAX_VMAX) {
        SNS_WRN("userFps:%.2f, line_period:%.2f, vmax:0x%x beyond max:0x%x, use max vmax.\n",
            nFps, line_period, vmax, IMX464_MAX_VMAX);
        vmax = IMX464_MAX_VMAX;
    }

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        vmax = AXSNS_ALIGN_DOWN(vmax, 2);
    }
    nRet = imx464_set_vmax(nPipeId, vmax);
    gImx464Params[nPipeId].vmax = vmax;

    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = vmax;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        fsc = 2 * vmax;
    } else {
        SNS_ERR("not support hdr mode:%d\n", sns_obj->sns_mode_obj.eHDRMode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    sns_obj->sns_param.sns_ae_param.nCurFps = 1 * SNS_1_SECOND_UNIT_US / (line_period * fsc);

    SNS_DBG("userFps:%.2f, nCurFps:%.2f, line_period:%.2f, vmax:0x%x\n",
            nFps, sns_obj->sns_param.sns_ae_param.nCurFps, line_period, vmax);

    return SNS_SUCCESS;
}

static AX_S32 imx464_ae_get_sensor_slow_shutter_param(ISP_PIPE_ID nPipeId,
        AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
{
    AX_S32 framerate = 30;
    AX_U32 nFps = 0;
    AX_U32 nVts = 0;
    AX_U32 shr0_min = 0;
    AX_U32 shr1_min = 0;
    AX_U32 fsc = 0;
    AX_F32 fSensorMinFps = 0;
    AX_F32 line_period = 0.0;   /* unit : us */
    AX_U32 line_period_fp = 0;
    AX_U32 exp_max_line = 0;
    AX_U32 lef_max_line = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    framerate = sns_obj->sns_mode_obj.nFrameRate;
    if (framerate > SNS_MAX_FRAME_RATE) {
        SNS_ERR("framerate:%d out of range:%d\n", framerate, SNS_MAX_FRAME_RATE);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    line_period = gImx464Params[nPipeId].line_period;
    line_period_fp = gImx464Params[nPipeId].line_period_fp;

    if(ax_sns_is_zero(line_period)) {
        SNS_ERR("line_period is zero !\n");
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ptSlowShutterParam->nGroupNum = AXSNS_MIN((sizeof(gFpsGear) / sizeof(AX_F32)), framerate);
    ax_sns_quick_sort_float(gFpsGear, ptSlowShutterParam->nGroupNum);
    fSensorMinFps = 1 * SNS_1_SECOND_UNIT_US / (line_period * IMX464_MAX_VMAX);
    ptSlowShutterParam->fMinFps = AXSNS_MAX(gFpsGear[0], fSensorMinFps);

    for (nFps = 0; nFps < ptSlowShutterParam->nGroupNum; nFps++) {
        if((AX_S32)gFpsGear[nFps] >= framerate) {
            SNS_WRN("userFps:%d >= framerate:%d, use fps:%d\n", (AX_S32)gFpsGear[nFps], framerate, framerate);
            nVts = sns_obj->sns_mode_obj.nVts;
        } else {
            nVts = 1 * SNS_1_SECOND_UNIT_US / (line_period * gFpsGear[nFps]);
            if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
                nVts = nVts / 2;
            }
        }

        if (nVts > IMX464_MAX_VMAX){
            SNS_WRN("userFps:%f, line_period:%.2f, nVts:0x%x > max_vmax:0x%x, use max_vmax.\n",
                gFpsGear[nFps], line_period, nVts, IMX464_MAX_VMAX);
            nVts = IMX464_MAX_VMAX;
        }

        if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            fsc = nVts;
            shr0_min = 3;
        } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            nVts = AXSNS_ALIGN_DOWN(nVts, 2);
            fsc = 2 * nVts;
            shr1_min = 9;

            /* SHR0_MIN */
            /* 1)ratio=lef/sef  2)shr0 > rhs1+9     3)2n */
            exp_max_line = fsc - shr1_min - 9;
            lef_max_line = exp_max_line * (IMX464_MAX_RATIO / (1 + IMX464_MAX_RATIO));
            shr0_min = fsc - lef_max_line;
            shr0_min = AXSNS_ALIGN_UP(shr0_min, 2);
        } else {
            SNS_ERR("not support hdr mode:%d\n", sns_obj->sns_mode_obj.eHDRMode);
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }

        ptSlowShutterParam->tSlowShutterTbl[nFps].nMaxIntTime = (fsc - shr0_min) * line_period_fp;
        ptSlowShutterParam->tSlowShutterTbl[nFps].fRealFps = 1 * SNS_1_SECOND_UNIT_US / (line_period * fsc);
        ptSlowShutterParam->fMaxFps = ptSlowShutterParam->tSlowShutterTbl[nFps].fRealFps;

        SNS_DBG("userFps:%.2f, fRealFps:%.2f, line_period:%.2f, vts:0x%x, ratio:%.2f, nMaxIntTime:%.2f\n", gFpsGear[nFps],
                ptSlowShutterParam->tSlowShutterTbl[nFps].fRealFps, line_period, nVts, IMX464_MAX_RATIO,
                (float)ptSlowShutterParam->tSlowShutterTbl[nFps].nMaxIntTime/1024);
    }

    SNS_DBG("nGroupNum:%d, fMinFps:%.2f, fMaxFps:%.2f, fSensorMinFps:%.2f, nFrameRate:%d\n",
            ptSlowShutterParam->nGroupNum, ptSlowShutterParam->fMinFps, ptSlowShutterParam->fMaxFps,
            fSensorMinFps, sns_obj->sns_mode_obj.nFrameRate);

    return SNS_SUCCESS;
}

AX_SENSOR_REGISTER_FUNC_T gSnsimx464Obj = {
    /* sensor ctrl */
    .pfn_sensor_reset                   = imx464_sensor_reset,
    .pfn_sensor_chipid                  = imx464_get_chipid,
    .pfn_sensor_init                    = imx464_init,
    .pfn_sensor_exit                    = imx464_exit,
    .pfn_sensor_streaming_ctrl          = imx464_stream_ctrl,
    .pfn_sensor_testpattern             = imx464_testpattern_ctrl,
    .pfn_sensor_mirror_flip             = imx464_mirror_flip,

    .pfn_sensor_set_mode                = imx464_set_mode,
    .pfn_sensor_get_mode                = imx464_get_mode,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = imx464_set_bus_info,
    .pfn_sensor_write_register          = imx464_reg_write,
    .pfn_sensor_read_register           = imx464_reg_read,

    /* default param */
    .pfn_sensor_get_default_params      = imx464_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = imx464_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = imx464_set_sensor_params,
    .pfn_sensor_get_params              = imx464_get_sensor_params,
    .pfn_sensor_get_gain_table          = imx464_get_sensor_gain_table,
    .pfn_sensor_set_again               = imx464_set_again,
    .pfn_sensor_set_dgain               = imx464_set_dgain,
    .pfn_sensor_get_again               = imx464_get_again,
    .pfn_sensor_get_dgain               = imx464_get_dgain,
    .pfn_sensor_set_integration_time    = imx464_set_integration_time,
    .pfn_sensor_get_integration_time    = imx464_get_integration_time,
    .pfn_sensor_hcglcg_ctrl             = imx464_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = imx464_set_fps,
    .pfn_sensor_get_slow_shutter_param  = imx464_ae_get_sensor_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = imx464_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = NULL,
};

