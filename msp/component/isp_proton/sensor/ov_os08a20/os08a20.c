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
#include "os08a20_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "os08a20_sdr.h"
#include "os08a20_hdr_2x.h"


/****************************************************************************
 * golbal variables  and macro definition
 ****************************************************************************/
SNSOS08A20_OBJ sns_os08a20params[DEF_VIN_PIPE_MAX_NUM];
AX_SNS_COMMBUS_T gOs08a20BusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0}};

SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {NULL};

#define OS08A20_MAX_VTS         (0xFFFF)
#define OS08A20_MAX_RATIO       (16.0f)
#define OS08A20_MIN_RATIO       (5.0f)

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = NULL)

/*user config*/
static AX_F32 gFpsGear[] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00, 11.00, 12.00, 13.00, 14.00, 15.00,
                        16.00, 17.00, 18.00, 19.00, 20.00, 21.00, 22.00, 23.00, 24.00, 25.00, 26.00, 27.00, 28.00, 29.00, 30.00};

static AX_S32 os08a20_set_aecparam(ISP_PIPE_ID nPipeId);

/****************************************************************************
 * Internal function definition
 ****************************************************************************/
static AX_S32 sensor_ctx_init(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = NULL;
    AX_S32 ret = 0;

    SNS_DBG("os08a20 sensor_ctx_init. ret = %d\n", ret);

    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);

    if (NULL == sns_obj) {
        sns_obj = (SNS_STATE_OBJ *)calloc(1, sizeof(SNS_STATE_OBJ));
        if (NULL == sns_obj) {
            SNS_ERR("malloc sns_ctx failed\r\n");
            return -1;
        }
    }

    memset(sns_obj, 0, sizeof(SNS_STATE_OBJ));

    SENSOR_SET_CTX(nPipeId, sns_obj);

    return SNS_SUCCESS;
}

static AX_VOID sensor_ctx_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = NULL;

    SENSOR_GET_CTX(nPipeId, sns_obj);
    free(sns_obj);
    SENSOR_RESET_CTX(nPipeId);
}

/****************************************************************************
 * sensor control function
 ****************************************************************************/
static AX_S32 os08a20_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    os08a20_reset(nPipeId);

    return SNS_SUCCESS;
}

static AX_S32 os08a20_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U32 sensor_id = 0;
    AX_U16 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= os08a20_reg_read(nPipeId, 0x300A) << 16;
    sensor_id |= os08a20_reg_read(nPipeId, 0x300B) << 8;
    sensor_id |= os08a20_reg_read(nPipeId, 0x300C);

    SNS_DBG("%s: sensor os08a20 id: 0x%x\n", __func__, sensor_id);

    if (sensor_id != OS08A20_SENSOR_CHIP_ID) {
        SNS_ERR("%s: Failed to read sensor os08a20 id\n", __func__);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    *pSnsId = sensor_id;

    return SNS_SUCCESS;
}

static AX_S32 sensor_i2c_init(ISP_PIPE_ID nPipeId)
{
    AX_S32 ret = 0;
    SNS_STATE_OBJ *sns_obj = NULL;
    AX_S32 snsId = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (NULL == sns_obj) {
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
    sns_obj->sns_i2c_obj.slave_addr = OS08A20_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = OS08A20_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = OS08A20_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = OS08A20_SWAP_BYTES;

    sns_obj->sns_i2c_obj.sns_i2c_bnum = gOs08a20BusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

    ret = os08a20_get_chipid(nPipeId, &snsId);
    if (ret !=  SNS_SUCCESS) {
        SNS_ERR("can't find os08a20 sensor id ret=%d.\r\n", ret);
        return ret;
    } else {
        SNS_DBG("Sensor: os08a20 check chip id success.\r\n");
    }

    SNS_DBG("os08a20 i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}

AX_S32 os08a20_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
{
    AX_U8 data;
    SNS_STATE_OBJ *sns_obj = NULL;
    AX_S32 nRet = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (-1 == sns_obj->sns_i2c_obj.sns_i2c_fd)
        return -1;

    i2c_read(sns_obj->sns_i2c_obj.sns_i2c_fd,
             sns_obj->sns_i2c_obj.slave_addr,
             addr,
             sns_obj->sns_i2c_obj.address_byte,
             (AX_U8 *)(&data),
             sns_obj->sns_i2c_obj.data_byte,
             sns_obj->sns_i2c_obj.swap_byte);

    return data;
}

AX_S32 os08a20_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
{
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (-1 == sns_obj->sns_i2c_obj.sns_i2c_fd) {
        return -1;
    }

    nRet = i2c_write(sns_obj->sns_i2c_obj.sns_i2c_fd, sns_obj->sns_i2c_obj.slave_addr, addr,
                     sns_obj->sns_i2c_obj.address_byte,
                     (AX_U8 *)(&data), sns_obj->sns_i2c_obj.data_byte, sns_obj->sns_i2c_obj.swap_byte);

    return nRet;
}

static void os08a20_init(ISP_PIPE_ID nPipeId)
{
    AX_S32 nImagemode = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = NULL;

    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    /* 1. contex init */
    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (NULL == sns_obj) {
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
    os08a20_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    os08a20_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    os08a20_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;
    sns_obj->sns_mode_obj.nVts = os08a20_get_vts(nPipeId);

    return;
}

static void os08a20_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = NULL;
    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (NULL == sns_obj) {
        SNS_ERR("os08a20_exit sns_obj null\n");
        return;
    }

    i2c_exit(sns_obj->sns_i2c_obj.sns_i2c_fd);

    sensor_ctx_exit(nPipeId);
    return;
}

static AX_S32 os08a20_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    gOs08a20BusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 os08a20_sensor_streaming_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
        result = os08a20_reg_write(nPipeId, 0x0100, 0x01); // stream on
        SNS_DBG("sensor stream on!\n");
    } else {
        result = os08a20_reg_write(nPipeId, 0x0100, 0x00); // stream off
        SNS_DBG("sensor stream off!\n");
    }
    if (0 != result) {
        return -1;
    }
    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return SNS_SUCCESS;
}

static AX_S32 os08a20_sensor_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
{
    AX_S32 width;
    AX_S32 height;
    AX_S32 hdrmode;
    AX_S32 framerate = 30; // init value to 30fps, void null fps gives.
    AX_S32 sns_seting_index = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_PTR_VALID(sns_mode);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (NULL == sns_obj) {
        /* contex init */
        nRet = sensor_ctx_init(nPipeId);
        if (0 != nRet) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return -1;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    width = sns_mode->nWidth;
    height = sns_mode->nHeight;
    framerate = sns_mode->nFrameRate;
    if ((1 != sns_mode->eSnsMode) &&
        (2 != sns_mode->eSnsMode) &&
        (3 != sns_mode->eSnsMode)) {
        hdrmode = 1;
    } else {
        hdrmode = sns_mode->eSnsMode;
    }

    if (width == 3840 && height == 2160 && hdrmode == 1 && framerate == 30) {
        sns_seting_index = e_os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_30fps;
    } else if (width == 3840 && height == 2160 && hdrmode == 1 && framerate == 25) {
        sns_seting_index = e_os08a20_4lane_1280Mbps_24MHz_3840x2160_12bit_rggb_sdr_25fps;
    } else if (width == 3840 && height == 2160 && hdrmode == 1 && framerate == 60) {
        sns_seting_index = e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_sdr_60fps;
    } else if (width == 3840 && height == 2160 && hdrmode == 2 && framerate == 25) {
        sns_seting_index = e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_25fps;
    } else if (width == 3840 && height == 2160 && hdrmode == 2 && framerate == 30) {
        sns_seting_index = e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr2x_30fps;
    } else if (width == 3840 && height == 2160 && hdrmode == 1 && framerate == 20) {
        sns_seting_index = e_os08a20_4lane_1440Mbps_24MHz_3840x2160_12bit_rggb_sdr_20fps;
    } else if (width == 3840 && height == 2160 && hdrmode == 2 && framerate == 20) {
        sns_seting_index = e_os08a20_4lane_1440Mbps_24MHz_3840x2160_10bit_rggb_hdr_20fps;
    } else {
        SNS_ERR("%s it's not supported. [%dx%d mode=%d] \n",
                __func__, width, height, hdrmode);
        return -1;
    }

    SNS_DBG("sns_seting_index %d!\n", sns_seting_index);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = framerate;
    sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel = sns_mode->eWithinBeyondfscSel;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return SNS_SUCCESS;
}

static AX_S32 os08a20_sensor_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
{
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_PTR_VALID(pSnsMode);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (NULL == sns_obj) {
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

static AX_S32 os08a20_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("test pattern enable: %d\n", on);
    if (1 == on) {
        /* enable test-pattern */
        os08a20_reg_write(nPipeId, 0x5081, 0x80);
    } else {
        /* disable test-pattern */
        os08a20_reg_write(nPipeId, 0x5081, 0x00);
    }

    return SNS_SUCCESS;
}

static AX_S32 os08a20_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
{
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);


    params->nSnsMode_caps = AX_SNS_LINEAR_MODE | AX_SNS_HDR_2X_MODE;
    params->nSnsRawType_caps = AX_SNS_RAWTYPE_12BIT | AX_SNS_RAWTYPE_10BIT;
    params->nSnsFps_caps = AX_SNS_25FPS | AX_SNS_30FPS;
    params->nSnsResolution_caps = AX_SNS_RES_3840_2160;
    return SNS_SUCCESS;
}

/****************************************************************************
 * get module default parameters function
 ****************************************************************************/
static AX_S32 os08a20_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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

    SNS_DBG(" current hdr mode %d \n", nHdrmode);

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

static AX_S32 os08a20_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level of linear mode */
    if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        ptBlackLevel->nBlackLevel[0] = 256;  /*08a sdr 12bit 64 => u8.6 256*/
        ptBlackLevel->nBlackLevel[1] = 256;
        ptBlackLevel->nBlackLevel[2] = 256;
        ptBlackLevel->nBlackLevel[3] = 256;
    } else {
        ptBlackLevel->nBlackLevel[0] = 1024;  /*08a hdr 10bit 64 => u8.6 1024*/
        ptBlackLevel->nBlackLevel[1] = 1024;
        ptBlackLevel->nBlackLevel[2] = 1024;
        ptBlackLevel->nBlackLevel[3] = 1024;
    }

    return SNS_SUCCESS;
}

/****************************************************************************
 * exposure control function
 ****************************************************************************/
static AX_S32 os08a20_set_aecparam(ISP_PIPE_ID nPipeId)
{
    AX_F32 vs_line_period = 0;

    SNS_STATE_OBJ *sns_obj = NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    sns_os08a20params[nPipeId].hts = os08a20_get_hts(nPipeId);
    sns_os08a20params[nPipeId].vs_hts = os08a20_get_vs_hts(nPipeId);
    sns_os08a20params[nPipeId].vts = os08a20_get_vts(nPipeId);
    /*TODO LCL sclk*/
    sns_os08a20params[nPipeId].sclk = os08a20_get_sclk(nPipeId);

    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        //sns_os08a20params[nPipeId].sclk = 162000000;
        sns_os08a20params[nPipeId].line_period = (float)sns_os08a20params[nPipeId].hts / sns_os08a20params[nPipeId].sclk
                * SNS_1_SECOND_UNIT_US;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        //sns_os08a20params[nPipeId].sclk = 144000000;
        sns_os08a20params[nPipeId].line_period = (float)(2 * sns_os08a20params[nPipeId].hts) / sns_os08a20params[nPipeId].sclk
                * SNS_1_SECOND_UNIT_US;
    } else {
        // wrong hdr mode
    }

    sns_os08a20params[nPipeId].line_period_fixednum = ax_float_convert_to_int(sns_os08a20params[nPipeId].line_period, 22, 10, 0);

    SNS_DBG("sns_os08a20params[nPipeId].line_period = %.3f line_period(U22.10) = %u \n",
                sns_os08a20params[nPipeId].line_period, sns_os08a20params[nPipeId].line_period_fixednum);

    /*TODO LCL nGainMode*/
    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_SEPARATION;
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = 4.05f;

    /* sensor again  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = 15.5;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 16;

    /* sensor dgain  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = 15.99;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor medium again limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX] = 15.5;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_MEDIUM_FRAME_IDX] = (AX_F32)1 / 16;

    /* sensor medium dgain limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX] = 15.99;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_MEDIUM_FRAME_IDX] = (AX_F32)1 / 1024;
    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxgain = (AX_F32)15.5 * 15.99;
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1.0f;

    sns_obj->sns_param.sns_ae_limit.nMinratio = OS08A20_MIN_RATIO;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = OS08A20_MAX_RATIO;

    if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 8 * sns_os08a20params[nPipeId].line_period_fixednum;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (float)(sns_os08a20params[nPipeId].vts - 8) *
                sns_os08a20params[nPipeId].line_period_fixednum;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = sns_os08a20params[nPipeId].line_period_fixednum;
    } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 4 * sns_os08a20params[nPipeId].line_period_fixednum;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (float)(
                    sns_os08a20params[nPipeId].vts - 8) * sns_os08a20params[nPipeId].line_period_fixednum ;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = sns_os08a20params[nPipeId].line_period_fixednum;

        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 4 * sns_os08a20params[nPipeId].line_period_fixednum;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (float)(
                    sns_os08a20params[nPipeId].vts - 8) * sns_os08a20params[nPipeId].line_period_fixednum;
    if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = AXSNS_MIN((SENSOR_MAX_LINE_GAP - os08a20_get_l2s_offset(nPipeId)) * sns_os08a20params[nPipeId].line_period_fixednum,
                                                                                    sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 45 - sns_obj->sns_mode_obj.nHeight));
    }
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_MEDIUM_FRAME_IDX] = sns_os08a20params[nPipeId].line_period_fixednum;
    } else {
        // wrong hdr mode
    }

    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 2.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] =
                                        ax_float_convert_to_int(0.001f * SNS_1_SECOND_UNIT_US, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] =
                                        ax_float_convert_to_int(0.001f * SNS_1_SECOND_UNIT_US, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return SNS_SUCCESS;
}

static AX_S32 os08a20_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 result = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result = os08a20_get_gain_table(params);
    return result;

    return SNS_SUCCESS;
}


static AX_S32 os08a20_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 Gain_in;
    AX_U8 Gain_de;
    AX_S32 result = 0;
    AX_F32 gain_value = 0;
    AX_F32 nGainFromUser = 0;

    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptAnalogGainTbl);

    /* long gain seting */
    nGainFromUser = ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX];
    nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] - nGainFromUser) < EPS) {
        SNS_DBG("SENSOR LONG new and cur again equal nGainFromUser:%f\n", nGainFromUser);
        /* nothing to do */
    } else {
        gain_value = os08a20_again2value(nGainFromUser, &Gain_in, &Gain_de);
        if (gain_value == -1) {
            SNS_ERR("SENSOR LONG nGainFromUser:%f match fail\n", nGainFromUser);
        } else {
            sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = gain_value;
            result = os08a20_sns_update_regs_table(nPipeId, OS08A20_LONG_AGAIN_H, (Gain_in & 0x3F));
            result |= os08a20_sns_update_regs_table(nPipeId, OS08A20_LONG_AGAIN_L, (Gain_de & 0xFF));
            if (result != 0) {
                SNS_ERR("SENSOR LONG nGainFromUser:%f update fail\n", nGainFromUser);
                return result;
            }
            SNS_DBG("SENSOR LONG nGainFromUser:%f, %f-0x%x-0x%x, 0x%x-0x%x\n",
                nGainFromUser, gain_value, Gain_in, Gain_de, (Gain_in & 0x3F), (Gain_de & 0xFF));
        }
    }

    /* medium gain seting */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nGainFromUser = ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
        nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] - nGainFromUser) < EPS) {
            SNS_DBG("SENSOR SHORT new and cur again equal nGainFromUser:%f\n", nGainFromUser);
            /* nothing to do */
        } else {
            gain_value = os08a20_again2value(nGainFromUser, &Gain_in, &Gain_de);
            if (gain_value == -1) {
                SNS_ERR("SENSOR SHORT nGainFromUser:%f match fail\n", nGainFromUser);
            } else {
                sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = gain_value;
                result = os08a20_sns_update_regs_table(nPipeId, OS08A20_SHORT_AGAIN_H, (Gain_in & 0x3F));
                result = os08a20_sns_update_regs_table(nPipeId, OS08A20_SHORT_AGAIN_L, (Gain_de & 0xFF));
                if (result != 0) {
                    SNS_ERR("SENSOR SHORT nGainFromUser:%f update fail\n", nGainFromUser);
                    return result;
                }
                SNS_DBG("SENSOR SHORT nGainFromUser:%f, %f-0x%x-0x%x, 0x%x-0x%x\n",
                nGainFromUser, gain_value, Gain_in, Gain_de, (Gain_in & 0x3F), (Gain_de & 0xFF));
            }
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 os08a20_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    AX_U8 Gain_in;
    AX_U8 Gain_de;
    AX_U8 Gain_de2;
    AX_S32 result = 0;
    AX_F32 gain_val = 0;
    AX_F32 nGainFromUser = 0;

    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptDigitalGainTbl);

    /* long frame digital gain seting */
    nGainFromUser = ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX];
    nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX]);
    if (fabs(sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] - nGainFromUser) < EPS) {
        //SNS_DBG("new and current gain is equal \n");
        /* nothing to do */
    } else {
        gain_val = os08a20_dgain2value(nGainFromUser, &Gain_in, &Gain_de, &Gain_de2);
        if (gain_val == -1) {
            SNS_ERR("new gain match failed \n");
        } else {
            sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = gain_val;
            result = os08a20_sns_update_regs_table(nPipeId, OS08A20_LONG_DGAIN_H, (Gain_in & 0x3F));
            result = os08a20_sns_update_regs_table(nPipeId, OS08A20_LONG_DGAIN_L, (Gain_de & 0xFF));
            if (result != 0) {
                SNS_ERR("write hw failed %d \n", result);
                return result;
            }
        }
    }

    /* medium frame digital gain seting */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        nGainFromUser = ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
        nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] - nGainFromUser) < EPS) {
            SNS_DBG("new and current gain is equal \n");
            /* nothing to do */
        } else {
            gain_val = os08a20_dgain2value(nGainFromUser, &Gain_in, &Gain_de, &Gain_de2);
            if (gain_val == -1) {
                SNS_ERR("new gain match failed \n");
            } else {
                sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = gain_val;
                result = os08a20_sns_update_regs_table(nPipeId, OS08A20_SHORT_DGAIN_H, (Gain_in & 0x3F));
                result = os08a20_sns_update_regs_table(nPipeId, OS08A20_SHORT_DGAIN_L, (Gain_de & 0xFF));
                if (result != 0) {
                    SNS_ERR("write hw failed %d \n", result);
                    return result;
                }
            }
        }
    }

    return SNS_SUCCESS;
}


static AX_S32 os08a20_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U8 ex_h;
    AX_U8 ex_l;
    AX_U32 ex_ival = 0;
    AX_S32 result = 0;
    float ratio = 0.0f;
    AX_U32 nIntTimeFromUser = 0;

    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptIntTimeTbl);

    SNS_DBG("Exptime:%d-%d-%d-%d, Hdrratio:%f-%f-%f-%f\n",
            ptIntTimeTbl->nIntTime[0], ptIntTimeTbl->nIntTime[1], ptIntTimeTbl->nIntTime[2], ptIntTimeTbl->nIntTime[3],
            ptIntTimeTbl->nHdrRatio[0], ptIntTimeTbl->nHdrRatio[1], ptIntTimeTbl->nHdrRatio[2], ptIntTimeTbl->nHdrRatio[3]);

    if (fabs(ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX]) <= EPS) {
        SNS_ERR("hdr ratio is error \n");
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    /* This sensor uses a ratio by default */
    ratio = ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX];
    ratio = AXSNS_CLIP3(ratio, sns_obj->sns_param.sns_ae_limit.nMinratio, sns_obj->sns_param.sns_ae_limit.nMaxratio);

    /* if 2DOL/3DOL, need re-limit s  frame MAX integration time dynamicly according current hdr ratio */
    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = ((sns_os08a20params[nPipeId].vts - 4) *
                sns_os08a20params[nPipeId].line_period_fixednum) * ((ratio) / (1 + ratio));

    } else {
        // do nothing
    }

    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                   sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                   sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

    if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] == nIntTimeFromUser) {
        //SNS_DBG("new and current integration time  is equal \n");
    } else {
        if (sns_os08a20params[nPipeId].line_period_fixednum == 0) {
            SNS_ERR("08a20Ctx->1h_period =0U (Division by zero !!!)\n");
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }

        ex_ival = nIntTimeFromUser / sns_os08a20params[nPipeId].line_period_fixednum;
        ex_l = REG_LOW_8BITS(ex_ival);
        ex_h = REG_HIGH_8BITS(ex_ival);
        SNS_DBG("Exptime long frame: time: %u us,  %d line\n", nIntTimeFromUser, ex_ival);

        os08a20_sns_update_regs_table(nPipeId, OS08A20_LONG_EXP_LINE_H, ex_h);
        os08a20_sns_update_regs_table(nPipeId, OS08A20_LONG_EXP_LINE_L, ex_l);

        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
        /* if 2DOL/3DOL, ned re-limit s frame and or vs frame MAX integration time dynamicly according current hdr ratio */
        if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
                sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = AXSNS_MIN3(sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 45 - sns_obj->sns_mode_obj.nHeight),
                                                                                            (SENSOR_MAX_LINE_GAP - os08a20_get_l2s_offset(nPipeId)) * sns_os08a20params[nPipeId].line_period_fixednum,
                                                                                            sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 2) - sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);
            } else {
                sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 2) -
                                                                                            sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX];
            }
        } else {
            // do nothing
        }
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_MEDIUM_FRAME_IDX];
        nIntTimeFromUser = nIntTimeFromUser / ratio;
        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]);

        if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] == nIntTimeFromUser) {
            SNS_DBG("new and current integration time  is equal \n");
        } else {
            if (sns_obj->sns_mode_obj.nFrameRate == 0U) {
                SNS_ERR("pos08a20Ctx->frame_rate =0U (Division by zero !!!)\n");
                return SNS_ERR_CODE_ILLEGAL_PARAMS;
            }

            if (sns_os08a20params[nPipeId].line_period_fixednum == 0) {
                SNS_ERR("pos08a20Ctx->1h_period =0U (Division by zero !!!)\n");
                return SNS_ERR_CODE_ILLEGAL_PARAMS;
            }
            ex_ival = nIntTimeFromUser / sns_os08a20params[nPipeId].line_period_fixednum;

            ex_l = REG_LOW_8BITS(ex_ival);
            ex_h = REG_HIGH_8BITS(ex_ival);

            SNS_DBG("Exptime medium frame: time: %u us,  %d line\n", nIntTimeFromUser, ex_ival);

            os08a20_sns_update_regs_table(nPipeId, OS08A20_SHORT_EXP_LINE_H, ex_h);
            os08a20_sns_update_regs_table(nPipeId, OS08A20_SHORT_EXP_LINE_L, ex_l);

            /* 2DOL line gap */
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] =
                AXSNS_MAX(ex_ival + os08a20_get_l2s_offset(nPipeId), 25);
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGapTime[HDR_LONG_FRAME_IDX] =
                sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] * sns_os08a20params[nPipeId].line_period + 1;

            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = nIntTimeFromUser;
        }
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX] =
            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX];
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_MEDIUM_FRAME_IDX] =
            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX];
    }

    return SNS_SUCCESS;
}

static AX_S32 os08a20_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    AX_F32 gain_value = 0;
    float gain = 0.0f;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);


    return SNS_SUCCESS;
}


static AX_S32 os08a20_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 os08a20_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}

static AX_S32 os08a20_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
{
    AX_S32 result = 0;
    AX_S32 framerate = 30;
    AX_U32 vts = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    framerate =  sns_obj->sns_mode_obj.nFrameRate;
    vts = sns_obj->sns_mode_obj.nVts;
    if (nFps >= framerate) {
        sns_os08a20params[nPipeId].vts = vts;
    } else {
        sns_os08a20params[nPipeId].vts = 1 * SNS_1_SECOND_UNIT_US / (sns_os08a20params[nPipeId].line_period * nFps);
    }

    if (sns_os08a20params[nPipeId].vts > OS08A20_MAX_VTS){
        sns_os08a20params[nPipeId].vts = OS08A20_MAX_VTS;
        nFps = 1 * SNS_1_SECOND_UNIT_US / (sns_os08a20params[nPipeId].line_period *sns_os08a20params[nPipeId].vts );
        SNS_ERR("Beyond minmum fps  %f\n",nFps);
    }
    result = os08a20_set_vts(nPipeId, sns_os08a20params[nPipeId].vts);

    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] =
        (sns_os08a20params[nPipeId].vts - 8) * sns_os08a20params[nPipeId].line_period_fixednum;
    if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = AXSNS_MIN3(sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 45 - sns_obj->sns_mode_obj.nHeight),
                                                                                    (SENSOR_MAX_LINE_GAP - os08a20_get_l2s_offset(nPipeId)) * sns_os08a20params[nPipeId].line_period_fixednum,
                                                                                    sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 2) - sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);
    } else {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = sns_os08a20params[nPipeId].line_period_fixednum * (sns_os08a20params[nPipeId].vts - 2) -
                                                                                    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX];
    }
    sns_obj->sns_param.sns_ae_param.nCurFps = 1 * SNS_1_SECOND_UNIT_US / (sns_os08a20params[nPipeId].line_period *sns_os08a20params[nPipeId].vts );


    SNS_DBG("nPipeId = %d, fps(from alg) = %f, current vts = 0x%x\n", nPipeId, nFps, sns_os08a20params[nPipeId].vts);

    return SNS_SUCCESS;
}

static AX_S32 os08a20_ae_get_slow_shutter_param(ISP_PIPE_ID nPipeId, AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
{
    AX_S32 framerate = 30;
    AX_U32 nfps = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_U32 nVts = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    framerate = sns_obj->sns_mode_obj.nFrameRate;
    if (SNS_MAX_FRAME_RATE < framerate) {
        SNS_ERR("framerate out of range : %d\n", framerate);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    if(ax_sns_is_zero(sns_os08a20params[nPipeId].line_period)) {
        SNS_ERR("line_period is zero : %.2f\n", sns_os08a20params[nPipeId].line_period);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ptSlowShutterParam->nGroupNum = AXSNS_MIN((sizeof(gFpsGear) / sizeof(AX_F32)), framerate);
    ax_sns_quick_sort_float(gFpsGear, ptSlowShutterParam->nGroupNum);
    ptSlowShutterParam->fMinFps =AXSNS_MAX(gFpsGear[0], (1 * SNS_1_SECOND_UNIT_US / (sns_os08a20params[nPipeId].line_period * OS08A20_MAX_VTS)));

    for (nfps = 0 ; nfps < ptSlowShutterParam->nGroupNum; nfps++) {
        nVts = 1 * SNS_1_SECOND_UNIT_US / (sns_os08a20params[nPipeId].line_period * gFpsGear[nfps]);
        if((AX_S32)gFpsGear[nfps] >= framerate) {
            nVts = sns_obj->sns_mode_obj.nVts;
        }
        if (nVts > OS08A20_MAX_VTS) {
            nVts = OS08A20_MAX_VTS;
            SNS_WRN("Beyond minmum fps  %f\n", ptSlowShutterParam->fMinFps);
        }
        if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime = (AX_U32)((nVts  - 8) * sns_os08a20params[nPipeId].line_period_fixednum);
        } else if(IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime = (AX_U32)((nVts  - 4) * sns_os08a20params[nPipeId].line_period_fixednum
            * OS08A20_MAX_RATIO / (OS08A20_MAX_RATIO + 1));
        }

        ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps = 1 * SNS_1_SECOND_UNIT_US / (sns_os08a20params[nPipeId].line_period * nVts);
        ptSlowShutterParam->fMaxFps  =  ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps;

        SNS_DBG("nPipeId = %d, line_period = %.2f, line_period_fixednum:%u ,fps = %.2f, nMaxIntTime = %d, vts=0x%x\n",
                nPipeId, sns_os08a20params[nPipeId].line_period,sns_os08a20params[nPipeId].line_period_fixednum,
                ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps,
                ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime, nVts);
    }

    return SNS_SUCCESS;
}


AX_SENSOR_REGISTER_FUNC_T gSnsos08a20Obj = {

    /* sensor ctrl */
    .pfn_sensor_reset                   = os08a20_sensor_reset,
    .pfn_sensor_chipid                  = os08a20_get_chipid,
    .pfn_sensor_init                    = os08a20_init,
    .pfn_sensor_exit                    = os08a20_exit,
    .pfn_sensor_streaming_ctrl          = os08a20_sensor_streaming_ctrl,
    .pfn_sensor_testpattern             = os08a20_testpattern_ctrl,

    .pfn_sensor_set_mode                = os08a20_sensor_set_mode,
    .pfn_sensor_get_mode                = os08a20_sensor_get_mode,
    .pfn_sensor_set_wdr_mode            = NULL,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = os08a20_set_bus_info,
    .pfn_sensor_write_register          = os08a20_reg_write,
    .pfn_sensor_read_register           = os08a20_reg_read,

    /* default param */
    .pfn_sensor_get_default_params      = os08a20_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = os08a20_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = os08a20_set_sensor_params,
    .pfn_sensor_get_params              = os08a20_get_sensor_params,
    .pfn_sensor_get_gain_table          = os08a20_get_sensor_gain_table,
    .pfn_sensor_set_again               = os08a20_set_again,
    .pfn_sensor_set_dgain               = os08a20_set_dgain,
    .pfn_sensor_set_integration_time    = os08a20_set_integration_time,
    .pfn_sensor_hcglcg_ctrl             = os08a20_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = os08a20_set_fps,
    .pfn_sensor_get_slow_shutter_param  = os08a20_ae_get_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = os08a20_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = NULL,
};
