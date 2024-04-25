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
#include "os04a10_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "os04a10_sdr.h"
#include "os04a10_hdr_2x.h"

/****************************************************************************
 * golbal variables  and macro definition                                   *
 ****************************************************************************/
SNSOS04A10_OBJ sns_os04a10params[DEF_VIN_PIPE_MAX_NUM];
AX_SNS_COMMBUS_T g_Os04A10BusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0}};

SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define OS04A10_MAX_VTS         (0xFFFF)
#define OS04A10_MAX_RATIO       (16.0f)
#define OS04A10_MIN_RATIO       (1.0f)

#define OS04A10_EXP_OFFSET_SDR           (0.3f) //unit:line
#define OS04A10_EXP_OFFSET_HDR_2STAGGER  (0.6f)
#define OS04A10_EXP_OFFSET_HDR_3STAGGER  (0.75f)

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = AX_NULL)

/*user config*/
static AX_F32 gFpsGear[] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00, 11.00, 12.00, 13.00, 14.00, 15.00,
                            16.00, 17.00, 18.00, 19.00, 20.00, 21.00, 22.00, 23.00, 24.00, 25.00, 26.00, 27.00, 28.00, 29.00, 30.00
                           };

static AX_S32 os04a10_set_aecparam(ISP_PIPE_ID nPipeId);
static AX_S32 os04a10_get_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl);
static AX_S32 os04a10_get_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl);
static AX_S32 os04a10_get_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl);

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
            SNS_ERR("malloc sns_ctx failed\r\n");
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
static AX_S32 os04a10_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    os04a10_reset(nPipeId, g_Os04A10BusInfo[nPipeId].I2cDev);

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U32 sensor_id = 0;
    AX_U16 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= os04a10_reg_read(nPipeId, 0x300A) << 16;
    sensor_id |= os04a10_reg_read(nPipeId, 0x300B) << 8;
    sensor_id |= os04a10_reg_read(nPipeId, 0x300C);

    SNS_DBG("%s: sensor os04a10 id: 0x%x\n", __func__, sensor_id);

    if (sensor_id != OS04A10_SENSOR_CHIP_ID) {
        SNS_ERR("%s: Failed to read sensor os04a10 id\n", __func__);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    *pSnsId = sensor_id;

    return SNS_SUCCESS;
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
    sns_obj->sns_i2c_obj.slave_addr = OS04A10_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = OS04A10_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = OS04A10_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = OS04A10_SWAP_BYTES;

    sns_obj->sns_i2c_obj.sns_i2c_bnum = g_Os04A10BusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

    ret = os04a10_get_chipid(nPipeId, &snsId);
    if (ret !=  SNS_SUCCESS) {
        SNS_ERR("can't find os04a10 sensor id ret=%d.\r\n", ret);
        return ret;
    } else {
        SNS_DBG("Sensor: os04a10 check chip id success.\r\n");
    }

    SNS_DBG("os04a10 i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}

static void os04a10_init(ISP_PIPE_ID nPipeId)
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
    os04a10_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    os04a10_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    os04a10_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;

    sns_obj->sns_mode_obj.nVts = os04a10_get_vts(nPipeId);

    return;
}

static void os04a10_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = NULL;
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

static AX_S32 os04a10_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    g_Os04A10BusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 os04a10_sensor_streaming_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
        result = os04a10_reg_write(nPipeId, 0x0100, 0x01); // stream on
        SNS_DBG("sensor stream on!\n");
    } else {
        result = os04a10_reg_write(nPipeId, 0x0100, 0x00); // stream off
        SNS_DBG("sensor stream off!\n");
    }
    if (0 != result) {
        return -1;
    }
    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return SNS_SUCCESS;
}

static AX_S32 os04a10_sensor_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
{
    AX_S32 width = 0;
    AX_S32 height = 0;
    AX_S32 hdrmode = 0;
    AX_S32 nRawType = 0;
    AX_S32 framerate = 25; // init value to 30fps, void null fps gives.
    AX_S32 sns_seting_index = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(sns_mode);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        nRet = sensor_ctx_init(nPipeId);
        if (0 != nRet) {
            SNS_ERR("sensor_ctx_init failed!\n");
            return SNS_ERR_CODE_INIT_FAILD;
        } else {
            SENSOR_GET_CTX(nPipeId, sns_obj);
        }
    }

    sns_obj->bSyncInit = AX_FALSE;
    width = sns_mode->nWidth;
    height = sns_mode->nHeight;
    framerate = sns_mode->nFrameRate;
    nRawType = sns_mode->eRawType;

    if ((AX_SNS_LINEAR_MODE != sns_mode->eSnsMode) &&
        (AX_SNS_HDR_2X_MODE != sns_mode->eSnsMode) &&
        (AX_SNS_HDR_3X_MODE != sns_mode->eSnsMode)) {
        hdrmode = AX_SNS_LINEAR_MODE;
    } else {
        hdrmode = sns_mode->eSnsMode;
    }

    if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 25 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_Linear_25fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 30 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_Linear_30fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 12 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_Linear_12fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 25 && nRawType == 12) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_12bit_Linear_25fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 30 && nRawType == 12) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_12bit_Linear_30fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && framerate == 30) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_2Stagger_HDR_30fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && framerate == 12) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_2Stagger_HDR_12fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && framerate == 25) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_2Stagger_HDR_25fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_3X_MODE) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_3Stagger_HDR_25fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 60 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_Linear_60fps;
    } else if (width == 2560 && height == 1440 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 60 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2560x1440_10bit_Linear_60fps;
    } else if (width == 2560 && height == 1440 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 50 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2560x1440_10bit_Linear_50fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 10 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_Linear_10fps;
    } else if (width == 2688 && height == 1520 && hdrmode == AX_SNS_HDR_2X_MODE && framerate == 10 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_2688x1520_10bit_2Stagger_HDR_10fps;
    } else if (width == 1920 && height == 1080 && hdrmode == AX_SNS_LINEAR_MODE && framerate == 60 && nRawType == 10) {
        sns_seting_index = e_OS04A10_4lane_1920x1080_10bit_Linear_60fps;
    } else {
        SNS_ERR("it's not supported. [%dx%d mode=%d] \n", width, height, hdrmode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("nPipeId=%d, sns_seting_index %d!\n", nPipeId, sns_seting_index);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = framerate;
    sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel = sns_mode->eWithinBeyondfscSel;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return SNS_SUCCESS;
}

static AX_S32 os04a10_sensor_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
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


static AX_S32 os04a10_sensor_set_wdr_mode(ISP_PIPE_ID nPipeId, AX_SNS_HDR_MODE_E eHdrMode)
{
    return SNS_SUCCESS;
}

static AX_S32 os04a10_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("test pattern enable: %d\n", on);
    if (1 == on) {
        /* enable test-pattern */
        os04a10_reg_write(nPipeId, 0x5080, 0x80); /* long  frame*/
        os04a10_reg_write(nPipeId, 0x50c0, 0x80); /* short frame*/
    } else {
        /* disable test-pattern */
        os04a10_reg_write(nPipeId, 0x5080, 0x00); /* long  frame*/
        os04a10_reg_write(nPipeId, 0x50c0, 0x00); /* short frame*/
    }

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
{
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);


    params->nSnsMode_caps = AX_SNS_LINEAR_MODE | AX_SNS_HDR_2X_MODE | AX_SNS_HDR_3X_MODE;
    params->nSnsRawType_caps = AX_SNS_RAWTYPE_12BIT;
    params->nSnsFps_caps = AX_SNS_25FPS;
    params->nSnsResolution_caps = AX_SNS_RES_2688_1520;
    return SNS_SUCCESS;
}

/****************************************************************************
 * get module default parameters function
 ****************************************************************************/
static AX_S32 os04a10_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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
            return SNS_ERR_CODE_INIT_FAILD;
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

static AX_S32 os04a10_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level of linear mode */
    if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        ptBlackLevel->nBlackLevel[0] = 1024;    /* linear mode 10bit sensor default blc 64(U10.0) --> 1024(U8.6) */
        ptBlackLevel->nBlackLevel[1] = 1024;
        ptBlackLevel->nBlackLevel[2] = 1024;
        ptBlackLevel->nBlackLevel[3] = 1024;
    } else {
        ptBlackLevel->nBlackLevel[0] = 1024;
        ptBlackLevel->nBlackLevel[1] = 1024;
        ptBlackLevel->nBlackLevel[2] = 1024;
        ptBlackLevel->nBlackLevel[3] = 1024;
    }

    return SNS_SUCCESS;
}


/****************************************************************************
 * exposure control function
 ****************************************************************************/
static AX_S32 os04a10_set_aecparam(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    AX_SNS_AE_SHUTTER_CFG_T tIntTimeTbl = {0};
    AX_SNS_AE_GAIN_CFG_T tAgainTbl = {0};
    AX_SNS_AE_GAIN_CFG_T tDgainTbl = {0};

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    sns_os04a10params[nPipeId].hts          = os04a10_get_hts(nPipeId);
    sns_os04a10params[nPipeId].vs_hts       = os04a10_get_vs_hts(nPipeId);
    sns_os04a10params[nPipeId].vts          = os04a10_get_vts(nPipeId);
    sns_os04a10params[nPipeId].sclk         = os04a10_get_sclk(nPipeId);
    sns_os04a10params[nPipeId].dcg_status   = os04a10_get_dcg_status(nPipeId);

    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
        sns_os04a10params[nPipeId].line_period = ((float)sns_os04a10params[nPipeId].hts / sns_os04a10params[nPipeId].sclk) *
                SNS_1_SECOND_UNIT_US;
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_os04a10params[nPipeId].line_period = ((float)(2 * sns_os04a10params[nPipeId].hts) / sns_os04a10params[nPipeId].sclk)
                * SNS_1_SECOND_UNIT_US;
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_3X_MODE) {
        sns_os04a10params[nPipeId].line_period = ((float)(3 * sns_os04a10params[nPipeId].hts) / sns_os04a10params[nPipeId].sclk)
                * SNS_1_SECOND_UNIT_US;
    } else {
        /* default config sdr mode params */
        sns_os04a10params[nPipeId].line_period = ((float)sns_os04a10params[nPipeId].hts / sns_os04a10params[nPipeId].sclk) *
                SNS_1_SECOND_UNIT_US;
    }

    sns_os04a10params[nPipeId].line_period_fixednum = ax_float_convert_to_int(sns_os04a10params[nPipeId].line_period, 22, 10, 0);

    SNS_DBG("sns_os04a10params[nPipeId].line_period = %.3f line_period(U22.10) = %u \n",
                sns_os04a10params[nPipeId].line_period, sns_os04a10params[nPipeId].line_period_fixednum);


    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_SEPARATION;
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = 3.5f;

    /* sensor again  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = OS04A10_MAX_AGAIN;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 16;

    /* sensor dgain  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = OS04A10_MAX_DGAIN;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor medium again limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX] = OS04A10_MAX_AGAIN;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_MEDIUM_FRAME_IDX] = (AX_F32)1 / 16;

    /* sensor medium dgain limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX] = OS04A10_MAX_DGAIN;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_MEDIUM_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor sort again limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_SHORT_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_SHORT_FRAME_IDX] = OS04A10_MAX_AGAIN;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_SHORT_FRAME_IDX] = (AX_F32)1 / 16;

    /* sensor sort dgain limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_SHORT_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_SHORT_FRAME_IDX] = OS04A10_MAX_DGAIN;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_SHORT_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxgain = (AX_F32)OS04A10_MAX_AGAIN * OS04A10_MAX_DGAIN;
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1/256; //U10.8

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 2 * sns_os04a10params[nPipeId].line_period_fixednum;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (float)(sns_os04a10params[nPipeId].vts - 8) *
                                                                                sns_os04a10params[nPipeId].line_period_fixednum;

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 2 * sns_os04a10params[nPipeId].line_period_fixednum;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (float)(sns_os04a10params[nPipeId].vts - 8) *
                                                                                  sns_os04a10params[nPipeId].line_period_fixednum;

    if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = AXSNS_MIN((SENSOR_MAX_LINE_GAP - os04a10_get_l2s_offset(nPipeId)) * sns_os04a10params[nPipeId].line_period_fixednum,
                                                                                    sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 32 - sns_obj->sns_mode_obj.nHeight));
    }

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_SHORT_FRAME_IDX] = 2 * sns_os04a10params[nPipeId].line_period_fixednum;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_SHORT_FRAME_IDX] = (float)(sns_os04a10params[nPipeId].vts - 8) *
                                                                                sns_os04a10params[nPipeId].line_period_fixednum;

    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX]   = sns_os04a10params[nPipeId].line_period_fixednum;
    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_MEDIUM_FRAME_IDX] = sns_os04a10params[nPipeId].line_period_fixednum;
    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_SHORT_FRAME_IDX]  = sns_os04a10params[nPipeId].line_period_fixednum;

    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_LONG_FRAME_IDX] = OS04A10_EXP_OFFSET_SDR;
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_LONG_FRAME_IDX] = OS04A10_EXP_OFFSET_HDR_2STAGGER;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_MEDIUM_FRAME_IDX] = OS04A10_EXP_OFFSET_HDR_2STAGGER;
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_3X_MODE) {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_LONG_FRAME_IDX] = OS04A10_EXP_OFFSET_HDR_3STAGGER;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_MEDIUM_FRAME_IDX] = OS04A10_EXP_OFFSET_HDR_3STAGGER;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_SHORT_FRAME_IDX] = OS04A10_EXP_OFFSET_HDR_3STAGGER;
    } else {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_LONG_FRAME_IDX] = OS04A10_EXP_OFFSET_SDR;
    }

    sns_obj->sns_param.sns_ae_limit.nMinratio = OS04A10_MIN_RATIO;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = OS04A10_MAX_RATIO;

    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    os04a10_get_integration_time(nPipeId, &tIntTimeTbl);
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = tIntTimeTbl.nIntTime[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = tIntTimeTbl.nIntTime[HDR_MEDIUM_FRAME_IDX];

    os04a10_get_again(nPipeId, &tAgainTbl);
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = tAgainTbl.nGain[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = tAgainTbl.nGain[HDR_MEDIUM_FRAME_IDX];

    os04a10_get_dgain(nPipeId, &tDgainTbl);
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = tDgainTbl.nGain[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = tDgainTbl.nGain[HDR_MEDIUM_FRAME_IDX];

    SNS_DBG("limit_again:%f-%f-%f-%f, limit_dgain:%f-%f-%f-%f, limit_exp:%.2f-%.2f-%.2f-%.2f\n",
        sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX],
        sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX],
        (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024);

    SNS_DBG("cur_again:%f-%f, cur_dgain:%f-%f, cur_exp:%.2f-%.2f, cur_fps:%.2f, cur_dcg:%d\n",
        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX],
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX],
        (float)sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024,
        sns_obj->sns_param.sns_ae_param.nCurFps,
        sns_os04a10params[nPipeId].dcg_status);

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 result = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result = os04a10_get_gain_table(params);
    return result;

    return SNS_SUCCESS;
}


static AX_S32 os04a10_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 Gain_in;
    AX_U8 Gain_de;
    AX_S32 result = 0;
    AX_F32 gain_value = 0;
    AX_F32 nGainFromUser = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptAnalogGainTbl);

    /* long gain seting */
    nGainFromUser = ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX];
    nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] - nGainFromUser) < EPS) {
        /* nothing to do */
    } else {
        gain_value = os04a10_again2value(nGainFromUser, &Gain_in, &Gain_de);
        if (gain_value == -1) {
            SNS_ERR("new gain match failed \n");
        } else {
            sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = gain_value;
            result = os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_AGAIN_H, (Gain_in & 0x1F));
            result |= os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_AGAIN_L, (Gain_de & 0xF0));
            if (result != 0) {
                SNS_ERR("write hw failed %d \n", result);
                return result;
            }
        }
    }

    /* medium gain seting */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nGainFromUser = ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
        nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] - nGainFromUser) < EPS) {
            SNS_DBG("new and current gain is equal \n");
            /* nothing to do */
        } else {
            gain_value = os04a10_again2value(nGainFromUser, &Gain_in, &Gain_de);
            if (gain_value == -1) {
                SNS_ERR(" new gain match failed \n");
            } else {
                sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = gain_value;
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_AGAIN_H, (Gain_in & 0x1F));
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_AGAIN_L, (Gain_de & 0xF0));
                if (result != 0) {
                    SNS_ERR("%s: write hw failed %d \n", __func__, result);
                    return result;
                }
            }
        }
    }

    /* sort gain seting */
    if (IS_3DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nGainFromUser = ptAnalogGainTbl->nGain[HDR_SHORT_FRAME_IDX];
        nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_SHORT_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_SHORT_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_SHORT_FRAME_IDX] - nGainFromUser) < EPS) {
            SNS_DBG(" new and current gain is equal \n");
            /* nothing to do */
        } else {
            gain_value = os04a10_again2value(nGainFromUser, &Gain_in, &Gain_de);
            if (gain_value == -1) {
                SNS_ERR("new gain match failed \n");

            } else {
                sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_SHORT_FRAME_IDX] = gain_value;
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_AGAIN_H, (Gain_in & 0x1F));
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_AGAIN_L, (Gain_de & 0xF0));
                if (result != 0) {
                    SNS_ERR("write hw failed %d \n", result);
                    return result;
                }
            }
        }

    }

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 gain_in = 0;
    AX_U8 gain_de = 0;
    AX_F32 gain_value = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptAnalogGainTbl);

    gain_in = os04a10_reg_read(nPipeId, OS04A10_LONG_AGAIN_H);
    gain_de = os04a10_reg_read(nPipeId, OS04A10_LONG_AGAIN_L);

    gain_value = os04a10_value2again(gain_in, gain_de);
    if (gain_value < 0) {
        SNS_ERR("get lef again failed, 0x%x-0x%x\n", gain_in, gain_de);
    } else {
        ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX] = gain_value;
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        gain_in = os04a10_reg_read(nPipeId, OS04A10_SHORT_AGAIN_H);
        gain_de = os04a10_reg_read(nPipeId, OS04A10_SHORT_AGAIN_L);

        gain_value = os04a10_value2again(gain_in, gain_de);
        if (gain_value < 0) {
            SNS_ERR("get sef1 again failed, 0x%x-0x%x\n", gain_in, gain_de);
        } else {
            ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX] = gain_value;
            ptAnalogGainTbl->nHdrRatio[HDR_LONG_FRAME_IDX] =
                ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX]/ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
        }
    }

    SNS_DBG("get current again:%f-%f, ratio[0]:%f\n",
        ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX],
        ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX],
        ptAnalogGainTbl->nHdrRatio[HDR_LONG_FRAME_IDX]);

    return SNS_SUCCESS;
}

static AX_S32 os04a10_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    AX_U8 Gain_in;
    AX_U8 Gain_de;
    AX_U8 Gain_de2;
    AX_S32 result = 0;
    AX_F32 gain_val = 0;
    AX_F32 nGainFromUser = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

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
        gain_val = os04a10_dgain2value(nGainFromUser, &Gain_in, &Gain_de, &Gain_de2);
        if (gain_val == -1) {
            SNS_ERR("new gain match failed \n");
        } else {
            sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = gain_val;
            result = os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_DGAIN_H, (Gain_in & 0xF));
            result = os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_DGAIN_M, (Gain_de & 0xFF));
            result = os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_DGAIN_L, (Gain_de2 & 0xC0));
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
            gain_val = os04a10_dgain2value(nGainFromUser, &Gain_in, &Gain_de, &Gain_de2);
            if (gain_val == -1) {
                SNS_ERR("new gain match failed \n");
            } else {
                sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = gain_val;
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_DGAIN_H, (Gain_in & 0xF));
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_DGAIN_M, (Gain_de & 0xFF));
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_DGAIN_L, (Gain_de2 & 0xC0));
                if (result != 0) {
                    SNS_ERR("write hw failed %d \n", result);
                    return result;
                }
            }
        }
    }

    /* short frame digital gain seting */
    if (IS_3DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        nGainFromUser = ptDigitalGainTbl->nGain[HDR_SHORT_FRAME_IDX];
        nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_SHORT_FRAME_IDX],
                                    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_SHORT_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_SHORT_FRAME_IDX] - nGainFromUser) < EPS) {
            SNS_DBG("new and current gain is equal \n");
            /* nothing to do */
        } else {
            gain_val = os04a10_dgain2value(nGainFromUser, &Gain_in, &Gain_de, &Gain_de2);
            if (gain_val == -1) {
                SNS_ERR("new gain match failed \n");
            } else {
                sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_SHORT_FRAME_IDX] = gain_val;
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_DGAIN_H, (Gain_in & 0xF));
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_DGAIN_M, (Gain_de & 0xFF));
                result = os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_DGAIN_L, (Gain_de2 & 0xC0));

                if (result != 0) {
                    SNS_ERR("write hw failed %d \n", result);
                    return result;
                }
            }
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    AX_U8 gain_in = 0;
    AX_U8 gain_de = 0;
    AX_U8 gain_de2 = 0;
    AX_F32 gain_value = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptDigitalGainTbl);

    gain_in = os04a10_reg_read(nPipeId, OS04A10_LONG_DGAIN_H);
    gain_de = os04a10_reg_read(nPipeId, OS04A10_LONG_DGAIN_M);
    gain_de2 = os04a10_reg_read(nPipeId, OS04A10_LONG_DGAIN_L);

    gain_value = os04a10_value2dgain(gain_in, gain_de, gain_de2);
    if (gain_value < 0) {
        SNS_ERR("get lef dgain failed, 0x%x-0x%x-0x%x\n", gain_in, gain_de, gain_de2);
    } else {
        ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX] = gain_value;
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        gain_in = os04a10_reg_read(nPipeId, OS04A10_SHORT_DGAIN_H);
        gain_de = os04a10_reg_read(nPipeId, OS04A10_SHORT_DGAIN_M);
        gain_de2 = os04a10_reg_read(nPipeId, OS04A10_SHORT_DGAIN_L);

        gain_value = os04a10_value2dgain(gain_in, gain_de, gain_de2);
        if (gain_value < 0) {
            SNS_ERR("get sef1 dgain failed, 0x%x-0x%x-0x%x\n", gain_in, gain_de, gain_de2);
        } else {
            ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX] = gain_value;
            ptDigitalGainTbl->nHdrRatio[HDR_LONG_FRAME_IDX] =
                ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX]/ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
        }
    }

    SNS_DBG("get current dgain:%f-%f, ratio[0]:%f\n",
        ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX],
        ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX],
        ptDigitalGainTbl->nHdrRatio[HDR_LONG_FRAME_IDX]);

    return SNS_SUCCESS;
}

static AX_S32 os04a10_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U8 ex_h;
    AX_U8 ex_l;
    AX_U32 ex_ival = 0;
    AX_F32 ratio = 0.0f;
    AX_U32 nIntTimeFromUser = 0;
    AX_F32 real_ratio = 0.0f;
    AX_U32 real_lef = 0;
    AX_U32 real_sef1 = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptIntTimeTbl);

    SNS_DBG("Exptime:%d-%d-%d-%d, Hdrratio:%f-%f-%f-%f\n",
            ptIntTimeTbl->nIntTime[0], ptIntTimeTbl->nIntTime[1], ptIntTimeTbl->nIntTime[2], ptIntTimeTbl->nIntTime[3],
            ptIntTimeTbl->nHdrRatio[0], ptIntTimeTbl->nHdrRatio[1], ptIntTimeTbl->nHdrRatio[2], ptIntTimeTbl->nHdrRatio[3]);

    if (fabs(ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX]) < EPS) {
        SNS_ERR("hdr ratio is error \n");
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    if (sns_os04a10params[nPipeId].line_period_fixednum == 0U) {
        SNS_ERR("pos04a10Ctx->1h_period =0U (nFrameRate:%u, line_period:%u Division by zero !!!)\n",
            sns_obj->sns_mode_obj.nFrameRate, sns_os04a10params[nPipeId].line_period_fixednum);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    /* This sensor uses a ratio by default */
    ratio = ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX];
    ratio = AXSNS_CLIP3(ratio, sns_obj->sns_param.sns_ae_limit.nMinratio, sns_obj->sns_param.sns_ae_limit.nMaxratio);

    /* if 2DOL/3DOL, need re-limit s  frame MAX integration time dynamicly according current hdr ratio */
    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = ((sns_os04a10params[nPipeId].vts - 10) *
                sns_os04a10params[nPipeId].line_period_fixednum) * ((ratio) / (1 + ratio));

    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_3X_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = ((sns_os04a10params[nPipeId].vts - 12) *
                sns_os04a10params[nPipeId].line_period_fixednum) * ((ratio * ratio) / (1 + ratio + (ratio * ratio)));
    } else {
        // do nothing
    }

    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                   sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                   sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] - nIntTimeFromUser) < EPS) {
        //SNS_DBG("new and current integration time  is equal \n");
    } else {

        ex_ival = nIntTimeFromUser / sns_os04a10params[nPipeId].line_period_fixednum;
        ex_l = REG_LOW_8BITS(ex_ival);
        ex_h = REG_HIGH_8BITS(ex_ival);
        SNS_DBG("Exptime long frame: time: %d us,  %d line\n", nIntTimeFromUser, ex_ival);

        os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_EXP_LINE_H, ex_h);
        os04a10_sns_update_regs_table(nPipeId, OS04A10_LONG_EXP_LINE_L, ex_l);

        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = ex_ival * sns_os04a10params[nPipeId].line_period_fixednum;
    }

    /* if 2DOL/3DOL, ned re-limit s frame and or vs frame MAX integration time dynamicly according current hdr ratio */
    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
            sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = AXSNS_MIN3(sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 32 - sns_obj->sns_mode_obj.nHeight),
                                                                                        (SENSOR_MAX_LINE_GAP - os04a10_get_l2s_offset(nPipeId)) * sns_os04a10params[nPipeId].line_period_fixednum,
                                                                                        sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 2) - sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);
        } else {
            sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 2) -
                                                                                        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX];
        }
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_3X_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = ((sns_os04a10params[nPipeId].vts - 12) *
            sns_os04a10params[nPipeId].line_period_fixednum - nIntTimeFromUser) * (float)(ratio / (1 + ratio));
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_SHORT_FRAME_IDX] = ((sns_os04a10params[nPipeId].vts - 12) *
                sns_os04a10params[nPipeId].line_period_fixednum - nIntTimeFromUser) * (float)((float)1 / (1 + ratio));
    } else {
        // do nothing
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nIntTimeFromUser = nIntTimeFromUser / AXSNS_DIV_0_TO_1_FLOAT(ratio);
        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]);

        if (fabs(sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] - nIntTimeFromUser) < EPS) {
            SNS_DBG("new and current integration time  is equal \n");
        } else {
            if (sns_obj->sns_mode_obj.nFrameRate == 0U) {
                SNS_ERR("pos04a10Ctx->frame_rate =0U (Division by zero !!!)\n");
                return SNS_ERR_CODE_ILLEGAL_PARAMS;
            }

            ex_ival = nIntTimeFromUser / sns_os04a10params[nPipeId].line_period_fixednum;

            ex_l = REG_LOW_8BITS(ex_ival);
            ex_h = REG_HIGH_8BITS(ex_ival);

            SNS_DBG("Exptime medium frame: time: %d us,  %d line\n", nIntTimeFromUser, ex_ival);

            os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_EXP_LINE_H, ex_h);
            os04a10_sns_update_regs_table(nPipeId, OS04A10_SHORT_EXP_LINE_L, ex_l);

            /* 2DOL line gap */
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = ex_ival + os04a10_get_l2s_offset(nPipeId);
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGapTime[HDR_LONG_FRAME_IDX] =
                sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] * sns_os04a10params[nPipeId].line_period + 1;

            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = ex_ival * sns_os04a10params[nPipeId].line_period_fixednum;
        }

        ratio = (float)sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] /
                       sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX];

        real_lef = sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] +
                    sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_LONG_FRAME_IDX] * sns_os04a10params[nPipeId].line_period_fixednum;

        real_sef1 = sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] +
                    sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_MEDIUM_FRAME_IDX] * sns_os04a10params[nPipeId].line_period_fixednum;

        real_ratio = (float)real_lef / real_sef1;
    }

    if (IS_3DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
        nIntTimeFromUser = nIntTimeFromUser / AXSNS_DIV_0_TO_1_FLOAT(ratio * ratio);
        if (fabs(sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_SHORT_FRAME_IDX] - nIntTimeFromUser) < EPS) {
            SNS_DBG("new and current integration time  is equal \n");
        } else {

            nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                           sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_SHORT_FRAME_IDX],
                                           sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_SHORT_FRAME_IDX]);

            ex_ival = nIntTimeFromUser / sns_os04a10params[nPipeId].line_period_fixednum;

            ex_l = REG_LOW_8BITS(ex_ival);
            ex_h = REG_HIGH_8BITS(ex_ival);

            os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_EXP_LINE_H, ex_h);
            os04a10_sns_update_regs_table(nPipeId, OS04A10_VS_EXP_LINE_L, ex_l);

            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_SHORT_FRAME_IDX] = nIntTimeFromUser;
        }
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX] =
            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX];
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_MEDIUM_FRAME_IDX] =
            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX];
    }

    SNS_DBG("ratio:%.2f, real_ratio:%.2f, line_period:%.2f, linegap[0]:%d, exp:%.2f-%.2f, exp_offset:%.2f-%.2f, limit_exp:%.2f-%.2f-%.2f-%.2f\n",
        ratio, real_ratio, sns_os04a10params[nPipeId].line_period,
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX],
        (float)sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024,
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_LONG_FRAME_IDX],
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeOffset[HDR_MEDIUM_FRAME_IDX],
        (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX]/1024,
        (float)sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX]/1024);

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U8 exp_h;
    AX_U8 exp_l;
    AX_U32 expline_l = 0;
    AX_U32 expline_s = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SNS_CHECK_PTR_VALID(ptIntTimeTbl);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    exp_h = os04a10_reg_read(nPipeId, OS04A10_LONG_EXP_LINE_H);
    exp_l = os04a10_reg_read(nPipeId, OS04A10_LONG_EXP_LINE_L);
    expline_l = exp_h << 8 | exp_l;

    ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX] = expline_l * sns_os04a10params[nPipeId].line_period_fixednum;

    if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        exp_h = os04a10_reg_read(nPipeId, OS04A10_SHORT_EXP_LINE_H);
        exp_l = os04a10_reg_read(nPipeId, OS04A10_SHORT_EXP_LINE_L);
        expline_s = exp_h << 8 | exp_l;
        ptIntTimeTbl->nIntTime[HDR_MEDIUM_FRAME_IDX] = expline_s * sns_os04a10params[nPipeId].line_period_fixednum;
        ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX] =
            (float)ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX] / ptIntTimeTbl->nIntTime[HDR_MEDIUM_FRAME_IDX];
    }

    SNS_DBG("get current exp:%.2f-%.2f, expline:0x%x-0x%x, ratio[0]:%.2f, line_period:%.2f\n",
            (float)ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX]/1024,
            (float)ptIntTimeTbl->nIntTime[HDR_MEDIUM_FRAME_IDX]/1024,
            expline_l, expline_s,
            ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX],
            sns_os04a10params[nPipeId].line_period);

    return SNS_SUCCESS;
}

AX_S32 os04a10_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    AX_U8 hcglcg_value = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_HCG_MODE) && (nSnsHcgLcg == AX_LCG_MODE)) {
        SNS_DBG("switch to LCG mode\n");
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_LCG_MODE;
        hcglcg_value = os04a10_reg_read(nPipeId, 0x376c);
        hcglcg_value = hcglcg_value | 0x70; // l/s/vs hcg/lcg switch

        /* group hold */
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP1_IDX, 0x00);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP2_IDX, 0x01);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_HCG_LCG_IDX, hcglcg_value);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP3_IDX, 0x11);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP4_IDX, 0x05);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP5_IDX, 0x01);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP6_IDX, 0xA0);

    } else if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_LCG_MODE) && (nSnsHcgLcg == AX_HCG_MODE)) {
        SNS_DBG("switch to HCG mode \n");
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_HCG_MODE;
        hcglcg_value = os04a10_reg_read(nPipeId, 0x376c);
        hcglcg_value = hcglcg_value & 0x8F; // l/s/vs hcg/lcg switch

        /* group hold */
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP1_IDX, 0x00);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP2_IDX, 0x01);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_HCG_LCG_IDX, hcglcg_value);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP3_IDX, 0x11);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP4_IDX, 0x05);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP5_IDX, 0x01);
        os04a10_sns_update_group_hold_regs(nPipeId, AX_GROUP6_IDX, 0xA0);

    } else {
        /*SNS_DBG("current hcg/lcg mode : %d, Mode of ae library : %d \n", nSnsHcgLcg,
                sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg);*/
    }

    return SNS_SUCCESS;
}


static AX_S32 os04a10_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 os04a10_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}


static AX_S32 os04a10_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
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
        sns_os04a10params[nPipeId].vts = vts;
    } else {
        sns_os04a10params[nPipeId].vts = 1 * SNS_1_SECOND_UNIT_US / (sns_os04a10params[nPipeId].line_period * nFps);
    }

    if (sns_os04a10params[nPipeId].vts > OS04A10_MAX_VTS) {
        sns_os04a10params[nPipeId].vts = OS04A10_MAX_VTS;
        nFps = 1 * SNS_1_SECOND_UNIT_US / (sns_os04a10params[nPipeId].line_period * sns_os04a10params[nPipeId].vts);
        SNS_ERR("Beyond minmum fps  %f\n", nFps);
    }
    result = os04a10_set_vts(nPipeId, sns_os04a10params[nPipeId].vts);

    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (sns_os04a10params[nPipeId].vts - 8) *
                                                                            sns_os04a10params[nPipeId].line_period_fixednum;
    if (sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = AXSNS_MIN3(sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 32 - sns_obj->sns_mode_obj.nHeight),
                                                                                    (SENSOR_MAX_LINE_GAP - os04a10_get_l2s_offset(nPipeId)) * sns_os04a10params[nPipeId].line_period_fixednum,
                                                                                    sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 2) - sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);
    } else {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = sns_os04a10params[nPipeId].line_period_fixednum * (sns_os04a10params[nPipeId].vts - 2) -
                                                                                    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX];
    }

    sns_obj->sns_param.sns_ae_param.nCurFps = 1 * SNS_1_SECOND_UNIT_US / (sns_os04a10params[nPipeId].line_period *
            sns_os04a10params[nPipeId].vts);

    SNS_DBG("nPipeId = %d, fps(from alg) = %f, current vts = 0x%x\n", nPipeId, nFps, sns_os04a10params[nPipeId].vts);

    return SNS_SUCCESS;
}

static AX_S32 os04a10_ae_get_slow_shutter_param(ISP_PIPE_ID nPipeId, AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
{

    AX_S32 framerate = 30;
    AX_U32 nfps = 0;
    AX_U32 nVts = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;


    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    framerate = sns_obj->sns_mode_obj.nFrameRate;
    if (SNS_MAX_FRAME_RATE < framerate) {
        SNS_ERR("framerate out of range : %d\n", framerate);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    if (ax_sns_is_zero(sns_os04a10params[nPipeId].line_period_fixednum)) {
        SNS_ERR("line_period is zero : %u\n", sns_os04a10params[nPipeId].line_period_fixednum);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ptSlowShutterParam->nGroupNum = AXSNS_MIN((sizeof(gFpsGear) / sizeof(AX_F32)), framerate);
    ax_sns_quick_sort_float(gFpsGear, ptSlowShutterParam->nGroupNum);
    ptSlowShutterParam->fMinFps = AXSNS_MAX(gFpsGear[0],
                                            (1 * SNS_1_SECOND_UNIT_US / (sns_os04a10params[nPipeId].line_period * OS04A10_MAX_VTS)));

    for (nfps = 0 ; nfps < ptSlowShutterParam->nGroupNum; nfps++) {
        nVts = 1 * SNS_1_SECOND_UNIT_US / (sns_os04a10params[nPipeId].line_period * gFpsGear[nfps]);
        if ((AX_S32)gFpsGear[nfps] >= framerate) {
            nVts = sns_obj->sns_mode_obj.nVts;
        }
        if (nVts > OS04A10_MAX_VTS) {
            nVts = OS04A10_MAX_VTS;
            SNS_WRN("Beyond minmum fps  %f\n", ptSlowShutterParam->fMinFps);
        }

        if (IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime = (nVts  - 8) * sns_os04a10params[nPipeId].line_period_fixednum;
        } else if (IS_2DOL_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime = (AX_U32)((nVts  - 8) * sns_os04a10params[nPipeId].line_period_fixednum
                    * OS04A10_MAX_RATIO / (OS04A10_MAX_RATIO + 1));
        }

        ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps = 1 * SNS_1_SECOND_UNIT_US / (sns_os04a10params[nPipeId].line_period
                * nVts);
        ptSlowShutterParam->fMaxFps  =  ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps;

        SNS_DBG("nPipeId = %d, line_period = %.2f, fps = %.2f, nMaxIntTime = %d, vts=0x%x\n",
                nPipeId, sns_os04a10params[nPipeId].line_period,
                ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps,
                ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime, nVts);
    }

    return SNS_SUCCESS;
}

static AX_S32 os04a10_get_sensor_temperature_info(ISP_PIPE_ID nPipeId, AX_S32 *nTemperature)
{
    SNS_CHECK_PTR_VALID(nTemperature);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    AX_F32 nTempVal = 0.0;

    *nTemperature |= os04a10_reg_read(nPipeId, 0x4F06) << 8;
    *nTemperature |= os04a10_reg_read(nPipeId, 0x4F07);

    if (*nTemperature > OS04A10_TEMPERATURE_THRESHOLD) {
        *nTemperature = *nTemperature - OS04A10_TEMPERATURE_THRESHOLD;
        nTempVal = ax_int_convert_to_float(*nTemperature, 8, 8, 0);
        *nTemperature = -(AX_S32)(nTempVal * 1000);
    } else {
        nTempVal = ax_int_convert_to_float(*nTemperature, 8, 8, 0);
        *nTemperature = (AX_S32)(nTempVal * 1000);
    }

    SNS_DBG("nPipeId = %d, nTemperature = %f\n", nPipeId, *nTemperature/1000.0);

    return SNS_SUCCESS;
}

AX_SENSOR_REGISTER_FUNC_T gSnsos04a10Obj = {

    /* sensor ctrl */
    .pfn_sensor_reset                   = os04a10_sensor_reset,
    .pfn_sensor_chipid                  = os04a10_get_chipid,
    .pfn_sensor_init                    = os04a10_init,
    .pfn_sensor_exit                    = os04a10_exit,
    .pfn_sensor_streaming_ctrl          = os04a10_sensor_streaming_ctrl,
    .pfn_sensor_testpattern             = os04a10_testpattern_ctrl,

    .pfn_sensor_set_mode                = os04a10_sensor_set_mode,
    .pfn_sensor_get_mode                = os04a10_sensor_get_mode,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = os04a10_set_bus_info,

    /* default param */
    .pfn_sensor_get_default_params      = os04a10_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = os04a10_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = os04a10_set_sensor_params,
    .pfn_sensor_get_params              = os04a10_get_sensor_params,
    .pfn_sensor_get_gain_table          = os04a10_get_sensor_gain_table,
    .pfn_sensor_set_again               = os04a10_set_again,
    .pfn_sensor_get_again               = os04a10_get_again,
    .pfn_sensor_set_dgain               = os04a10_set_dgain,
    .pfn_sensor_get_dgain               = os04a10_get_dgain,
    .pfn_sensor_set_integration_time    = os04a10_set_integration_time,
    .pfn_sensor_get_integration_time    = os04a10_get_integration_time,
    .pfn_sensor_hcglcg_ctrl             = os04a10_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = os04a10_set_fps,
    .pfn_sensor_get_slow_shutter_param  = os04a10_ae_get_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = os04a10_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = os04a10_get_sensor_temperature_info,
};

