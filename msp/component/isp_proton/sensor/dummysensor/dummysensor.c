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
#include "dummysensor_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "dummysensor_sdr.h"
#include "dummysensor_hdr_2x.h"

/* Control print level */
static AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
static AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

/****************************************************************************
 * golbal variables  and macro definition                                                         *
 ****************************************************************************/

#ifndef EPS
    #define EPS         (1E-06)
#endif
#define SNS_1_SECOND_UNIT_US            (1000000)

static AX_SNS_COMMBUS_T g_DummyBusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0}};
SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = AX_NULL)

AX_S32 dummysensor_set_aecparam(ISP_PIPE_ID nPipeId);

/****************************************************************************
 * Internal function definition
 ****************************************************************************/
static AX_S32 sensor_ctx_init(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 ret = 0;

    SNS_DBG("dummysensor sensor_ctx_init. ret = %d\n", ret);
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
static void dummysensor_init(ISP_PIPE_ID nPipeId)
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

    /* 2. refresh ae param */
    dummysensor_set_aecparam(nPipeId);

    return;
}

static void dummysensor_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = NULL;
    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        SNS_ERR("dummysensor_exit sns_obj null\n");
        return;
    }

    sensor_ctx_exit(nPipeId);
    return;
}

static AX_S32 dummy_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    g_DummyBusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_streaming_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
    } else {
    }
    if (0 != result) {
        return -1;
    }
    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_sensor_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
{
    AX_S32 width;
    AX_S32 height;
    AX_S32 hdrmode;
    AX_S32 framerate = 25; // init value to 30fps, void null fps gives.
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

    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = framerate;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_sensor_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
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


static AX_S32 dummysensor_sensor_set_wdr_mode(ISP_PIPE_ID nPipeId, AX_SNS_HDR_MODE_E eHdrMode)
{
    return SNS_SUCCESS;
}

static AX_S32 dummysensor_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("test pattern enable: %d\n", on);
    if (1 == on) {
    } else {
    }

    return SNS_SUCCESS;
}

/****************************************************************************
 * get module default parameters function
 ****************************************************************************/
static AX_S32 dummy_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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

/* dummy sensor black level will overwrite by AX_ISP_SetPubAttr() */
static AX_S32 dummy_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level of linear mode */
    if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        ptBlackLevel->nBlackLevel[0] = 0;  /* linear mode 10bit sensor default blc 64(U10.0) --> 1024(U8.6) */
        ptBlackLevel->nBlackLevel[1] = 0;
        ptBlackLevel->nBlackLevel[2] = 0;
        ptBlackLevel->nBlackLevel[3] = 0;
    } else {
        ptBlackLevel->nBlackLevel[0] = 0;
        ptBlackLevel->nBlackLevel[1] = 0;
        ptBlackLevel->nBlackLevel[2] = 0;
        ptBlackLevel->nBlackLevel[3] = 0;
    }

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
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
 * exposure control function
 ****************************************************************************/
AX_S32 dummysensor_set_aecparam(ISP_PIPE_ID nPipeId)
{
    AX_F32 vs_line_period = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

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

    /* sensor sort again limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_SHORT_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_SHORT_FRAME_IDX] = 15.5;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_SHORT_FRAME_IDX] = (AX_F32)1 / 16;

    /* sensor sort dgain limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_SHORT_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_SHORT_FRAME_IDX] = 15.99;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_SHORT_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxgain = (AX_F32)15.5 * 15.99;
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1.0f;

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 20; // need calc by self
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = 33000;
    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = 10;

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 20;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 33000;
    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_MEDIUM_FRAME_IDX] = 10;

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_SHORT_FRAME_IDX] = 20;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_SHORT_FRAME_IDX] = 33000;
    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_SHORT_FRAME_IDX] = 10;

    sns_obj->sns_param.sns_ae_limit.nMinratio = 5.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = 16.0f;
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 2.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = 10000;
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 10000;
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_SHORT_FRAME_IDX] = 10000;
    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
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

    if (ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX];
    }

    if (ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = ptAnalogGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
    }

    if (ptAnalogGainTbl->nGain[HDR_SHORT_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_SHORT_FRAME_IDX] = ptAnalogGainTbl->nGain[HDR_SHORT_FRAME_IDX];
    }

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
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

    if (ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX];
    }

    if (ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = ptDigitalGainTbl->nGain[HDR_MEDIUM_FRAME_IDX];
    }

    if (ptDigitalGainTbl->nGain[HDR_SHORT_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_SHORT_FRAME_IDX] = ptDigitalGainTbl->nGain[HDR_SHORT_FRAME_IDX];
    }

    return SNS_SUCCESS;
}


static AX_S32 dummysensor_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U8 ex_h;
    AX_U8 ex_l;
    AX_U32 ex_ival = 0;
    AX_S32 result = 0;
    float ratio = 0.0f;
    AX_U32 nIntTimeFromUser = 0;

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

    if (ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX]) {
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    }

    if ((ptIntTimeTbl->nIntTime[HDR_MEDIUM_FRAME_IDX]) && (ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX])) {
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 
            ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX] / ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX];
    }

    if ((ptIntTimeTbl->nIntTime[HDR_SHORT_FRAME_IDX]) && (ptIntTimeTbl->nHdrRatio[HDR_MEDIUM_FRAME_IDX])) {
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_SHORT_FRAME_IDX] =
                sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] /ptIntTimeTbl->nHdrRatio[HDR_MEDIUM_FRAME_IDX];
    }

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_get_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    /* now nothing to do */
    return SNS_SUCCESS;
}

static AX_S32 dummysensor_get_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    /* now nothing to do */
    return SNS_SUCCESS;
}


static AX_S32 dummysensor_get_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    /* now nothing to do */
    return SNS_SUCCESS;
}


AX_S32 dummysensor_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    AX_U8 hcglcg_value = 0;
    AX_U32 result;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_HCG_MODE) && (nSnsHcgLcg == AX_LCG_MODE)) {
        SNS_DBG("%s: switch to LCG mode \n", __func__);
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_LCG_MODE;
    } else if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_LCG_MODE) && (nSnsHcgLcg == AX_HCG_MODE)) {
        SNS_DBG("%s: switch to HCG mode \n", __func__);
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_HCG_MODE;
    } else if (nSnsHcgLcg == AX_LCG_NOTSUPPORT_MODE) {
        SNS_DBG("%s: switch to LCG_NOTSUPPORT_MODE mode \n", __func__);
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_LCG_NOTSUPPORT_MODE;
    }

    return SNS_SUCCESS;
}


static AX_S32 dummysensor_get_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 dummysensor_set_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}

static AX_S32 dummysensor_get_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i;
    AX_S32 ret = 0;
    AX_S32 d_max = 0;
    AX_S32 d_min = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    if (!params)
        return -1;

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    d_max = log10(sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]) * (float)200/3;
    d_min = log10(sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX]) * (float)200/3;

    params->nAgainTableSize = d_max - d_min + 1;
    for ( int i = d_min; i < (d_max + 1); i++)
    {
         nAgainTable[i] = pow(10, i * (float)3/200);
         params->pAgainTable = nAgainTable;
    }

    for ( int i = d_min; i < (d_max + 1); i++)
    {
         nDgainTable[i] = pow(10, i * (float)3/200);
         params->pDgainTable = nDgainTable;
    }

    return ret;
}

static AX_S32 dummysensor_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 result = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result = dummysensor_get_gain_table(nPipeId, params);

    return result;
}


AX_SENSOR_REGISTER_FUNC_T gSnsdummyObj = {

    /* sensor ctrl */
    .pfn_sensor_chipid                  = AX_NULL,
    .pfn_sensor_init                    = dummysensor_init,
    .pfn_sensor_exit                    = dummysensor_exit,
    .pfn_sensor_streaming_ctrl          = dummysensor_streaming_ctrl,
    .pfn_sensor_testpattern             = dummysensor_testpattern_ctrl,

    .pfn_sensor_set_mode                = dummysensor_sensor_set_mode,
    .pfn_sensor_get_mode                = dummysensor_sensor_get_mode,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = dummy_set_bus_info,
    .pfn_sensor_write_register          = AX_NULL,
    .pfn_sensor_read_register           = AX_NULL,

    /* default param */
    .pfn_sensor_get_default_params      = dummy_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = dummy_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = dummysensor_set_params,
    .pfn_sensor_get_params              = dummysensor_get_params,
    .pfn_sensor_get_gain_table          = dummysensor_get_sensor_gain_table, // need check
    .pfn_sensor_set_again               = dummysensor_set_again,
    .pfn_sensor_set_dgain               = dummysensor_set_dgain,
    .pfn_sensor_get_again               = dummysensor_get_again,
    .pfn_sensor_get_dgain               = dummysensor_get_dgain,
    .pfn_sensor_set_integration_time    = dummysensor_set_integration_time,
    .pfn_sensor_get_integration_time    = dummysensor_get_integration_time,
    .pfn_sensor_hcglcg_ctrl             = dummysensor_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = AX_NULL,

};

