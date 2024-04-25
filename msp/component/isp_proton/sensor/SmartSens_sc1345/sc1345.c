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
#include "sc1345_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "sc1345_sdr.h"


/****************************************************************************
 * golbal variables  and macro definition
 ****************************************************************************/
SNSSC1345_OBJ gSc1345Params[DEF_VIN_PIPE_MAX_NUM];
AX_SNS_COMMBUS_T gSc1345BusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0} };

SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define SC1345_MAX_VTS         (0x7FFF)
#define SC1345_MAX_RATIO       (16.0f)
#define SC1345_MIN_RATIO       (1.0f)

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = AX_NULL)

/*user config*/
static AX_F32 gFpsGear[] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00, 11.00, 12.00, 13.00, 14.00, 15.00,
                        16.00, 17.00, 18.00, 19.00, 20.00, 21.00, 22.00, 23.00, 24.00, 25.00, 26.00, 27.00, 28.00, 29.00, 30.00};

static AX_S32 sc1345_set_aecparam(ISP_PIPE_ID nPipeId);


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
AX_S32 sc1345_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
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

AX_S32 sc1345_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
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
    sns_obj->sns_i2c_obj.slave_addr = SC1345_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = SC1345_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = SC1345_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = SC1345_SWAP_BYTES;


    sns_obj->sns_i2c_obj.sns_i2c_bnum = gSc1345BusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

#if 0
    ret = sc1345_get_chipid(nPipeId, &snsId);
    if (ret < 0) {
        SNS_ERR("can't find sc1345 sensor id.\r\n");
        return ret;
    } else {
        SNS_DBG("Sensor: sc1345 check chip id success.\r\n");
    }

#endif

    SNS_DBG("sc1345 i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}


static void sc1345_init(ISP_PIPE_ID nPipeId)
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
    sc1345_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    sc1345_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    sc1345_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;

    sns_obj->sns_mode_obj.nVts = sc1345_get_vts( nPipeId);

    return;
}


static void sc1345_exit(ISP_PIPE_ID nPipeId)
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


static AX_S32 sc1345_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    gSc1345BusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 sc1345_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    sc1345_reset(nPipeId, gSc1345BusInfo[nPipeId].I2cDev);

    return SNS_SUCCESS;
}

static AX_S32 sc1345_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U16 sensor_id = 0;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= sc1345_reg_read(nPipeId, 0x3107) << 8;
    sensor_id |= sc1345_reg_read(nPipeId, 0x3108);

    SNS_DBG("%s: sensor sc1345 id: 0x%x\n", __func__, sensor_id);

    if (sensor_id != SC1345_SENSOR_CHIP_ID) {
        SNS_ERR("%s: Failed to read sensor sc1345 id\n", __func__);
        return -1;
    }
    return 0;
}

static AX_S32 sc1345_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 sc1345_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}


static AX_S32 sc1345_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
{
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    params->nSnsMode_caps = AX_SNS_LINEAR_MODE;
    params->nSnsRawType_caps = AX_SNS_RAWTYPE_10BIT;
    params->nSnsFps_caps = AX_SNS_25FPS | AX_SNS_30FPS;
    params->nSnsResolution_caps = AX_SNS_RES_2880_1620;
    return SNS_SUCCESS;
}


static AX_S32 sc1345_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    /* sc1345 sensor not support hcg mode, nothing to do */
    return SNS_SUCCESS;
}

static AX_S32 sc1345_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
{
    AX_S32 result = 0;
    AX_S32 width;
    AX_S32 height;
    AX_S32 hdrmode;
    AX_S32 fps;
    AX_S32 sns_seting_index = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(sns_mode);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        /* contex init */
        result = sensor_ctx_init(nPipeId);
        if (0 != result) {
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
    hdrmode = AX_SNS_LINEAR_MODE;

    if (width == 1280 && height == 720 && fps == 25) {
        sns_seting_index = e_SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_25fps_RGGB;
    } else if (width == 1280 && height == 720 && fps == 30) {
        sns_seting_index = e_SC1345_MIPI_24Minput_1lane_378Mbps_10bit_1280x720_30fps_RGGB;
    } else {
        SNS_ERR("it's not supported. [%dx%d fps=%d] \n", width, height, fps);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("sns_seting_index %d!\n", sns_seting_index);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = fps;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return 0;
}

static AX_S32 sc1345_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
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

static AX_S32 sc1345_set_wdr_mode(ISP_PIPE_ID nPipeId, AX_SNS_HDR_MODE_E eHdrMode)
{
    return SNS_SUCCESS;
}
static AX_S32 sc1345_stream_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
        result |= sc1345_reg_write(nPipeId, 0x0100, 0x01); // sleep mode disable
    } else {
        result |= sc1345_reg_write(nPipeId, 0x0100, 0x00); // sleep mode enable
    }
    if (0 != result) {
        return -1;
    }

    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return 0;
}

static AX_S32 sc1345_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    if (1 == on) {
        /* enable test-pattern */
    } else {
        /* disable test-pattern */
    }

    return (result);
}


/****************************************************************************
 * get module default parameters function
 ****************************************************************************/
static AX_S32 sc1345_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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

    SNS_DBG(" current hdr mode %d..\n", nHdrmode);

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
    }

    return SNS_SUCCESS;
}

static AX_S32 sc1345_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level of linear mode */
    if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        ptBlackLevel->nBlackLevel[0] = 1024;    /* actual test 16(U8.0) --> 1024(U8.6) */
        ptBlackLevel->nBlackLevel[1] = 1024;
        ptBlackLevel->nBlackLevel[2] = 1024;
        ptBlackLevel->nBlackLevel[3] = 1024;
    } else {
        SNS_ERR("no support current hdr mode:%d !\n", sns_obj->sns_mode_obj.eHDRMode);
        return SNS_ERR_CODE_FAILED;
    }

    return SNS_SUCCESS;
}


/****************************************************************************
 * exposure control function
 ****************************************************************************/
static AX_S32 sc1345_set_aecparam(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    gSc1345Params[nPipeId].vts        = sc1345_get_vts(nPipeId);
    gSc1345Params[nPipeId].fps        = sns_obj->sns_mode_obj.nFrameRate;
    gSc1345Params[nPipeId].line_period = (float)(1 / (float)(gSc1345Params[nPipeId].vts) / 
                        gSc1345Params[nPipeId].fps) * SNS_1_SECOND_UNIT_US;
    gSc1345Params[nPipeId].line_period_fixednum = ax_float_convert_to_int(gSc1345Params[nPipeId].line_period, 22, 10, 0);

    SNS_DBG("fps:%d, line_period %fus, line_period_fixednum:%u ,vts:0x%x\n", sns_obj->sns_mode_obj.nFrameRate,
        gSc1345Params[nPipeId].line_period, gSc1345Params[nPipeId].line_period_fixednum,
        gSc1345Params[nPipeId].vts);

    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_SEPARATION;
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = 4.05f;

    /* sensor again  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = 15.75f;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 128;

    /* sensor dgain  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = 31.5f;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 128;

    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxgain = (AX_F32)15.75 * 31.5f;
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1.0f;

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 0;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = 2 * (gSc1345Params[nPipeId].vts - 8) *
            gSc1345Params[nPipeId].line_period_fixednum / 2;

    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = 
            gSc1345Params[nPipeId].line_period_fixednum / 2;

    sns_obj->sns_param.sns_ae_limit.nMinratio = SC1345_MIN_RATIO;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = SC1345_MAX_RATIO;
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 2.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = ax_float_convert_to_int(1000, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = ax_float_convert_to_int(1000, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return SNS_SUCCESS;
}


static AX_S32 sc1345_get_fps()
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 sc1345_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U8 ex_h;
    AX_U8 ex_m;
    AX_U8 ex_l;
    AX_U32 ex_ival = 0;
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

    /* This sensor uses a ratio by default */
    ratio = ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX];
    ratio = AXSNS_CLIP3(ratio, sns_obj->sns_param.sns_ae_limit.nMinratio, sns_obj->sns_param.sns_ae_limit.nMaxratio);

    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] == nIntTimeFromUser) {
        //SNS_DBG("new and current integration time  is equal \n");
    } else {

        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

        if (fabs(gSc1345Params[nPipeId].line_period) < EPS) {
            SNS_ERR("sc1345 line_period = 0 is invalid !\n");
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }

        ex_ival = nIntTimeFromUser / (gSc1345Params[nPipeId].line_period_fixednum >> 1);
        sc1345_set_int_t_l(nPipeId, ex_ival);
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
    }

    return SNS_SUCCESS;
}

static AX_S32 sc1345_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 gain = 0;
    AX_U8 gain_fine = 0;
    AX_F32 gain_value = 0;
    AX_S32 result = 0;
    AX_F32 nGainFromUser = 0;
    AX_U8 again_calib_0 = 0;
    AX_U8 again_calib_1 = 0;

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
        gain_value = sc1345_again2value(nGainFromUser, &gain, &gain_fine);
        if (gain_value == -1) {
            SNS_ERR("new again:%f match failed !\n", nGainFromUser);
        } else {
            //calib from again
            if(gain_value < 2.0) //[gain<2]
            {
                again_calib_0 = 0x0c;
                again_calib_1 = 0xc6;
            }
            else if(gain_value < 4.0) //[2=<gain<4]
            {
                again_calib_0 = 0x16;
                again_calib_1 = 0xc6;
            }
            else if(gain_value < 8.0) //[4=<gain<8]
            {
                again_calib_0 = 0x16;
                again_calib_1 = 0xd6;
            }
            else //again >= 8.0
            {
                again_calib_0 = 0x1d;
                again_calib_1 = 0xd6;
            }

            result |= sc1345_sns_update_regs_table(nPipeId, SC1345_AGAIN_CALIB_0, again_calib_0);
            result |= sc1345_sns_update_regs_table(nPipeId, SC1345_AGAIN_CALIB_1, again_calib_1);
            result |= sc1345_sns_update_regs_table(nPipeId, SC1345_AGAIN, (gain & 0xFF));
            result |= sc1345_sns_update_regs_table(nPipeId, SC1345_AGAIN_FINE, (gain_fine & 0xFF));
            if (result != 0) {
                SNS_ERR("%s: update again:%f failed ret:%d.\n", __func__, nGainFromUser, result);
                return result;
            }
            sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = gain_value;
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 sc1345_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    AX_U8 gain = 0;
    AX_U8 gain_fine = 0;
    AX_F32 gain_value = 0;
    AX_S32 result = 0;
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
        /* nothing to do */
    } else {
        gain_value = sc1345_dgain2value(nGainFromUser, &gain, &gain_fine);
        if (gain_value == -1) {
            SNS_ERR("new dgain:%f match failed !\n", nGainFromUser);
        } else {
            result |= sc1345_sns_update_regs_table(nPipeId, SC1345_DGAIN, (gain & 0xFF));
            result |= sc1345_sns_update_regs_table(nPipeId, SC1345_DGAIN_FINE, (gain_fine & 0xFF));
            if (result != 0) {
                SNS_ERR("%s: update dgain:%f failed ret:%d.\n", __func__, nGainFromUser, result);
                return result;
            }

            sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = gain_value;
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 sc1345_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i = 0;

    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    return sc1345_get_gain_table(params);
}


static AX_S32 sc1345_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
{
    AX_S32 result = 0;
    AX_S32 framerate = 30;
    AX_U32 vts =0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    framerate =  sns_obj->sns_mode_obj.nFrameRate;
    vts = sns_obj->sns_mode_obj.nVts;
    if (nFps >= framerate) {
        gSc1345Params[nPipeId].vts = vts ;
    } else {
        gSc1345Params[nPipeId].vts = 1 * SNS_1_SECOND_UNIT_US / (gSc1345Params[nPipeId].line_period * nFps) ;
    }

    if (gSc1345Params[nPipeId].vts > SC1345_MAX_VTS){
        gSc1345Params[nPipeId].vts = SC1345_MAX_VTS;
        nFps = 1 * SNS_1_SECOND_UNIT_US / (gSc1345Params[nPipeId].line_period *gSc1345Params[nPipeId].vts );
        SNS_ERR("Beyond minmum fps  %f\n",nFps);
    }
    result = sc1345_set_vts(nPipeId, gSc1345Params[nPipeId].vts);

    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] =
           (2 * (gSc1345Params[nPipeId].vts - 8)  *  gSc1345Params[nPipeId].line_period_fixednum) >> 1;
    sns_obj->sns_param.sns_ae_param.nCurFps = 1 * SNS_1_SECOND_UNIT_US / 
            (gSc1345Params[nPipeId].line_period *gSc1345Params[nPipeId].vts );

    SNS_DBG("nPipeId = %d, fps(from alg) = %f, current vts = 0x%x\n", nPipeId, nFps, gSc1345Params[nPipeId].vts);

    return SNS_SUCCESS;
}


static AX_S32 sc1345_ae_get_slow_shutter_param(ISP_PIPE_ID nPipeId, AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
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

    if(ax_sns_is_zero(gSc1345Params[nPipeId].line_period)) {
        SNS_ERR("line_period is zero : %.2f\n", gSc1345Params[nPipeId].line_period);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ptSlowShutterParam->nGroupNum = AXSNS_MIN((sizeof(gFpsGear) / sizeof(AX_F32)), framerate);
    ax_sns_quick_sort_float(gFpsGear, ptSlowShutterParam->nGroupNum);
    ptSlowShutterParam->fMinFps =AXSNS_MAX(gFpsGear[0], (1 * SNS_1_SECOND_UNIT_US / (gSc1345Params[nPipeId].line_period * SC1345_MAX_VTS)));

    for (nfps = 0 ; nfps < ptSlowShutterParam->nGroupNum; nfps++) {
        nVts = 1 * SNS_1_SECOND_UNIT_US / (gSc1345Params[nPipeId].line_period * gFpsGear[nfps]);
        if((AX_S32)gFpsGear[nfps] >= framerate) {
            nVts = sns_obj->sns_mode_obj.nVts;
        }
        if (nVts > SC1345_MAX_VTS) {
            nVts = SC1345_MAX_VTS;
            SNS_WRN("Beyond minmum fps  %f\n", ptSlowShutterParam->fMinFps);
        }

        ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime = 2 * (AX_U32)((nVts - 8) * gSc1345Params[nPipeId].line_period / 2);
        ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps = 1 * SNS_1_SECOND_UNIT_US / (gSc1345Params[nPipeId].line_period * nVts);
        ptSlowShutterParam->fMaxFps  =  ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps;

        SNS_DBG("nPipeId = %d, line_period = %.2f, fps = %.2f, nMaxIntTime = %d, vts=0x%x\n",
                nPipeId, gSc1345Params[nPipeId].line_period,
                ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps,
                ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime, nVts);
    }
    return SNS_SUCCESS;
}


AX_SENSOR_REGISTER_FUNC_T gSnssc1345Obj = {
    /* sensor ctrl */
    .pfn_sensor_reset                   = sc1345_sensor_reset,
    .pfn_sensor_chipid                  = sc1345_get_chipid,
    .pfn_sensor_init                    = sc1345_init,
    .pfn_sensor_exit                    = sc1345_exit,
    .pfn_sensor_streaming_ctrl          = sc1345_stream_ctrl,
    .pfn_sensor_testpattern             = sc1345_testpattern_ctrl,

    .pfn_sensor_set_mode                = sc1345_set_mode,
    .pfn_sensor_get_mode                = sc1345_get_mode,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = sc1345_set_bus_info,
    .pfn_sensor_write_register          = sc1345_reg_write,
    .pfn_sensor_read_register           = sc1345_reg_read,

    /* default param */
    .pfn_sensor_get_default_params      = sc1345_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = sc1345_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = sc1345_set_sensor_params,
    .pfn_sensor_get_params              = sc1345_get_sensor_params,
    .pfn_sensor_get_gain_table          = sc1345_get_sensor_gain_table,
    .pfn_sensor_set_again               = sc1345_set_again,
    .pfn_sensor_set_dgain               = sc1345_set_dgain,
    .pfn_sensor_set_integration_time    = sc1345_set_integration_time,
    .pfn_sensor_hcglcg_ctrl             = sc1345_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = sc1345_set_fps,
    .pfn_sensor_get_slow_shutter_param  = sc1345_ae_get_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = sc1345_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = NULL,
};

