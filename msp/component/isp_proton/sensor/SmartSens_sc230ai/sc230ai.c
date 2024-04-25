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
#include "sc230ai_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "sc230ai_sdr.h"
#include "sc230ai_hdr_2x.h"


/****************************************************************************
 * golbal variables  and macro definition
 ****************************************************************************/

SNSSC230AI_OBJ sns_sc230aiparams[DEF_VIN_PIPE_MAX_NUM];
AX_SNS_COMMBUS_T gSc230aiBusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0} };

SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = AX_NULL)


static AX_S32 sc230ai_set_aecparam(ISP_PIPE_ID nPipeId);


/****************************************************************************
 * Internal function definition
 ****************************************************************************/
static AX_S32 sensor_ctx_init(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 ret = 0;

    SNS_DBG("sc230ai sensor_ctx_init. ret = %d\n", ret);
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
AX_S32 sc230ai_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
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

AX_S32 sc230ai_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
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
    sns_obj->sns_i2c_obj.slave_addr = SC230AI_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = SC230AI_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = SC230AI_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = SC230AI_SWAP_BYTES;


    sns_obj->sns_i2c_obj.sns_i2c_bnum = gSc230aiBusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

#if 0
    ret = sc230ai_get_chipid(nPipeId, &snsId);
    if (ret < 0) {
        SNS_ERR("can't find sc230ai sensor id.\r\n");
        return ret;
    } else {
        SNS_DBG("Sensor: sc230ai check chip id success.\r\n");
    }

#endif

    SNS_DBG("sc230ai i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}


static void sc230ai_init(ISP_PIPE_ID nPipeId)
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
    sc230ai_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    sc230ai_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    sc230ai_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;

    return;
}


static void sc230ai_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        SNS_ERR("sc230ai_exit sns_obj null\n");
        return;
    }

    i2c_exit(sns_obj->sns_i2c_obj.sns_i2c_fd);

    sensor_ctx_exit(nPipeId);
    return;
}


static AX_S32 sc230ai_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    gSc230aiBusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    sc230ai_reset(nPipeId, gSc230aiBusInfo[nPipeId].I2cDev);

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U16 sensor_id = 0;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= sc230ai_reg_read(nPipeId, 0x3107) << 8;
    sensor_id |= sc230ai_reg_read(nPipeId, 0x3108);

    SNS_DBG("%s: sensor sc230ai id: 0x%x\n", __func__, sensor_id);

    if (sensor_id != SC230AI_SENSOR_CHIP_ID) {
        SNS_ERR("%s: Failed to read sensor sc230ai id\n", __func__);
        return -1;
    }
    return 0;
}

static AX_S32 sc230ai_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}


static AX_S32 sc230ai_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
{
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    params->nSnsMode_caps = AX_SNS_LINEAR_MODE | AX_SNS_HDR_2X_MODE | AX_SNS_HDR_3X_MODE;
    params->nSnsRawType_caps = AX_SNS_RAWTYPE_12BIT;
    params->nSnsFps_caps = AX_SNS_15FPS | AX_SNS_30FPS | AX_SNS_60FPS;
    params->nSnsResolution_caps = AX_SNS_RES_3840_2160;
    return SNS_SUCCESS;
}


static AX_S32 sc230ai_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    /* sc230ai sensor not support hcg mode, nothing to do */
    return SNS_SUCCESS;
}

static AX_S32 sc230ai_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
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
    if ((AX_SNS_LINEAR_MODE != sns_mode->eSnsMode)
        && (AX_SNS_HDR_2X_MODE != sns_mode->eSnsMode)
        && (AX_SNS_HDR_3X_MODE != sns_mode->eSnsMode)) {
        hdrmode = AX_SNS_LINEAR_MODE;
    } else {
        hdrmode = sns_mode->eSnsMode;
    }

    if (width == 1920 && height == 1080 && hdrmode == AX_SNS_LINEAR_MODE && fps == 25) {
        sns_seting_index = e_SC230AI_1920x1080_MIPI_24Minput_2lane_405Mbps_10bit_25fps_sdr;
    } else if (width == 1920 && height == 1080 && hdrmode == AX_SNS_LINEAR_MODE && fps == 30) {
        sns_seting_index = e_SC230AI_1920x1080_MIPI_24Minput_2lane_405Mbps_10bit_30fps_sdr;
    } else if (width == 1920 && height == 1080 && hdrmode == AX_SNS_HDR_2X_MODE && fps == 25) {
        sns_seting_index = e_SC230AI_1920x1080_MIPI_24Minput_2lane_810Mbps_10bit_25fps_hdr;
    } else if (width == 1920 && height == 1080 && hdrmode == AX_SNS_HDR_2X_MODE && fps == 30) {
        sns_seting_index = e_SC230AI_1920x1080_MIPI_24Minput_2lane_810Mbps_10bit_30fps_hdr;
    } else {
        SNS_ERR("it's not supported. [%dx%d fps:%d mode=%d] \n", width, height, fps, hdrmode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("nPipeId=%d, sns_seting_index %d!\n", nPipeId, sns_seting_index);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = fps;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
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

static AX_S32 sc230ai_set_wdr_mode(ISP_PIPE_ID nPipeId, AX_SNS_HDR_MODE_E eHdrMode)
{
    return SNS_SUCCESS;
}

static AX_S32 sc230ai_stream_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
        result = sc230ai_reg_write(nPipeId, 0x0100, 0x01); // stream on
        SNS_DBG("sensor stream on!\n");
    } else {
        result = sc230ai_reg_write(nPipeId, 0x0100, 0x00); // stream off
        SNS_DBG("sensor stream off!\n");
    }
    if (0 != result) {
        return -1;
    }
    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("test pattern enable: %d\n", on);
    if (1 == on) {
        /* enable test-pattern */
        sc230ai_reg_write(nPipeId, 0x4501, 0xcc);  //fixme
    } else {
        /* disable test-pattern */
        sc230ai_reg_write(nPipeId, 0x4501, 0xc4);
    }

    return SNS_SUCCESS;
}


/****************************************************************************
 * get module default parameters function
 ****************************************************************************/
static AX_S32 sc230ai_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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
        ptDftParam->ptPfr            = &pfr_param_sdr;
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
        ptDftParam->ptWbGain        = &wbGain_param_hdr_2x;
        ptDftParam->ptRltm          = &rltm_param_hdr_2x;
        ptDftParam->ptDehaze        = &dehaze_param_hdr_2x;
        ptDftParam->ptNpu           = &npu_param_hdr_2x;
        ptDftParam->ptPfr            = &pfr_param_hdr_2x;
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

static AX_S32 sc230ai_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
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
static AX_S32 sc230ai_set_aecparam(ISP_PIPE_ID nPipeId)
{
    AX_U32 r1 = 0, r2 = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    sns_sc230aiparams[nPipeId].vts        = sc230ai_get_vts(nPipeId);
    sns_sc230aiparams[nPipeId].vts_s      = sc230ai_get_vts_s(nPipeId);
    sns_sc230aiparams[nPipeId].fps        = sns_obj->sns_mode_obj.nFrameRate;
    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
        sns_sc230aiparams[nPipeId].line_period = ((float)(1 / (float)(sns_sc230aiparams[nPipeId].vts) /
                sns_sc230aiparams[nPipeId].fps)
                * SNS_1_SECOND_UNIT_US);
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_sc230aiparams[nPipeId].line_period = ((float)(1 / (float)(sns_sc230aiparams[nPipeId].vts) /
                sns_sc230aiparams[nPipeId].fps)
                * SNS_1_SECOND_UNIT_US);
    } else {
        /* default config sdr mode params */
        sns_sc230aiparams[nPipeId].line_period = ((float)(1 / (float)(sns_sc230aiparams[nPipeId].vts) /
                sns_sc230aiparams[nPipeId].fps)
                * SNS_1_SECOND_UNIT_US);
    }

    sns_sc230aiparams[nPipeId].line_period_fixednum = ax_float_convert_to_int(sns_sc230aiparams[nPipeId].line_period, 22, 10, 0);

    SNS_DBG("fps:%d, line_period %fus, line_period_fixednum:%u ,vts:0x%x\n", sns_obj->sns_mode_obj.nFrameRate,
        sns_sc230aiparams[nPipeId].line_period, sns_sc230aiparams[nPipeId].line_period_fixednum,
        sns_sc230aiparams[nPipeId].vts);

    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_SEPARATION;
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = 4.05f;

    /* sensor again  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = 108.512f;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor dgain  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = 15.875f;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor medium again limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_MEDIUM_FRAME_IDX] = 108.512f;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_MEDIUM_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor medium dgain limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_MEDIUM_FRAME_IDX] = 15.875f;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_MEDIUM_FRAME_IDX] = (AX_F32)1 / 1024;

    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxgain = (AX_F32)108.512 * 15.875f;
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1.0f;

    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
        /* L Frame config */
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = (2 * sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (2 * (sns_sc230aiparams[nPipeId].vts - 9) *
            sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = sns_sc230aiparams[nPipeId].line_period_fixednum >> 1;
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        /* config sensor WithinFSC mode */
        if(sns_obj->sns_param.sns_dev_attr.eWithinBeyondfscSel == AX_SNS_WITHIN_MODE) {
            r1 = sc230ai_get_vts_s_r1(nPipeId);
            r2 = sc230ai_get_vts_s_r2(nPipeId);
            sns_sc230aiparams[nPipeId].vts_s = sns_sc230aiparams[nPipeId].vts - (3249 - 3248 + 1 + r1 - r2 + 1) * 2 - 32; //from sc
            sc230ai_set_vts_s(nPipeId, sns_sc230aiparams[nPipeId].vts_s);
            SNS_DBG("within mode vts_s:0x%x\n", sns_sc230aiparams[nPipeId].vts_s);
        }
        /* L Frame config */
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = (2 * sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = ((2 * sns_sc230aiparams[nPipeId].vts - 2 * sns_sc230aiparams[nPipeId].vts_s - 17) *
            sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = (4 * sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1 ;

        /* S Frame config */
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (2 * sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (2 * (sns_sc230aiparams[nPipeId].vts_s - 15) *
            sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_MEDIUM_FRAME_IDX] = (4 * sns_sc230aiparams[nPipeId].line_period_fixednum) >> 1;
    }

    sns_obj->sns_param.sns_ae_limit.nMinratio = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = 16.0f;
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 2.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = ax_float_convert_to_int(1000, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = ax_float_convert_to_int(1000, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return SNS_SUCCESS;
}


static AX_S32 sc230ai_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
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

    /* if 2DOL/3DOL, need re-limit s  frame MAX integration time dynamicly according current hdr ratio */
    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = AXSNS_MIN(
                    ((2 * sns_sc230aiparams[nPipeId].vts - 2 * sns_sc230aiparams[nPipeId].vts_s - 17) *
                     sns_sc230aiparams[nPipeId].line_period_fixednum / 2) * ((ratio) / (1 + ratio)),
                    (2 * sns_sc230aiparams[nPipeId].vts_s - 15) * sns_sc230aiparams[nPipeId].line_period_fixednum / 2 * ratio);
        sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 2 * sns_sc230aiparams[nPipeId].line_period_fixednum /
                2;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = 4 *
                sns_sc230aiparams[nPipeId].line_period_fixednum / 2;
    } else {
        // do nothing
    }

    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] == nIntTimeFromUser) {
        SNS_DBG("LEF new and current integration time  is equal \n");
    } else {
        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

        if (sns_sc230aiparams[nPipeId].line_period_fixednum == 0) {
            SNS_ERR("230aiCtx->1h_period =0U (Division by zero !!!)\n");
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }

        ex_ival = nIntTimeFromUser / (sns_sc230aiparams[nPipeId].line_period_fixednum >> 1);
        sc230ai_set_int_t_l(nPipeId, ex_ival);
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
        /* if 2DOL/3DOL, ned re-limit s frame and or vs frame MAX integration time dynamicly according current hdr ratio */
        if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
            sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (2 * sns_sc230aiparams[nPipeId].vts_s - 15)
                    * sns_sc230aiparams[nPipeId].line_period_fixednum / 2;
            sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX] = 2 * sns_sc230aiparams[nPipeId].line_period_fixednum /
                    2;
        } else {
            // do nothing
        }
    }

    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {

        nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
        nIntTimeFromUser = nIntTimeFromUser / AXSNS_DIV_0_TO_1_FLOAT(ratio);
        if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] == nIntTimeFromUser) {
            SNS_DBG("SEF new and current integration time  is equal \n");
        } else {
            nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                           sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_MEDIUM_FRAME_IDX],
                                           sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX]);
            if (sns_obj->sns_mode_obj.nFrameRate == 0U) {
                SNS_ERR("psc230aiCtx->frame_rate =0U (Division by zero !!!)\n");
                return SNS_ERR_CODE_ILLEGAL_PARAMS;
            }

            if (fabs(sns_sc230aiparams[nPipeId].line_period) < EPS) {
                SNS_ERR("1h_period == 0U (Division by zero !!!)\n");
                return SNS_ERR_CODE_ILLEGAL_PARAMS;
            }

            ex_ival = nIntTimeFromUser / (sns_sc230aiparams[nPipeId].line_period_fixednum >> 1);
            sc230ai_set_int_t_s(nPipeId, ex_ival);
            sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = nIntTimeFromUser;
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_MEDIUM_FRAME_IDX] = nIntTimeFromUser;
            /* 2DOL line gap */
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] = sc230ai_get_vts_s(nPipeId) / 2;
            sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGapTime[HDR_LONG_FRAME_IDX] =
                sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] * sns_sc230aiparams[nPipeId].line_period + 1;
        }
    }

    return SNS_SUCCESS;
}


static AX_S32 sc230ai_get_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    //now nothing to do
    return 0;
}

static AX_S32 sc230ai_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 Gain_in;
    AX_S32 result = 0;
    AX_F32 gain_val = 0;
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
        gain_val = sc230ai_again2value(nGainFromUser, &Gain_in);
        if (gain_val == -1) {
            SNS_ERR("new gain match failed \n");
        } else {
            result = sc230ai_sns_update_regs_table(nPipeId, SC230AI_AGAIN_L, Gain_in);
            if (result != 0) {
                SNS_ERR("write hw failed %d \n", result);
                return result;
            }
            sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = gain_val;
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
            gain_val = sc230ai_again2value(nGainFromUser, &Gain_in);
            if (gain_val == -1) {
                SNS_ERR(" new gain match failed \n");
            } else {
                result = sc230ai_sns_update_regs_table(nPipeId, SC230AI_AGAIN_S, Gain_in);
                if (result != 0) {
                    SNS_ERR("%s: write hw failed %d \n", __func__, result);
                    return result;
                }
                sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = gain_val;
            }
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_get_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 sc230ai_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    AX_U8 Gain_in;
    AX_U8 Gain_de;
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
        gain_val = sc230ai_dgain2value(nGainFromUser, &Gain_in, &Gain_de);
        if (gain_val == -1) {
            SNS_ERR("new gain match failed \n");
        } else {
            result = sc230ai_sns_update_regs_table(nPipeId, SC230AI_DGAIN_L, Gain_in);
            result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_DGAIN_FINE_L, Gain_de);
            if (result != 0) {
                SNS_ERR("write hw failed %d \n", result);
                return result;
            }
            sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = gain_val;
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
            gain_val = sc230ai_dgain2value(nGainFromUser, &Gain_in, &Gain_de);
            if (gain_val == -1) {
                SNS_ERR("new gain match failed \n");
            } else {
                result = sc230ai_sns_update_regs_table(nPipeId, SC230AI_DGAIN_S, Gain_in);
                result |= sc230ai_sns_update_regs_table(nPipeId, SC230AI_DGAIN_FINE_S, Gain_de);
                if (result != 0) {
                    SNS_ERR("write hw failed %d \n", result);
                    return result;
                }
                sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = gain_val;
            }
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 sc230ai_get_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 sc230ai_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 result = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result = sc230ai_get_gain_table(params);

    return result;
}

static AX_S32 sc230ai_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
{
    /* to do */
    return SNS_SUCCESS;
}

static AX_S32 sc230ai_ae_get_slow_shutter_param(ISP_PIPE_ID nPipeId, AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
{
    return SNS_SUCCESS;
}

AX_SENSOR_REGISTER_FUNC_T gSnssc230aiObj = {
    /* sensor ctrl */
    .pfn_sensor_reset                   = sc230ai_sensor_reset,
    .pfn_sensor_chipid                  = sc230ai_get_chipid,
    .pfn_sensor_init                    = sc230ai_init,
    .pfn_sensor_exit                    = sc230ai_exit,
    .pfn_sensor_streaming_ctrl          = sc230ai_stream_ctrl,
    .pfn_sensor_testpattern             = sc230ai_testpattern_ctrl,

    .pfn_sensor_set_mode                = sc230ai_set_mode,
    .pfn_sensor_get_mode                = sc230ai_get_mode,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = sc230ai_set_bus_info,
    .pfn_sensor_write_register          = sc230ai_reg_write,
    .pfn_sensor_read_register           = sc230ai_reg_read,

    /* default param */
    .pfn_sensor_get_default_params      = sc230ai_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = sc230ai_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = sc230ai_set_sensor_params,
    .pfn_sensor_get_params              = sc230ai_get_sensor_params,
    .pfn_sensor_get_gain_table          = sc230ai_get_sensor_gain_table,
    .pfn_sensor_set_again               = sc230ai_set_again,
    .pfn_sensor_set_dgain               = sc230ai_set_dgain,
    .pfn_sensor_get_again               = sc230ai_get_again,
    .pfn_sensor_get_dgain               = sc230ai_get_dgain,
    .pfn_sensor_set_integration_time    = sc230ai_set_integration_time,
    .pfn_sensor_get_integration_time    = sc230ai_get_integration_time,
    .pfn_sensor_hcglcg_ctrl             = sc230ai_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = sc230ai_set_fps,
    .pfn_sensor_get_slow_shutter_param  = sc230ai_ae_get_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = sc230ai_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = NULL,
};

