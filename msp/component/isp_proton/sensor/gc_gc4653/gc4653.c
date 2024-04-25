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
#include "gc4653_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "gc4653_sdr.h"

/****************************************************************************
 * golbal variables  and macro definition                                                         *
 ****************************************************************************/
#define SNS_1_SECOND_UNIT_US            (1000000)

SNSGC4653_OBJ sns_gc4653params[DEF_VIN_PIPE_MAX_NUM];
AX_SNS_COMMBUS_T gGc4653BusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0}};

SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define GC4653_MAX_VTS         (0x3FFF)
#define GC4653_MAX_RATIO       (16.0f)
#define GC4653_MIN_RATIO       (1.0f)

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = AX_NULL)

/*user config*/
static AX_F32 gFpsGear[] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00, 11.00, 12.00, 13.00, 14.00, 15.00,
                        16.00, 17.00, 18.00, 19.00, 20.00, 21.00, 22.00, 23.00, 24.00, 25.00, 26.00, 27.00, 28.00, 29.00, 30.00};

static AX_S32 gc4653_set_aecparam(ISP_PIPE_ID nPipeId);

/****************************************************************************
 * Internal function definition
 ****************************************************************************/
static AX_S32 sensor_ctx_init(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 ret = 0;

    SNS_DBG("gc4653 sensor_ctx_init. ret = %d\n", ret);
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
static AX_S32 gc4653_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    gc4653_reset(nPipeId, gGc4653BusInfo[nPipeId].I2cDev);

    return SNS_SUCCESS;
}

static AX_S32 gc4653_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U16 sensor_id = 0;
    AX_U16 val = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= gc4653_reg_read(nPipeId, 0x03f0) << 8;
    sensor_id |= gc4653_reg_read(nPipeId, 0x03f1);

    SNS_DBG("%s: sensor gc4653 id: 0x%x\n", __func__, sensor_id);

    *pSnsId = sensor_id;

    if (sensor_id != GC4653_SENSOR_CHIP_ID) {
        printf("%s: Failed to read sensor gc4653 id\n", __func__);
        return -1;
    }

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
    sns_obj->sns_i2c_obj.slave_addr = GC4653_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = GC4653_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = GC4653_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = GC4653_SWAP_BYTES;

    sns_obj->sns_i2c_obj.sns_i2c_bnum = gGc4653BusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

    ret = gc4653_get_chipid(nPipeId, &snsId);
    if (ret !=  SNS_SUCCESS) {
        SNS_ERR("can't find gc4653 sensor id.\r\n");
        return ret;
    } else {
        SNS_DBG("Sensor: gc4653 check chip id success.\r\n");
    }

    SNS_DBG("gc4653 i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}

AX_S32 gc4653_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
{
    AX_U8 data;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 nRet = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (-1 == sns_obj->sns_i2c_obj.sns_i2c_fd)
        return SNS_ERR_CODE_ILLEGAL_PARAMS;

    i2c_read(sns_obj->sns_i2c_obj.sns_i2c_fd,
             sns_obj->sns_i2c_obj.slave_addr,
             addr,
             sns_obj->sns_i2c_obj.address_byte,
             (AX_U8 *)(&data),
             sns_obj->sns_i2c_obj.data_byte,
             sns_obj->sns_i2c_obj.swap_byte);

    return data;
}

AX_S32 gc4653_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
{
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (-1 == sns_obj->sns_i2c_obj.sns_i2c_fd) {
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    nRet = i2c_write(sns_obj->sns_i2c_obj.sns_i2c_fd, sns_obj->sns_i2c_obj.slave_addr, addr,
                     sns_obj->sns_i2c_obj.address_byte,
                     (AX_U8 *)(&data), sns_obj->sns_i2c_obj.data_byte, sns_obj->sns_i2c_obj.swap_byte);

    return nRet;
}

static void gc4653_init(ISP_PIPE_ID nPipeId)
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
    gc4653_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    gc4653_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    gc4653_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;

    sns_obj->sns_mode_obj.nVts = gc4653_get_vts( nPipeId);

    return;
}

static void gc4653_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = NULL;
    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        SNS_ERR("gc4653_exit sns_obj null\n");
        return;
    }

    i2c_exit(sns_obj->sns_i2c_obj.sns_i2c_fd);

    sensor_ctx_exit(nPipeId);
    return;
}

static AX_S32 gc4653_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    gGc4653BusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 gc4653_sensor_streaming_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if (1 == on) {
        result = gc4653_reg_write(nPipeId, 0x0100, 0x09); // stream on
        SNS_DBG("sensor stream on!\n");
    } else {
        result = gc4653_reg_write(nPipeId, 0x0100, 0x00); // stream off
        SNS_DBG("sensor stream off!\n");
    }
    if (0 != result) {
        return -1;
    }
    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return SNS_SUCCESS;
}

static AX_S32 gc4653_sensor_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
{
    AX_S32 width;
    AX_S32 height;
    AX_S32 hdrmode;
    AX_S32 framerate = 30; // init value to 30fps, void null fps gives.
    AX_S32 sns_seting_index = 0;
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    printf("gc4653_sensor_set_mode\n");

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
    if ((1 != sns_mode->eSnsMode) &&
        (2 != sns_mode->eSnsMode) &&
        (3 != sns_mode->eSnsMode)) {
        hdrmode = 1;
    } else {
        hdrmode = sns_mode->eSnsMode;
    }

    if (width == 2560 && height == 1440  && framerate == 30) {
        sns_seting_index = Allpixel_CSI2_2lane_648Mbps_24MHz_AD10bit_Master_LCG_30fps;
    } else if (width == 2560 && height == 1440  && framerate == 25) {
        sns_seting_index = Allpixel_CSI2_2lane_648Mbps_24MHz_AD10bit_Master_LCG_25fps;
    } else {
        SNS_ERR("%s it's not supported. [%dx%d mode=%d] \n",
                __func__, width, height, hdrmode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("sns_seting_index %d!, hdrmode = %d\n", sns_seting_index, hdrmode);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = framerate;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return SNS_SUCCESS;
}

static AX_S32 gc4653_sensor_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
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

static AX_S32 gc4653_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("test pattern enable, not supported: %d\n", on);
    if (1 == on) {
        /* enable test-pattern */
    } else {
        /* disable test-pattern */
    }

    return SNS_SUCCESS;
}

static AX_S32 gc4653_mirror_flip(ISP_PIPE_ID nPipeId, AX_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
    switch (eSnsMirrorFlip)
    {
        default:
        case AX_SNS_MF_NORMAL:
            gc4653_reg_write(nPipeId, 0x0101, 0x0);
            break;

        case AX_SNS_MF_MIRROR:
            gc4653_reg_write(nPipeId, 0x0101, 0x01);
            break;

        case AX_SNS_MF_FLIP:
            gc4653_reg_write(nPipeId, 0x0101, 0x02);
            break;

        case AX_SNS_MF_MIRROR_FLIP:
            gc4653_reg_write(nPipeId, 0x0101, 0x03);
            break;
    }
    return SNS_SUCCESS;
}

static AX_S32 gc4653_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
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
static AX_S32 gc4653_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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
        ptDftParam->ptAeDftParam    = &ae_param_sdr;
        ptDftParam->ptAwbDftParam   = &awb_param_sdr;
        break;
    }

    return SNS_SUCCESS;
}

static AX_S32 gc4653_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level LCL */
    /* black level of linear mode */
    if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        ptBlackLevel->nBlackLevel[0] = 1024;
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
static AX_S32 gc4653_set_aecparam(ISP_PIPE_ID nPipeId)
{
    AX_F32 vs_line_period = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    sns_gc4653params[nPipeId].hts      = gc4653_get_hts(nPipeId);
    sns_gc4653params[nPipeId].vs_hts   = gc4653_get_hts(nPipeId);
    sns_gc4653params[nPipeId].vts      = gc4653_get_vts(nPipeId);
    sns_gc4653params[nPipeId].sclk     = 144;  //Mbps

    sns_gc4653params[nPipeId].line_period = sns_gc4653params[nPipeId].hts / sns_gc4653params[nPipeId].sclk;
    sns_gc4653params[nPipeId].line_period_fixednum = ax_float_convert_to_int(sns_gc4653params[nPipeId].line_period, 22, 10, 0);

    SNS_DBG("fps:%d, line_period %fus, line_period_fixednum:%u ,hts:0x%x, vts:0x%x\n", sns_obj->sns_mode_obj.nFrameRate,
        sns_gc4653params[nPipeId].line_period, sns_gc4653params[nPipeId].line_period_fixednum,
        sns_gc4653params[nPipeId].hts, sns_gc4653params[nPipeId].vts);

    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_SEPARATION;
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = 1.0f;

    /* sensor again  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = 75.828;
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 4;

    /* sensor dgain  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = 15.89;
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1 / 64;

    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = 1.0f;
    sns_obj->sns_param.sns_ae_limit.nMaxgain = (AX_F32)75.828 * 15.89;
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (AX_F32)1.0f;

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = sns_gc4653params[nPipeId].line_period_fixednum;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = \
            sns_gc4653params[nPipeId].line_period_fixednum * (sns_gc4653params[nPipeId].vts - 8);
    sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = sns_gc4653params[nPipeId].line_period_fixednum;

    sns_obj->sns_param.sns_ae_limit.nMinratio = GC4653_MIN_RATIO;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = GC4653_MAX_RATIO;
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = ax_float_convert_to_int(0.001f * SNS_1_SECOND_UNIT_US, 22, 10, 0);
    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return SNS_SUCCESS;
}

static AX_S32 gc4653_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 result = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result = gc4653_get_gain_table(params);
    return result;
}

static AX_S32 gc4653_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 Gain_in;
    AX_U8 Gain_de;
    AX_S32 result = 0;
    AX_F32 gain_value = 0;
    AX_F32 nGainFromUser = 0;
    AX_U8 Gain_par[7];

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
        // need realize
        gain_value = gc4653_again2value(nGainFromUser, Gain_par);
        if (gain_value == -1) {
            SNS_ERR("userAgain:%f, new gain match failed \n", nGainFromUser);
            return -1;
        } else {
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN0, Gain_par[0]);
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN1, Gain_par[1] & 0x3);
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN2, Gain_par[2]);
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN3, Gain_par[3] & 0x3F);
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN4, Gain_par[4]);
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN5, Gain_par[5]);
            gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_AGAIN6, Gain_par[6]);

            sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = gain_value;

            SNS_DBG("userAgain:%f, gain_value:%f, 0x%x-0x%x(0x%x)-0x%x-0x%x(0x%x)-0x%x-0x%x-0x%x\n",
                ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX], gain_value, Gain_par[0], Gain_par[1] & 0x3, Gain_par[1],
                Gain_par[2], Gain_par[3] & 0x3F, Gain_par[3], Gain_par[4], Gain_par[5], Gain_par[6]);
        }
    }

    return SNS_SUCCESS;
}

static AX_S32 gc4653_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
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
        gc4653_dgain2value(nGainFromUser, &Gain_in, &Gain_de);
        gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_DGAIN0, Gain_in & 0x0F);
        gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_DGAIN1, Gain_de & 0x3F);

        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = nGainFromUser;

        SNS_DBG("userDgain:%f, 0x%x(0x%x)-0x%x(0x%x)\n", ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX],
                Gain_in & 0x0F, Gain_in, Gain_de & 0x3F, Gain_de);
    }

    return SNS_SUCCESS;
}


static AX_S32 gc4653_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
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

    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] == nIntTimeFromUser) {
        //SNS_DBG("new and current integration time  is equal \n");
    } else {
        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

        if (sns_gc4653params[nPipeId].line_period_fixednum == 0) {
            SNS_ERR("gc4653 Ctx->1h_period =0U (Division by zero !!!)\n");
            return SNS_ERR_CODE_ILLEGAL_PARAMS;
        }

        ex_ival = nIntTimeFromUser / sns_gc4653params[nPipeId].line_period_fixednum;
        ex_l = REG_LOW_8BITS(ex_ival);
        ex_h = REG_HIGH_8BITS(ex_ival);
        gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_EXP_LINE_H, ex_h & 0x3F);
        gc4653_sns_update_regs_table(nPipeId, GC4653_LONG_EXP_LINE_L, ex_l);

        SNS_DBG("LEF nIntTimeFromUser:%d, 0x%x[0x%x(0x%x)-0x%x]\n", nIntTimeFromUser, ex_ival, ex_h & 0x3F, ex_h, ex_l);

        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
    }

    return SNS_SUCCESS;
}

static AX_S32 gc4653_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    return SNS_SUCCESS;
}

static AX_S32 gc4653_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 gc4653_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}


static AX_S32 gc4653_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
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
        sns_gc4653params[nPipeId].vts = vts ;
    } else {
        sns_gc4653params[nPipeId].vts = 1 * SNS_1_SECOND_UNIT_US / (sns_gc4653params[nPipeId].line_period * nFps);
    }

    if (sns_gc4653params[nPipeId].vts > GC4653_MAX_VTS){
        sns_gc4653params[nPipeId].vts = GC4653_MAX_VTS;
        nFps = 1 * SNS_1_SECOND_UNIT_US / (sns_gc4653params[nPipeId].line_period *sns_gc4653params[nPipeId].vts );
        SNS_ERR("Beyond minmum fps  %f\n",nFps);
    }
    result = gc4653_set_vts(nPipeId, sns_gc4653params[nPipeId].vts);
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = \
                sns_gc4653params[nPipeId].line_period_fixednum * (sns_gc4653params[nPipeId].vts - 8);
    sns_obj->sns_param.sns_ae_param.nCurFps = \
        1 * SNS_1_SECOND_UNIT_US / (sns_gc4653params[nPipeId].line_period *sns_gc4653params[nPipeId].vts );

    SNS_DBG("nPipeId = %d, fps(from alg) = %f, current vts = 0x%x\n", nPipeId, nFps, sns_gc4653params[nPipeId].vts);

    return SNS_SUCCESS;
}

static AX_S32 gc4653_ae_get_slow_shutter_param(ISP_PIPE_ID nPipeId, AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
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

    if(ax_sns_is_zero(sns_gc4653params[nPipeId].line_period)) {
        SNS_ERR("line_period is zero : %f\n", sns_gc4653params[nPipeId].line_period);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ptSlowShutterParam->nGroupNum = AXSNS_MIN((sizeof(gFpsGear) / sizeof(AX_F32)), framerate);
    ax_sns_quick_sort_float(gFpsGear, ptSlowShutterParam->nGroupNum);
    ptSlowShutterParam->fMinFps =AXSNS_MAX(gFpsGear[0], (1 * SNS_1_SECOND_UNIT_US / (sns_gc4653params[nPipeId].line_period * GC4653_MAX_VTS)));

    for (nfps = 0 ; nfps < ptSlowShutterParam->nGroupNum; nfps++) {
        nVts = 1 * SNS_1_SECOND_UNIT_US / (sns_gc4653params[nPipeId].line_period * gFpsGear[nfps]);
        if((AX_S32)gFpsGear[nfps] >= framerate) {
            nVts = sns_obj->sns_mode_obj.nVts;
        }
        if (nVts > GC4653_MAX_VTS) {
            nVts = GC4653_MAX_VTS;
            SNS_WRN("Beyond minmum fps  %f\n", ptSlowShutterParam->fMinFps);
        }

        ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime = (nVts - 8) * sns_gc4653params[nPipeId].line_period_fixednum;
        ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps = 1 * SNS_1_SECOND_UNIT_US /
                                                            (sns_gc4653params[nPipeId].line_period * nVts);
        ptSlowShutterParam->fMaxFps  =  ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps;

        SNS_DBG("nPipeId = %d, line_period = %f, fps = %f, nMaxIntTime = %d, vts=0x%x\n",
                nPipeId, sns_gc4653params[nPipeId].line_period,
                ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps,
                ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime, nVts);
	}
    return SNS_SUCCESS;
}


AX_SENSOR_REGISTER_FUNC_T gSnsgc4653Obj = {

    /* sensor ctrl */
    .pfn_sensor_reset                   = gc4653_sensor_reset,
    .pfn_sensor_chipid                  = gc4653_get_chipid,
    .pfn_sensor_init                    = gc4653_init,
    .pfn_sensor_exit                    = gc4653_exit,
    .pfn_sensor_streaming_ctrl          = gc4653_sensor_streaming_ctrl,
    .pfn_sensor_testpattern             = gc4653_testpattern_ctrl,
    .pfn_sensor_mirror_flip             = gc4653_mirror_flip,

    .pfn_sensor_set_mode                = gc4653_sensor_set_mode,
    .pfn_sensor_get_mode                = gc4653_sensor_get_mode,
    .pfn_sensor_set_wdr_mode            = NULL,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = gc4653_set_bus_info,
    .pfn_sensor_write_register          = gc4653_reg_write,
    .pfn_sensor_read_register           = gc4653_reg_read,

    /* default param */
    .pfn_sensor_get_default_params      = gc4653_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = gc4653_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = gc4653_set_sensor_params,
    .pfn_sensor_get_params              = gc4653_get_sensor_params,
    .pfn_sensor_get_gain_table          = gc4653_get_sensor_gain_table,
    .pfn_sensor_set_again               = gc4653_set_again,
    .pfn_sensor_set_dgain               = gc4653_set_dgain,
    .pfn_sensor_set_integration_time    = gc4653_set_integration_time,
    .pfn_sensor_hcglcg_ctrl             = gc4653_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = gc4653_set_fps,
    .pfn_sensor_get_slow_shutter_param  = gc4653_ae_get_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = gc4653_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = NULL,
};
