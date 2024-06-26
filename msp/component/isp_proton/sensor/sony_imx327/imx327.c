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
#include "imx327_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_internal.h"
#include "isp_sensor_types.h"

/* default param */
#include "imx327_sdr.h"

/****************************************************************************
 * golbal variables  and macro definition
 ****************************************************************************/
#define INCLK_TO_37P125M_MODE           (37.125)
#define BASIC_READOUT_LINES             (1109)
#define IMX327_MAX_VMAX                 (0x3FFFF)
#define IMX327_MAX_RATIO                (16.0f)
#define IMX327_MIN_RATIO                (1.0f)

AX_SNS_COMMBUS_T g_imx327BusInfo[DEF_VIN_PIPE_MAX_NUM] = { {0} };

SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM] = {AX_NULL};

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])
#define SENSOR_SET_CTX(dev, pstCtx) (sns_ctx[dev] = pstCtx)
#define SENSOR_RESET_CTX(dev) (sns_ctx[dev] = AX_NULL)

/*user config*/
static AX_F32 gFpsGear[] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00, 11.00, 12.00, 13.00, 14.00, 15.00,
                        16.00, 17.00, 18.00, 19.00, 20.00, 21.00, 22.00, 23.00, 24.00, 25.00, 26.00, 27.00, 28.00, 29.00, 30.00};

AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

AX_S32 imx327_set_aecparam(ISP_PIPE_ID nPipeId);


/****************************************************************************
 * Internal function definition
 ****************************************************************************/
static AX_S32 sensor_ctx_init(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_S32 ret = 0;

    SNS_DBG("imx327 sensor_ctx_init. ret = %d\n", ret);
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
AX_S32 imx327_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
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

AX_S32 imx327_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
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
    sns_obj->sns_i2c_obj.slave_addr = IMX327_SLAVE_ADDR;
    sns_obj->sns_i2c_obj.address_byte = IMX327_ADDR_BYTE;
    sns_obj->sns_i2c_obj.data_byte = IMX327_DATA_BYTE;
    sns_obj->sns_i2c_obj.swap_byte = IMX327_SWAP_BYTES;


    sns_obj->sns_i2c_obj.sns_i2c_bnum = g_imx327BusInfo[nPipeId].I2cDev;

    sns_obj->sns_i2c_obj.sns_i2c_fd = i2c_init(sns_obj->sns_i2c_obj.sns_i2c_bnum,
                                      sns_obj->sns_i2c_obj.slave_addr);

#if 0
    ret = imx327_get_chipid(nPipeId, &snsId);
    if (ret < 0) {
        SNS_ERR("can't find imx327 sensor id.\r\n");
        return ret;
    } else {
        SNS_DBG("Sensor: imx327 check chip id success.\r\n");
    }

#endif

    SNS_DBG("imx327 i2c init finish, i2c bus %d \n", sns_obj->sns_i2c_obj.sns_i2c_bnum);

    return SNS_SUCCESS;
}


static void imx327_init(ISP_PIPE_ID nPipeId)
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
    imx327_write_settings(nPipeId, nImagemode);

    /* 4. refresh ae param */
    imx327_set_aecparam(nPipeId);

    /* 5. refresh ae regs table */
    imx327_sns_refresh_all_regs_from_tbl(nPipeId);
    sns_obj->bSyncInit = AX_FALSE;

    sns_obj->sns_mode_obj.nVts = imx327_get_vmax(nPipeId);
    return;
}


static void imx327_exit(ISP_PIPE_ID nPipeId)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    if (nPipeId < 0 || (nPipeId >= DEF_VIN_PIPE_MAX_NUM)) {
        return;
    }

    SENSOR_GET_CTX(nPipeId, sns_obj);
    if (AX_NULL == sns_obj) {
        SNS_ERR("imx327_exit sns_obj null\n");
        return;
    }

    i2c_exit(sns_obj->sns_i2c_obj.sns_i2c_fd);

    sensor_ctx_exit(nPipeId);
    return;
}


static AX_S32 imx327_set_bus_info(ISP_PIPE_ID nPipeId, AX_SNS_COMMBUS_T tSnsBusInfo)
{
    g_imx327BusInfo[nPipeId].I2cDev = tSnsBusInfo.I2cDev;

    return SNS_SUCCESS;
}

static AX_S32 imx327_sensor_reset(ISP_PIPE_ID nPipeId)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    /* sensor reset : Users need to modify this part of the code according to their own hardware conditions */
    imx327_reset(nPipeId, g_imx327BusInfo[nPipeId].I2cDev);

    return SNS_SUCCESS;
}

static AX_S32 imx327_get_chipid(ISP_PIPE_ID nPipeId, AX_S32 *pSnsId)
{
    AX_U16 sensor_id = 0;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    sensor_id |= imx327_reg_read(nPipeId, 0x3107) << 8;
    sensor_id |= imx327_reg_read(nPipeId, 0x3108);

    SNS_DBG("%s: sensor imx327 id: 0x%x\n", __func__, sensor_id);

    if (sensor_id != IMX327_SENSOR_CHIP_ID) {
        SNS_ERR("%s: Failed to read sensor imx327 id\n", __func__);
        return -1;
    }
    return 0;
}

static AX_S32 imx327_get_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(pSnsParam, &sns_obj->sns_param, sizeof(AX_SNS_PARAMS_T));

    return SNS_SUCCESS;
}

static AX_S32 imx327_set_sensor_params(ISP_PIPE_ID nPipeId, AX_SNS_PARAMS_T *pSnsParam)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(pSnsParam);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    memcpy(&sns_obj->sns_param, pSnsParam, sizeof(AX_SNS_PARAMS_T));
    return SNS_SUCCESS;
}


static AX_S32 imx327_get_sensor_caps(ISP_PIPE_ID nPipeId, AX_SNS_CAP_T *params)
{
    return SNS_SUCCESS;
}

static AX_S32 imx327_set_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *sns_mode)
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
    if (AX_SNS_LINEAR_MODE != sns_mode->eSnsMode) {
        hdrmode = AX_SNS_LINEAR_MODE;
    } else {
        hdrmode = sns_mode->eSnsMode;
    }

    if (width == 1920 && height == 1080 && hdrmode == AX_SNS_LINEAR_MODE && fps == 25) {
        sns_seting_index = e_imx327_4line_1920x1080_linear_rggb_12bit_25fps;
    } else if (width == 1920 && height == 1080 && hdrmode == AX_SNS_LINEAR_MODE && fps == 30) {
        sns_seting_index = e_imx327_4line_1920x1080_linear_rggb_12bit_30fps;
    } else {
        SNS_ERR("it's not supported. [%dx%d mode=%d] \n", width, height, hdrmode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    SNS_DBG("sns_seting_index:%d, mode:%d_%d_%d_%d\n", sns_seting_index, hdrmode, width, height, fps);
    sns_obj->eImgMode = sns_seting_index;
    sns_obj->sns_mode_obj.eHDRMode = hdrmode;
    sns_obj->sns_mode_obj.nWidth = width;
    sns_obj->sns_mode_obj.nHeight = height;
    sns_obj->sns_mode_obj.nFrameRate = fps;
    memcpy(&sns_obj->sns_param.sns_dev_attr, sns_mode, sizeof(AX_SNS_ATTR_T));

    return 0;
}

static AX_S32 imx327_get_mode(ISP_PIPE_ID nPipeId, AX_SNS_ATTR_T *pSnsMode)
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

static AX_S32 imx327_set_wdr_mode(ISP_PIPE_ID nPipeId, AX_SNS_HDR_MODE_E eHdrMode)
{
    return SNS_SUCCESS;
}
static AX_S32 imx327_stream_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    SNS_DBG("sensor pipe:%d stream on:%d\n", nPipeId, on);

    if (1 == on) {
        result |= imx327_reg_write(nPipeId, IMX327_XMSTA, 0x00);
        result |= imx327_reg_write(nPipeId, IMX327_STANDBY, 0x00);
    } else {
        result |= imx327_reg_write(nPipeId, IMX327_STANDBY, 0x01);
        result |= imx327_reg_write(nPipeId, IMX327_XMSTA, 0x01);
    }
    if (0 != result) {
        return -1;
    }
    /*sleep time: 1/framerate (s), add 50% margin, convert to us*/
    usleep(1500000 * (1 / AXSNS_DIV_0_TO_1_FLOAT((AX_F32)(sns_obj->sns_param.sns_ae_param.nCurFps))));

    return 0;
}

static AX_S32 imx327_testpattern_ctrl(ISP_PIPE_ID nPipeId, AX_U32 on)
{
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("sensor pipe:%d test pattern enable:%d\n", nPipeId, on);

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
static AX_S32 imx327_get_isp_default_params(ISP_PIPE_ID nPipeId, AX_SENSOR_DEFAULT_PARAM_T *ptDftParam)
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

    }

    return SNS_SUCCESS;
}

static AX_S32 imx327_get_isp_black_level(ISP_PIPE_ID nPipeId, AX_SNS_BLACK_LEVEL_T *ptBlackLevel)
{
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptBlackLevel);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    /* black level */
        ptBlackLevel->nBlackLevel[0] = 960;    /* linear mode 10bit sensor default blc 240(U12.0) --> 960(U8.6) */
        ptBlackLevel->nBlackLevel[1] = 960;
        ptBlackLevel->nBlackLevel[2] = 960;
        ptBlackLevel->nBlackLevel[3] = 960;

    return SNS_SUCCESS;
}


/****************************************************************************
 * exposure control function
 ****************************************************************************/

static AX_F32 imx327_get_line_period_f(AX_SNS_HDR_MODE_E eHDRMode, AX_U32 nFps)
{
    AX_F32 line_period_f = 0;

    if (IS_LINEAR_MODE(eHDRMode)) {
        if (nFps == 30) {
            line_period_f = 29.6;
        } else if (nFps == 25) {
            line_period_f = 35.6;
        } else {
            SNS_ERR("sdr fps = %d, not support!\n", nFps);
            return (-1.0);
        }
    } else {
        SNS_ERR("eHDRMode = %d, not support!\n", eHDRMode);
        return (-1.0);
    }

    return line_period_f;
}

static AX_U32 imx327_get_line_period(AX_SNS_HDR_MODE_E eHDRMode, AX_U32 nFps)
{
    AX_F32 line_period_f = 0;
    AX_U32 line_period = 0;

    line_period_f = imx327_get_line_period_f(eHDRMode, nFps);
    line_period = ax_float_convert_to_int(line_period_f, 22, 10, 0); // U22.10
    SNS_DBG("line_period_f:%f, line_period:%u\n", line_period_f, line_period);

    return line_period;
}




/****************************************************************************
 * exposure control function
 ****************************************************************************/
AX_S32 imx327_set_aecparam(ISP_PIPE_ID nPipeId)
{
    AX_U32 shs1 = 0, shs2 = 0, shs_lef = 0, shs3 = 0, rhs1 = 0, rhs2 = 0, vmax = 0, hmax = 0, fps = 0;
    AX_U32 rhs1_max = 0, shs1_min = 0;
    AX_U32 line_period = 0;
    AX_U32 fsc = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    shs1 = imx327_get_shs1(nPipeId);
    shs2 = imx327_get_shs2(nPipeId);
    rhs1 = imx327_get_rhs1(nPipeId);
    vmax = imx327_get_vmax(nPipeId);
    hmax = imx327_get_hmax(nPipeId);

    SNS_DBG("get shs1:0x%x, shs2:0x%x, rhs1:0x%x, vmax:0x%x, hmax:0x%x\n", shs1, shs2, rhs1, vmax, hmax);

    /* debug: The user views the sensor datasheets to complete the configuration */
    fsc = vmax;


    line_period = imx327_get_line_period(sns_obj->sns_mode_obj.eHDRMode, sns_obj->sns_mode_obj.nFrameRate);
    sns_obj->sns_param.sns_ae_param.nGainMode = AX_ADGAIN_COMBINED;
    sns_obj->sns_param.sns_ae_param.nSnsHcgLcgRatio = 2.6f;

    /* sensor again  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX] = pow(10, 0);
    sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX] = pow(10, (float)71.4 / 20);
    sns_obj->sns_param.sns_ae_param.nAGainIncrement[HDR_LONG_FRAME_IDX] = pow(10, (float)0.3 / 20);

    /* sensor dgain  limit*/
    sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX] = pow(10, 0);
    sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX] = pow(10, 0);
    sns_obj->sns_param.sns_ae_param.nDGainIncrement[HDR_LONG_FRAME_IDX] = pow(10, (float)0.3 / 20);

    /* sensor  total  gain limit*/
    sns_obj->sns_param.sns_ae_limit.nMingain = sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_limit.nMaxgain = sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX];
    sns_obj->sns_param.sns_ae_param.nIspDGainIncrement[HDR_LONG_FRAME_IDX] = (float)1/256; //U10.8

    sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] = 1 * line_period;
    sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (fsc - 2) * line_period;

    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_LONG_FRAME_IDX] = line_period;
        sns_obj->sns_param.sns_ae_param.nIntegrationTimeIncrement[HDR_MEDIUM_FRAME_IDX] = line_period;
    } else {
        // wrong hdr mode
    }

    sns_obj->sns_param.sns_ae_limit.nInitMaxIntegrationTime = sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] ;
    sns_obj->sns_param.sns_ae_limit.nInitMinIntegrationTime = sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX] ;

    sns_obj->sns_param.sns_ae_limit.nMinratio = IMX327_MIN_RATIO;
    sns_obj->sns_param.sns_ae_limit.nMaxratio = IMX327_MAX_RATIO;
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_MEDIUM_FRAME_IDX] = 1.0f;
    sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_MEDIUM_FRAME_IDX] = 1.0f;

    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = (fsc - shs_lef - 1) * line_period; // U22.10
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (rhs1 - shs1 - 1) * line_period; // U22.10

    sns_obj->sns_param.sns_ae_param.nCurFps = sns_obj->sns_mode_obj.nFrameRate;

    return 0;
}

static AX_S32 imx327_get_fps()
{
    //now nothing to do
    return SNS_SUCCESS;
}

/* note: integration time unit : us */
static AX_S32 imx327_set_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    AX_U32 shs1 = 0, shs2 = 0, shs_lef = 0, rhs1 = 0, rhs2 = 0, vmax = 0, hmax = 0;
    AX_F32 lef = 0;
    AX_F32 sef1 = 0;
    AX_U32 fsc = 0;
    AX_U32 line_period = 0;
    AX_S32 result = 0;
    float ratio = 0.0f;
    AX_U32 rhs1_temp = 0;
    AX_U32 nIntTimeFromUser = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptIntTimeTbl);

    SNS_DBG("Exptime:%d-%d, Hdrratio:%f-%f\n",
            ptIntTimeTbl->nIntTime[0], ptIntTimeTbl->nIntTime[1],
            ptIntTimeTbl->nHdrRatio[0], ptIntTimeTbl->nHdrRatio[1]);

    shs1 = imx327_get_shs1(nPipeId);
    shs2 = imx327_get_shs2(nPipeId);
    rhs1 = imx327_get_rhs1(nPipeId);
    vmax = imx327_get_vmax(nPipeId);
    hmax = imx327_get_hmax(nPipeId);

    fsc = vmax;


    line_period = imx327_get_line_period(sns_obj->sns_mode_obj.eHDRMode, sns_obj->sns_mode_obj.nFrameRate);

    if ((vmax == 0U) || (sns_obj->sns_mode_obj.nFrameRate == 0U) || (line_period == 0U)) {
        SNS_ERR("param(vmax:%d, nFrameRate:%d, line_period:%u) == 0 (Division by zero !!!)\n", vmax, sns_obj->sns_mode_obj.nFrameRate, line_period);
        return (-1);
    }

    ratio = ptIntTimeTbl->nHdrRatio[HDR_LONG_FRAME_IDX];
    ratio = AXSNS_CLIP3(ratio, sns_obj->sns_param.sns_ae_limit.nMinratio, sns_obj->sns_param.sns_ae_limit.nMaxratio);

    /* exposure time of LEF */
    nIntTimeFromUser = ptIntTimeTbl->nIntTime[HDR_LONG_FRAME_IDX];
    if (sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] == nIntTimeFromUser) {
        sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = nIntTimeFromUser;
        //SNS_WRN("%s: 1DOL new and current integration time  is equal \n", __func__);
    } else {

        nIntTimeFromUser = AXSNS_CLIP3(nIntTimeFromUser,
                                       sns_obj->sns_param.sns_ae_limit.nMinIntegrationTime[HDR_LONG_FRAME_IDX],
                                       sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX]);

        if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {

            shs1 = fsc - nIntTimeFromUser / line_period - 1;
            result = imx327_set_shs1(nPipeId, shs1);

        } else {
            SNS_ERR("wrong hdr mode");
        }

    }


    /* Calculate the true effective exposure ratio */

    /* Tlef = (FSC-shs1-1) * line_period */
    lef = (fsc - shs1 - 1) * line_period;

    /* Actual configured exposure time */
    sns_obj->sns_param.sns_ae_param.nCurIntegrationTime[HDR_LONG_FRAME_IDX] = lef;
    sns_obj->sztRegsInfo[0].tSnsExpInfo.szExpTime[HDR_LONG_FRAME_IDX] = lef;
    SNS_DBG("line_period:%u, line_period_f:%.2f, vmax:0x%x, hmax:0x%x, shs1:0x%x, shs2:0x%x, shs_lef:0x%x, rhs1:0x%x\n",
            line_period, ax_int_convert_to_float(line_period, 22, 10, 0), vmax, hmax, shs1, shs2, shs_lef, rhs1);
    SNS_DBG("current long exposure time:lef= %.2f\n\n", lef);

    return 0;
}

static AX_S32 imx327_get_integration_time(ISP_PIPE_ID nPipeId, AX_SNS_AE_SHUTTER_CFG_T *ptIntTimeTbl)
{
    //now nothing to do
    return 0;
}

static AX_S32 imx327_set_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    AX_U8 Gainl = 0;
    AX_F32 LocalNewGain = 0;
    AX_S32 result = 0;
    AX_F32 local_lgain = 0.0f;
    AX_F32 nGainFromUser = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptAnalogGainTbl);

    /* long gain  seting */
    nGainFromUser = ptAnalogGainTbl->nGain[HDR_LONG_FRAME_IDX];
    nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] - nGainFromUser) < EPS) {
        //SNS_DBG("%s: new and current gain is equal \n", __func__);
    } else {
        LocalNewGain = 20 * log10(nGainFromUser);
        LocalNewGain = LocalNewGain * (float)10 / 3;

        Gainl = ((AX_U8)LocalNewGain & 0xFF);

        result |= imx327_sns_update_regs_table(nPipeId, IMX327_GAIN, Gainl);
        if (result != 0) {
            SNS_ERR("%s: update long again fail, result:%d\n", __func__, result);
            return result;
        }

        sns_obj->sns_param.sns_ae_param.nCurAGain[HDR_LONG_FRAME_IDX] = nGainFromUser;
    }

    return SNS_SUCCESS;
}

static AX_S32 imx327_get_again(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptAnalogGainTbl)
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 imx327_set_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    /* Current version does not support */
#if 0
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    AX_F32 nGainFromUser = 0;
    float LocalNewGain = 0;
    AX_U8 Gainl;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);
    SNS_CHECK_PTR_VALID(ptDigitalGainTbl);


    nGainFromUser = ptDigitalGainTbl->nGain[HDR_LONG_FRAME_IDX];
    nGainFromUser = AXSNS_CLIP3(nGainFromUser, sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX],
                                sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX]);

    if (fabs(sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] - nGainFromUser) < EPS) {
        //SNS_DBG("%s: new and current gain is equal \n", __func__);
    } else {
        LocalNewGain = 20 * log10(nGainFromUser);
        LocalNewGain = LocalNewGain * (float)10 / 3;

        Gainl = ((AX_U8)LocalNewGain & 0xFF);
                SNS_ERR("digital Gainl: %d\n", Gainl);

        result = imx327_sns_update_regs_table(nPipeId, IMX327_GAIN, Gainl);
        if (result != 0) {
            SNS_ERR("%s: write hw failed %d\n", __func__, result);
            return result;
        }
        sns_obj->sns_param.sns_ae_param.nCurDGain[HDR_LONG_FRAME_IDX] = nGainFromUser;
    }
#endif

    return SNS_SUCCESS;
}

static AX_S32 imx327_get_dgain(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_CFG_T *ptDigitalGainTbl)
{
    //now nothing to do
    return SNS_SUCCESS;
}

static AX_S32 imx327_get_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i = 0;
    AX_S32 ret = 0;
    int d_max = 0;
    int d_min = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);

    // value <=> dB:  dB = 20* log10(value),    value = 10 ^ (dB/20)
    // dB <=> d:      d = dB * (10/3) ,         dB = d * (3/10)
    // value <=>d :   d = (200/3) log10(value), value = 10 ^(d * (3/200))
    d_max = log10(sns_obj->sns_param.sns_ae_limit.nMaxAgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3;
    d_min = log10(sns_obj->sns_param.sns_ae_limit.nMinAgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3;
    params->nAgainTableSize = d_max - d_min + 1;
    for (int i = d_min; i < (d_max + 1); i++) {
        nAgainTable[i] = pow(10, i * (float)3 / 200);
        params->pAgainTable = nAgainTable;
    }

    d_max = round(log10(sns_obj->sns_param.sns_ae_limit.nMaxDgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3);
    d_min = round(log10(sns_obj->sns_param.sns_ae_limit.nMinDgain[HDR_LONG_FRAME_IDX]) * (float)200 / 3);
    params->nDgainTableSize = d_max - d_min + 1;
    for (int i = d_min; i < (d_max + 1); i++) {
        nDgainTable[i] = pow(10, i * (float)3 / 200);
        params->pDgainTable = nDgainTable;
    }

    return ret;
}

static AX_S32 imx327_get_sensor_gain_table(ISP_PIPE_ID nPipeId, AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_S32 result = 0;
    SNS_CHECK_PTR_VALID(params);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    result = imx327_get_gain_table(nPipeId, params);

    return result;
}

AX_S32 imx327_hcglcg_ctrl(ISP_PIPE_ID nPipeId, AX_U32 nSnsHcgLcg)
{
    AX_U8 hcglcg_value = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_HCG_MODE) && (nSnsHcgLcg == AX_LCG_MODE)) {
        SNS_DBG("switch to LCG mode \n");
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_LCG_MODE;
        hcglcg_value = imx327_reg_read(nPipeId, IMX327_FDG_SEL);
        hcglcg_value = hcglcg_value & 0xEF; // l/s/vs hcg/lcg switch

        imx327_sns_update_regs_table(nPipeId, IMX327_FDG_SEL, hcglcg_value);

    } else if ((sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg == AX_LCG_MODE) && (nSnsHcgLcg == AX_HCG_MODE)) {
        SNS_DBG("switch to HCG mode \n");
        sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg = AX_HCG_MODE;
        hcglcg_value = imx327_reg_read(nPipeId, IMX327_FDG_SEL);
        hcglcg_value = hcglcg_value | 0x10; // l/s/vs hcg/lcg switch

        imx327_sns_update_regs_table(nPipeId, IMX327_FDG_SEL, hcglcg_value);

    } else {
        /*SNS_DBG("current hcg/lcg mode : %d, Mode of ae library : %d \n", nSnsHcgLcg,
                sns_obj->sns_param.sns_dev_attr.eSnsHcgLcg);*/
    }

    return SNS_SUCCESS;
}

static AX_S32 imx327_set_fps(ISP_PIPE_ID nPipeId, AX_F32 nFps, AX_SNS_PARAMS_T *sns_param)
{
    AX_U32 vmax = 0;
    AX_U32 rhs1_max = 0, shs1_min = 0;
    AX_F32 line_period_f = 0.0;
    AX_U32 line_period = 0;
    AX_U32 fsc = 0;

    AX_S32 result = 0;
    AX_S32 framerate = 30;
    AX_U32 Vmax=0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    line_period_f = imx327_get_line_period_f(sns_obj->sns_mode_obj.eHDRMode, sns_obj->sns_mode_obj.nFrameRate);
    line_period   = imx327_get_line_period(sns_obj->sns_mode_obj.eHDRMode, sns_obj->sns_mode_obj.nFrameRate);

    framerate = sns_obj->sns_mode_obj.nFrameRate;
    Vmax = sns_obj->sns_mode_obj.nVts;
    if (nFps >= framerate) {
        vmax = Vmax ;
    } else {
        vmax = 1 * SNS_1_SECOND_UNIT_US / (line_period_f * nFps);
    }

    if (vmax > IMX327_MAX_VMAX){
        vmax = IMX327_MAX_VMAX;
        nFps = 1 * SNS_1_SECOND_UNIT_US / (line_period_f * vmax);
        SNS_ERR("Beyond minmum fps  %f\n",nFps);
    }
    result = imx327_set_vmax(nPipeId, vmax);

    if (AX_SNS_HDR_2X_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        fsc = 2 * vmax;
        rhs1_max = fsc - 2 * BASIC_READOUT_LINES - 21;
        shs1_min = 2;
    } else if (AX_SNS_LINEAR_MODE == sns_obj->sns_mode_obj.eHDRMode) {
        fsc = vmax;
    } else {
        SNS_ERR("not support hdr mode:%d \n", sns_obj->sns_mode_obj.eHDRMode);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_LONG_FRAME_IDX] = (fsc - 2) * line_period;
    } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
        sns_obj->sns_param.sns_ae_limit.nMaxIntegrationTime[HDR_MEDIUM_FRAME_IDX] = (rhs1_max - shs1_min - 1) * line_period;
    }
    sns_obj->sns_param.sns_ae_param.nCurFps = 1 * SNS_1_SECOND_UNIT_US / (line_period_f * vmax);
    return SNS_SUCCESS;
}

AX_S32 imx327_ae_get_sensor_slow_shutter_param(ISP_PIPE_ID nPipeId, AX_SNS_AE_SLOW_SHUTTER_PARAM_T *ptSlowShutterParam)
{
    AX_S32 framerate = 30;
    AX_U32 nfps = 0;
    AX_U32 nVts = 0;
    AX_U32 vmax = 0;
    AX_U32 rhs1_max = 0, shs1_min = 0;
    AX_F32 line_period_f = 0.0;
    AX_U32 line_period = 0;
    AX_U32 fsc = 0;

    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);
    SNS_CHECK_PTR_VALID(sns_obj);

    framerate = sns_obj->sns_mode_obj.nFrameRate;
    if (SNS_MAX_FRAME_RATE < framerate) {
        SNS_ERR("framerate out of range : %d\n", framerate);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    line_period_f = imx327_get_line_period_f(sns_obj->sns_mode_obj.eHDRMode, sns_obj->sns_mode_obj.nFrameRate);
    line_period   = imx327_get_line_period(sns_obj->sns_mode_obj.eHDRMode, sns_obj->sns_mode_obj.nFrameRate);
    if(ax_sns_is_zero(line_period)) {
        SNS_ERR("line_period is zero : %d\n", line_period);
        return SNS_ERR_CODE_ILLEGAL_PARAMS;
    }

    ptSlowShutterParam->nGroupNum = AXSNS_MIN((sizeof(gFpsGear) / sizeof(AX_F32)), framerate);
    ax_sns_quick_sort_float(gFpsGear, ptSlowShutterParam->nGroupNum);
    ptSlowShutterParam->fMinFps =AXSNS_MAX(gFpsGear[0], ((AX_F32)1 * SNS_1_SECOND_UNIT_US / (line_period_f * IMX327_MAX_VMAX)));

    for (nfps = 0 ; nfps < ptSlowShutterParam->nGroupNum; nfps++) {
        nVts = 1 * SNS_1_SECOND_UNIT_US / (line_period_f * gFpsGear[nfps]);
        if((AX_S32)gFpsGear[nfps] >= framerate) {
            nVts = sns_obj->sns_mode_obj.nVts;
        }
        if (nVts > IMX327_MAX_VMAX) {
            nVts = IMX327_MAX_VMAX;
            SNS_WRN("Beyond minmum fps  %f\n", ptSlowShutterParam->fMinFps);
        }

        if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_LINEAR_MODE) {
            fsc = nVts;
            ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime= (fsc - 2) * line_period;
        } else if (sns_obj->sns_mode_obj.eHDRMode == AX_SNS_HDR_2X_MODE) {
            fsc = 2 * nVts;
            rhs1_max = fsc - 2 * BASIC_READOUT_LINES - 21;
            shs1_min = 2;
            ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime  = (rhs1_max - shs1_min - 1) * line_period;
        }

        ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps = 1 * SNS_1_SECOND_UNIT_US / (line_period_f * nVts);
        ptSlowShutterParam->fMaxFps  =  ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps;

        SNS_DBG("nPipeId = %d, line_period_f = %f, fps = %.2f, nMaxIntTime = %d, vts=0x%x\n",
                nPipeId, line_period_f, ptSlowShutterParam->tSlowShutterTbl[nfps].fRealFps,
                ptSlowShutterParam->tSlowShutterTbl[nfps].nMaxIntTime, nVts);
    }
    return SNS_SUCCESS;
}

AX_SENSOR_REGISTER_FUNC_T gSnsimx327Obj = {
    /* sensor ctrl */
    .pfn_sensor_reset                   = imx327_sensor_reset,
    .pfn_sensor_chipid                  = imx327_get_chipid,
    .pfn_sensor_init                    = imx327_init,
    .pfn_sensor_exit                    = imx327_exit,
    .pfn_sensor_streaming_ctrl          = imx327_stream_ctrl,
    .pfn_sensor_testpattern             = imx327_testpattern_ctrl,

    .pfn_sensor_set_mode                = imx327_set_mode,
    .pfn_sensor_get_mode                = imx327_get_mode,

    /* communication : register read/write */
    .pfn_sensor_set_bus_info            = imx327_set_bus_info,
    .pfn_sensor_write_register          = imx327_reg_write,
    .pfn_sensor_read_register           = imx327_reg_read,

    /* default param */
    .pfn_sensor_get_default_params      = imx327_get_isp_default_params,
    .pfn_sensor_get_isp_black_level     = imx327_get_isp_black_level,

    /* ae ctrl */
    .pfn_sensor_set_params              = imx327_set_sensor_params,
    .pfn_sensor_get_params              = imx327_get_sensor_params,
    .pfn_sensor_get_gain_table          = imx327_get_sensor_gain_table,
    .pfn_sensor_set_again               = imx327_set_again,
    .pfn_sensor_set_dgain               = imx327_set_dgain,
    .pfn_sensor_set_integration_time    = imx327_set_integration_time,
    .pfn_sensor_get_integration_time    = imx327_get_integration_time,
    .pfn_sensor_hcglcg_ctrl             = imx327_hcglcg_ctrl,
    .pfn_sensor_set_fps                 = imx327_set_fps,
    .pfn_sensor_get_slow_shutter_param	= imx327_ae_get_sensor_slow_shutter_param,
    .pfn_sensor_get_sns_reg_info        = imx327_ae_get_sensor_reg_info,
    .pfn_sensor_get_temperature_info    = NULL,
};

