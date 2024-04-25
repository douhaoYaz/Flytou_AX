static AX_ISP_VERSION_T ax_isp_version_param_sdr = {
    /* nIspMajor */
    0,
    /* nIspMinor1 */
    1,
    /* nIspMinor2 */
    0,
    /* szBuildTime[32] */
    "",
    /* szIspVersion[64] */
    "AX620_ISP_V0.1.0",
};

static AX_ISP_IQ_EIS_PARAM_T eis_param_sdr = {
    /* bEisEnable */
    1,
    /* nEisDelayNum */
    2,
    /* nCropRatioH */
    25,
    /* nCropRatioW */
    25,
    /* tEisNpuParam */
    {
        /* szWbtModelName[256] */
        "/opt/etc/IMX327_SDR_EIS_1920x1080_12b_LCG_A1-16X_LV0_V112_0002911671_220602_AX620A.pro",
        /* szModelName[256] */
        "IMX327_SDR_EIS_1920x1080_12b_LCG_A1-16X_LV0_V112_0002911671_220602_AX620A",
    },
};