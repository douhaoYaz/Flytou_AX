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
        "/opt/etc/SC530AI_SDR_EIS_2880x1616_10b_LCG_A1-16X_LV0_V112_0002919277_220606_AX620A.pro",
        /* szModelName[256] */
        "SC530AI_SDR_EIS_2880x1616_10b_LCG_A1-16X_LV0_V112_0002919277_220606_AX620A",
    },
};
