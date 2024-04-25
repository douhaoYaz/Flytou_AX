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
        "/opt/etc/OS04A10_EIS_DBL_1344Wx760H_10bit_vnpu112_220421.pro",
        /* szModelName[256] */
        "OS04A10_EIS_DBL_1344Wx760H_10bit_vnpu112_220421",
    },
};