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

static AX_ISP_IQ_PFR_PARAM_T pfr_param_sdr = {
    /* nPfrEn */
    1,
    /* nRefMode */
    0,
    /* nProcessMode */
    0,
    /* nDepurpleStrength */
    128,
    /* nLumaRatio[3] */
    {32, 64, 32, /*0 - 2*/},
    /* nLumaLut[8] */
    {0, 0, 128, 128, 128, 64, 0, 0, /*0 - 7*/},
    /* nAngleRatioLut[16] */
    {128, 128, 128, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 96, 128, 128, /*0 - 15*/},
    /* nCcmMatrix[3][3] */
    {
        {332, 0, -77, /*0 - 2*/},
        {-140, 414, -17, /*0 - 2*/},
        {14, -274, 516, /*0 - 2*/},
    },
    /* tManualParam */
    {
        /* nLuxConfidenceLevel */
        128,
        /* nCctConfidenceLevel */
        128,
        /* nMaskMatrix[3][3] */
        {
            {64, 128, 64, /*0 - 2*/},
            {64, 128, 64, /*0 - 2*/},
            {64, 128, 64, /*0 - 2*/},
        },
    },
    /* tAutoParam */
    {
        /* nLuxNum */
        2,
        /* nCctNum */
        2,
        /* nRefLuxOrGain[16] */
        {1024, 2048, /*0 - 1*/},
        /* nRefCct[16] */
        {3000, 5000, /*0 - 1*/},
        /* nLuxConfidenceLevel[16] */
        {128, 128, /*0 - 1*/},
        /* nCctConfidenceLevel[16] */
        {128, 128, /*0 - 1*/},
    },
    /* tNpuPfrParam */
    {
        /* szWbtModelName[256] */
        "/opt/etc/OS08A10_SDR_PFD_3840x2160_10b_LCG_A1-16X_LV0_V112_202_0002868202_220526_AX620A.pro",
        /* szModelName[256] */
        "OS08A10_SDR_PFD_3840x2160_10b_LCG_A1-16X_LV0_V112_202_0002868202_220526_AX620A",
    },
};