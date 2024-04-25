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

static AX_ISP_IQ_NPU_PARAM_T npu_param_sdr = {
    /* bNrEnable */
    0,
    /* bAutoMode */
    1,
    /* bUpdateTable */
    1,
    /* nHdrMode */
    1,
    /* nRefMode */
    1,
    /* tDummyParam */
    {
        /* nDummyModelNum */
        1,
        /* tDummyModelTable[32] */
        {
            /* 0 */
            {
                /* szWbtModelName[256] */
                "/opt/etc/SC530AI_DUMMY_WNR-2D3D_2688x1616_10b_V112_0002742592_220425_AX620A.pro",
                /* nHcgMode */
                0,
                /* nIsoThresholdMin */
                100,
                /* nIsoThresholdMax */
                8100,
                /* szModelName[256] */
                "SC530AI_DUMMY_WNR-2D3D_2688x1616_10b_V112_0002742592_220425_AX620A",
                /* n2DLevel */
                0,
                /* n3DLevel */
                0,
                /* nRefGrpNum */
                5,
                /* nRefValue */
                {100, 200, 300, 400, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nBiasIn */
                0,
                /* nBiasOut */
                0,
            },
        },
    },
    /* tManualParam */
    {
        /* szWbtModelName[256] */
        "/opt/etc/SC530AI_SDR_WNR-2D_2880x1616_10b_A1-81X_V112_5WBT_0000000000_220606_AX620A.pro",
        /* szModelName[256] */
        "SC530AI_SDR_WNR-2D_2880x1616_10b_LCG_A1-10X_LV0_V112_0002919561_220606_AX620A",
        /* n2DLevel */
        0,
        /* n3DLevel */
        0,
        /* nSpatialMR */
        128,
        /* nSpatialBR */
        128,
        /* nTemporalMR */
        128,
        /* nTemporalBR */
        128,
        /* nBiasIn */
        0,
        /* nBiasOut */
        0,
    },
    /* tAutoParam */
    {
        /* nAutoModelNum */
        5,
        /* tAutoModelTable[32] */
        {
            /* 0 */
            {
                /* szWbtModelName[256] */
                "/opt/etc/SC530AI_SDR_WNR-2D_2880x1616_10b_A1-81X_V112_5WBT_0000000000_220606_AX620A.pro",
                /* nHcgMode */
                0,
                /* nIsoThresholdMin */
                100,
                /* nIsoThresholdMax */
                1000,
                /* szModelName[256] */
                "SC530AI_SDR_WNR-2D_2880x1616_10b_LCG_A1-10X_LV0_V112_0002919561_220606_AX620A",
                /* n2DLevel */
                0,
                /* n3DLevel */
                0,
                /* nRefGrpNum */
                5,
                /* nRefValue */
                {100, 200, 300, 400, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nBiasIn */
                0,
                /* nBiasOut */
                0,
            },
            /* 1 */
            {
                /* szWbtModelName[256] */
                "/opt/etc/SC530AI_SDR_WNR-2D_2880x1616_10b_A1-81X_V112_5WBT_0000000000_220606_AX620A.pro",
                /* nHcgMode */
                0,
                /* nIsoThresholdMin */
                1000,
                /* nIsoThresholdMax */
                2000,
                /* szModelName[256] */
                "SC530AI_SDR_WNR-2D_2880x1616_10b_LCG_A10-20X_LV0_V112_0002919562_220606_AX620A",
                /* n2DLevel */
                0,
                /* n3DLevel */
                0,
                /* nRefGrpNum */
                5,
                /* nRefValue */
                {100, 200, 300, 400, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nBiasIn */
                0,
                /* nBiasOut */
                0,
            },
            /* 2 */
            {
                /* szWbtModelName[256] */
                "/opt/etc/SC530AI_SDR_WNR-2D_2880x1616_10b_A1-81X_V112_5WBT_0000000000_220606_AX620A.pro",
                /* nHcgMode */
                0,
                /* nIsoThresholdMin */
                2000,
                /* nIsoThresholdMax */
                4000,
                /* szModelName[256] */
                "SC530AI_SDR_WNR-2D_2880x1616_10b_LCG_A20-40X_LV0_V112_0002919563_220606_AX620A",
                /* n2DLevel */
                0,
                /* n3DLevel */
                0,
                /* nRefGrpNum */
                5,
                /* nRefValue */
                {100, 200, 300, 400, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nBiasIn */
                0,
                /* nBiasOut */
                0,
            },
            /* 3 */
            {
                /* szWbtModelName[256] */
                "/opt/etc/SC530AI_SDR_WNR-2D_2880x1616_10b_A1-81X_V112_5WBT_0000000000_220606_AX620A.pro",
                /* nHcgMode */
                0,
                /* nIsoThresholdMin */
                4000,
                /* nIsoThresholdMax */
                8100,
                /* szModelName[256] */
                "SC530AI_SDR_WNR-2D_2880x1616_10b_LCG_A40-80X_LV0_V112_0002919564_220606_AX620A",
                /* n2DLevel */
                0,
                /* n3DLevel */
                0,
                /* nRefGrpNum */
                5,
                /* nRefValue */
                {100, 200, 300, 400, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nBiasIn */
                0,
                /* nBiasOut */
                0,
            },
            /* 4 */
            {
                /* szWbtModelName[256] */
                "/opt/etc/SC530AI_SDR_WNR-2D_2880x1616_10b_A1-81X_V112_5WBT_0000000000_220606_AX620A.pro",
                /* nHcgMode */
                0,
                /* nIsoThresholdMin */
                8100,
                /* nIsoThresholdMax */
                129600,
                /* szModelName[256] */
                "SC530AI_SDR_WNR-2D_2880x1616_10b_LCG_A81X_I1-16X_LV0_V112_0002919565_220606_AX620A",
                /* n2DLevel */
                0,
                /* n3DLevel */
                0,
                /* nRefGrpNum */
                5,
                /* nRefValue */
                {100, 200, 300, 400, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nSpatialBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalMR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nTemporalBR */
                {20, 60, 100, 160, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                /* nBiasIn */
                0,
                /* nBiasOut */
                0,
            },
        },
    },
};

static AX_ISP_IQ_WNR_PARAM_T wnr_param_sdr = {
    /* nWnrEn */
    1,
    /* nAutoMode */
    1,
    /* tWnrNoiseParam */
    {
        /* tHcgTable */
        {
            /* nShotNoiseCoeffsA[4] */
            {0, 0, 0, 0, /*0 - 3*/},
            /* nShotNoiseCoeffsB[4] */
            {0, 0, 0, 0, /*0 - 3*/},
            /* nReadNoiseCoeffsA[4] */
            {0, 0, 0, 0, /*0 - 3*/},
            /* nReadNoiseCoeffsB[4] */
            {0, 0, 0, 0, /*0 - 3*/},
            /* nReadNoiseCoeffsC[4] */
            {0, 0, 0, 0, /*0 - 3*/},
        },
        /* tLcgTable */
        {
            /* nShotNoiseCoeffsA[4] */
            {41231012, 42527013, 42042193, 38885812, /*0 - 3*/},
            /* nShotNoiseCoeffsB[4] */
            {41685170, 37924749, 31142883, 44257278, /*0 - 3*/},
            /* nReadNoiseCoeffsA[4] */
            {594272, 573142, 561033, 582470, /*0 - 3*/},
            /* nReadNoiseCoeffsB[4] */
            {29108740, 19515856, 19114893, 28986532, /*0 - 3*/},
            /* nReadNoiseCoeffsC[4] */
            {-22060947, 28141648, 29664016, -21113803, /*0 - 3*/},
        },
    },
    /* tManualParam */
    {
        /* nWnrMode */
        2,
        /* tWnrParam */
        {
            /* nRatio[4] */
            {256, 256, 256, 256, /*0 - 3*/},
            /* nShrinkageStrength */
            10,
            /* nShrinkageWeight[16][16] */
            {
                {83, 83, 91, 91, 102, 102, 102, 102, 103, 103, 103, 103, 103, 103, 103, 103, /*0 - 15*/},
                {83, 83, 91, 91, 102, 102, 102, 102, 103, 103, 103, 103, 103, 103, 103, 103, /*0 - 15*/},
                {91, 91, 102, 102, 103, 103, 103, 103, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {91, 91, 102, 102, 103, 103, 103, 103, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {102, 102, 103, 103, 115, 115, 115, 115, 118, 118, 118, 118, 118, 118, 118, 118, /*0 - 15*/},
                {102, 102, 103, 103, 115, 115, 115, 115, 118, 118, 118, 118, 118, 118, 118, 118, /*0 - 15*/},
                {102, 102, 103, 103, 115, 115, 115, 115, 118, 118, 118, 118, 118, 118, 118, 118, /*0 - 15*/},
                {102, 102, 103, 103, 115, 115, 115, 115, 118, 118, 118, 118, 118, 118, 118, 118, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
                {103, 103, 115, 115, 118, 118, 118, 118, 128, 128, 128, 128, 128, 128, 128, 128, /*0 - 15*/},
            },
            /* nBlendStrength */
            200,
            /* nBlendWeight[16][16] */
            {
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
            },
            /* nDeghostStrength */
            256,
            /* nStrengthLut[128] */
            {
                4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 3986, 3877, 3767, 3657, 3547, 3438,  /* 0 - 31*/
                3328, 3218, 3109, 2999, 2889, 2779, 2670, 2560, 2450, 2341, 2231, 2121, 2011, 1902, 1792, 1682, 1573, 1463, 1353, 1243, 1134, 1024, 914, 805, 695, 585, 475, 366, 256, 256, 256, 256,  /* 32 - 63*/
                256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256,  /* 64 - 95*/
                256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256,  /* 96 - 127*/
            },
        },
    },
    /* tAutoParam */
    {
        /* nGrpNum */
        1,
        /* tSubAutoParam[16] */
        {
            /* 0 */
            {
                /* nSubGrpNum */
                1,
                /* nWnrMode */
                2,
                /* nIsoThresholdValue[9] */
                {100, /*0 - 0*/},
                /* tWnrParam[9] */
                {
                    /* 0 */
                    {
                        /* nRatio[4] */
                        {256, 256, 256, 256, /*0 - 3*/},
                        /* nShrinkageStrength */
                        4,
                        /* nShrinkageWeight[16][16] */
                        {
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                            {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, /*0 - 15*/},
                        },
                        /* nBlendStrength */
                        200,
                        /* nBlendWeight[16][16] */
                        {
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                            {115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, /*0 - 15*/},
                        },
                        /* nDeghostStrength */
                        256,
                        /* nStrengthLut[128] */
                        {
                            4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 3986, 3877, 3767, 3657, 3547, 3438,  /* 0 - 31*/
                            3328, 3218, 3109, 2999, 2889, 2779, 2670, 2560, 2450, 2341, 2231, 2121, 2011, 1902, 1792, 1682, 1573, 1463, 1353, 1243, 1134, 1024, 914, 805, 695, 585, 475, 366, 256, 256, 256, 256,  /* 32 - 63*/
                            256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256,  /* 64 - 95*/
                            256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256, 256,  /* 96 - 127*/
                        },
                    },
                },
            },
        },
    },
};