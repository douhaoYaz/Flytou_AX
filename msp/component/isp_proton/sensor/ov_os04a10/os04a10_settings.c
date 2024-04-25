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
#include "i2c.h"
#include "ax_base_type.h"
#include "os04a10_settings.h"
#include "ax_isp_common.h"
#include "isp_sensor_types.h"
#include "isp_sensor_internal.h"

extern AX_SNS_COMMBUS_T g_Os04A10BusInfo[DEF_VIN_PIPE_MAX_NUM];
extern SNS_STATE_OBJ *sns_ctx[DEF_VIN_PIPE_MAX_NUM];

#define SENSOR_GET_CTX(dev, pstCtx) (pstCtx = sns_ctx[dev])

static AX_SNS_DRV_DELAY_TABLE_T gOs04a10AeRegsTable[] = {
    /* nRegAddr */          /*regs value*/  /*Delay Frame Num*/
    {OS04A10_LONG_EXP_LINE_H,       0,          0},
    {OS04A10_LONG_EXP_LINE_L,       0,          0},
    {OS04A10_LONG_AGAIN_H,          0,          0},
    {OS04A10_LONG_AGAIN_L,          0,          0},
    {OS04A10_LONG_DGAIN_H,          0,          0},
    {OS04A10_LONG_DGAIN_M,          0,          0},
    {OS04A10_LONG_DGAIN_L,          0,          0},

    {OS04A10_SHORT_EXP_LINE_H,      0,          0},
    {OS04A10_SHORT_EXP_LINE_L,      0,          0},
    {OS04A10_SHORT_AGAIN_H,         0,          0},
    {OS04A10_SHORT_AGAIN_L,         0,          0},
    {OS04A10_SHORT_DGAIN_H,         0,          0},
    {OS04A10_SHORT_DGAIN_M,         0,          0},
    {OS04A10_SHORT_DGAIN_L,         0,          0},

    {OS04A10_VTS_H,                 0,          0},
    {OS04A10_VTS_L,                 0,          0},

    /* group hold */
    {OS04A10_GROUP_SWCTRL1,         0,          1},
    {OS04A10_GROUP_ACCESS1,         0,          1},
    {OS04A10_HCG_LCG,               0,          1},
    {OS04A10_GROUP_ACCESS2,         0,          1},
    {OS04A10_GROUP_SWCTRL2,         0,          1},
    {OS04A10_GROUP10_STAY_NUM,      0,          1},
    {OS04A10_SW_GROUP_ACCESS,       0,          1},
};



static AX_F32 nAgainTable[SENSOR_MAX_GAIN_STEP];
static AX_F32 nDgainTable[SENSOR_MAX_GAIN_STEP];

typedef struct _OS04A10_AGAIN_TABLE_T_ {
    float gain;
    AX_U8 again_in;
    AX_U8 again_de;
} OS04A10_AGAIN_TABLE_T;

typedef struct _OS04A10_DGAIN_TABLE_T_ {
    float gain;
    AX_U8 dgain_in;
    AX_U8 dgain_de;
    AX_U8 dgain_de2;
} OS04A10_DGAIN_TABLE_T;

const OS04A10_AGAIN_TABLE_T os04a10_again_table[] = {
    {1,       0x01, 0x00},
    {1.0625,  0x01, 0x10},
    {1.125,   0x01, 0x20},
    {1.1875,  0x01, 0x30},
    {1.25,    0x01, 0x40},
    {1.3125,  0x01, 0x50},
    {1.375,   0x01, 0x60},
    {1.4375,  0x01, 0x70},
    {1.5,     0x01, 0x80},
    {1.5625,  0x01, 0x90},
    {1.625,   0x01, 0xA0},
    {1.6875,  0x01, 0xB0},
    {1.75,    0x01, 0xC0},
    {1.8125,  0x01, 0xD0},
    {1.875,   0x01, 0xE0},
    {1.9375,  0x01, 0xF0},
    {2,       0x02, 0x00},
    {2.125,   0x02, 0x20},
    {2.25,    0x02, 0x40},
    {2.375,   0x02, 0x60},
    {2.5,     0x02, 0x80},
    {2.625,   0x02, 0xA0},
    {2.75,    0x02, 0xC0},
    {2.875,   0x02, 0xE0},
    {3,       0x03, 0x00},
    {3.125,   0x03, 0x20},
    {3.25,    0x03, 0x40},
    {3.375,   0x03, 0x60},
    {3.5,     0x03, 0x80},
    {3.625,   0x03, 0xA0},
    {3.75,    0x03, 0xC0},
    {3.875,   0x03, 0xE0},
    {4,       0x04, 0x00},
    {4.25,    0x04, 0x40},
    {4.5,     0x04, 0x80},
    {4.75,    0x04, 0xC0},
    {5,       0x05, 0x00},
    {5.25,    0x05, 0x40},
    {5.5,     0x05, 0x80},
    {5.75,    0x05, 0xC0},
    {6,       0x06, 0x00},
    {6.25,    0x06, 0x40},
    {6.5,     0x06, 0x80},
    {6.75,    0x06, 0xC0},
    {7,       0x07, 0x00},
    {7.25,    0x07, 0x40},
    {7.5,     0x07, 0x80},
    {7.75,    0x07, 0xC0},
    {8,       0x08, 0x00},
    {8.5,     0x08, 0x80},
    {9,       0x09, 0x00},
    {9.5,     0x09, 0x80},
    {10,      0x0A, 0x00},
    {10.5,    0x0A, 0x80},
    {11,      0x0B, 0x00},
    {11.5,    0x0B, 0x80},
    {12,      0x0C, 0x00},
    {12.5,    0x0C, 0x80},
    {13,      0x0D, 0x00},
    {13.5,    0x0D, 0x80},
    {14,      0x0E, 0x00},
    {14.5,    0x0E, 0x80},
    {15,      0x0F, 0x00},
    {15.5,    0x0F, 0x80},
    {16.0,    0x0F, 0xF0},
};

const OS04A10_DGAIN_TABLE_T os04a10_dgain_table[] = {
   /* gain       dg_h  dg_l  dg_ll*/
    {1,          0x01, 0x00, 0x00},
    {1.00390625, 0x01, 0x01, 0x00},
    {1.00781250, 0x01, 0x02, 0x00},
    {1.01171875, 0x01, 0x03, 0x00},
    {1.01562500, 0x01, 0x04, 0x00},
    {1.01953125, 0x01, 0x05, 0x00},
    {1.02343750, 0x01, 0x06, 0x00},
    {1.02734375, 0x01, 0x07, 0x00},
    {1.03125000, 0x01, 0x08, 0x00},
    {1.03515625, 0x01, 0x09, 0x00},
    {1.03906250, 0x01, 0x0a, 0x00},
    {1.04296875, 0x01, 0x0b, 0x00},
    {1.04687500, 0x01, 0x0c, 0x00},
    {1.05078125, 0x01, 0x0d, 0x00},
    {1.05468750, 0x01, 0x0e, 0x00},
    {1.05859375, 0x01, 0x0f, 0x00},
    {1.06250000, 0x01, 0x10, 0x00},
    {1.06640625, 0x01, 0x11, 0x00},
    {1.07031250, 0x01, 0x12, 0x00},
    {1.07421875, 0x01, 0x13, 0x00},
    {1.07812500, 0x01, 0x14, 0x00},
    {1.08203125, 0x01, 0x15, 0x00},
    {1.08593750, 0x01, 0x16, 0x00},
    {1.08984375, 0x01, 0x17, 0x00},
    {1.09375000, 0x01, 0x18, 0x00},
    {1.09765625, 0x01, 0x19, 0x00},
    {1.10156250, 0x01, 0x1a, 0x00},
    {1.10546875, 0x01, 0x1b, 0x00},
    {1.10937500, 0x01, 0x1c, 0x00},
    {1.11328125, 0x01, 0x1d, 0x00},
    {1.11718750, 0x01, 0x1e, 0x00},
    {1.12109375, 0x01, 0x1f, 0x00},
    {1.12500000, 0x01, 0x20, 0x00},
    {1.12890625, 0x01, 0x21, 0x00},
    {1.13281250, 0x01, 0x22, 0x00},
    {1.13671875, 0x01, 0x23, 0x00},
    {1.14062500, 0x01, 0x24, 0x00},
    {1.14453125, 0x01, 0x25, 0x00},
    {1.14843750, 0x01, 0x26, 0x00},
    {1.15234375, 0x01, 0x27, 0x00},
    {1.15625000, 0x01, 0x28, 0x00},
    {1.16015625, 0x01, 0x29, 0x00},
    {1.16406250, 0x01, 0x2a, 0x00},
    {1.16796875, 0x01, 0x2b, 0x00},
    {1.17187500, 0x01, 0x2c, 0x00},
    {1.17578125, 0x01, 0x2d, 0x00},
    {1.17968750, 0x01, 0x2e, 0x00},
    {1.18359375, 0x01, 0x2f, 0x00},
    {1.18750000, 0x01, 0x30, 0x00},
    {1.19140625, 0x01, 0x31, 0x00},
    {1.19531250, 0x01, 0x32, 0x00},
    {1.19921875, 0x01, 0x33, 0x00},
    {1.20312500, 0x01, 0x34, 0x00},
    {1.20703125, 0x01, 0x35, 0x00},
    {1.21093750, 0x01, 0x36, 0x00},
    {1.21484375, 0x01, 0x37, 0x00},
    {1.21875000, 0x01, 0x38, 0x00},
    {1.22265625, 0x01, 0x39, 0x00},
    {1.22656250, 0x01, 0x3a, 0x00},
    {1.23046875, 0x01, 0x3b, 0x00},
    {1.23437500, 0x01, 0x3c, 0x00},
    {1.23828125, 0x01, 0x3d, 0x00},
    {1.24218750, 0x01, 0x3e, 0x00},
    {1.24609375, 0x01, 0x3f, 0x00},
    {1.25000000, 0x01, 0x40, 0x00},
    {1.25390625, 0x01, 0x41, 0x00},
    {1.25781250, 0x01, 0x42, 0x00},
    {1.26171875, 0x01, 0x43, 0x00},
    {1.26562500, 0x01, 0x44, 0x00},
    {1.26953125, 0x01, 0x45, 0x00},
    {1.27343750, 0x01, 0x46, 0x00},
    {1.27734375, 0x01, 0x47, 0x00},
    {1.28125000, 0x01, 0x48, 0x00},
    {1.28515625, 0x01, 0x49, 0x00},
    {1.28906250, 0x01, 0x4a, 0x00},
    {1.29296875, 0x01, 0x4b, 0x00},
    {1.29687500, 0x01, 0x4c, 0x00},
    {1.30078125, 0x01, 0x4d, 0x00},
    {1.30468750, 0x01, 0x4e, 0x00},
    {1.30859375, 0x01, 0x4f, 0x00},
    {1.31250000, 0x01, 0x50, 0x00},
    {1.31640625, 0x01, 0x51, 0x00},
    {1.32031250, 0x01, 0x52, 0x00},
    {1.32421875, 0x01, 0x53, 0x00},
    {1.32812500, 0x01, 0x54, 0x00},
    {1.33203125, 0x01, 0x55, 0x00},
    {1.33593750, 0x01, 0x56, 0x00},
    {1.33984375, 0x01, 0x57, 0x00},
    {1.34375000, 0x01, 0x58, 0x00},
    {1.34765625, 0x01, 0x59, 0x00},
    {1.35156250, 0x01, 0x5a, 0x00},
    {1.35546875, 0x01, 0x5b, 0x00},
    {1.35937500, 0x01, 0x5c, 0x00},
    {1.36328125, 0x01, 0x5d, 0x00},
    {1.36718750, 0x01, 0x5e, 0x00},
    {1.37109375, 0x01, 0x5f, 0x00},
    {1.37500000, 0x01, 0x60, 0x00},
    {1.37890625, 0x01, 0x61, 0x00},
    {1.38281250, 0x01, 0x62, 0x00},
    {1.38671875, 0x01, 0x63, 0x00},
    {1.39062500, 0x01, 0x64, 0x00},
    {1.39453125, 0x01, 0x65, 0x00},
    {1.39843750, 0x01, 0x66, 0x00},
    {1.40234375, 0x01, 0x67, 0x00},
    {1.40625000, 0x01, 0x68, 0x00},
    {1.41015625, 0x01, 0x69, 0x00},
    {1.41406250, 0x01, 0x6a, 0x00},
    {1.41796875, 0x01, 0x6b, 0x00},
    {1.42187500, 0x01, 0x6c, 0x00},
    {1.42578125, 0x01, 0x6d, 0x00},
    {1.42968750, 0x01, 0x6e, 0x00},
    {1.43359375, 0x01, 0x6f, 0x00},
    {1.43750000, 0x01, 0x70, 0x00},
    {1.44140625, 0x01, 0x71, 0x00},
    {1.44531250, 0x01, 0x72, 0x00},
    {1.44921875, 0x01, 0x73, 0x00},
    {1.45312500, 0x01, 0x74, 0x00},
    {1.45703125, 0x01, 0x75, 0x00},
    {1.46093750, 0x01, 0x76, 0x00},
    {1.46484375, 0x01, 0x77, 0x00},
    {1.46875000, 0x01, 0x78, 0x00},
    {1.47265625, 0x01, 0x79, 0x00},
    {1.47656250, 0x01, 0x7a, 0x00},
    {1.48046875, 0x01, 0x7b, 0x00},
    {1.48437500, 0x01, 0x7c, 0x00},
    {1.48828125, 0x01, 0x7d, 0x00},
    {1.49218750, 0x01, 0x7e, 0x00},
    {1.49609375, 0x01, 0x7f, 0x00},
    {1.50000000, 0x01, 0x80, 0x00},
    {1.50390625, 0x01, 0x81, 0x00},
    {1.50781250, 0x01, 0x82, 0x00},
    {1.51171875, 0x01, 0x83, 0x00},
    {1.51562500, 0x01, 0x84, 0x00},
    {1.51953125, 0x01, 0x85, 0x00},
    {1.52343750, 0x01, 0x86, 0x00},
    {1.52734375, 0x01, 0x87, 0x00},
    {1.53125000, 0x01, 0x88, 0x00},
    {1.53515625, 0x01, 0x89, 0x00},
    {1.53906250, 0x01, 0x8a, 0x00},
    {1.54296875, 0x01, 0x8b, 0x00},
    {1.54687500, 0x01, 0x8c, 0x00},
    {1.55078125, 0x01, 0x8d, 0x00},
    {1.55468750, 0x01, 0x8e, 0x00},
    {1.55859375, 0x01, 0x8f, 0x00},
    {1.56250000, 0x01, 0x90, 0x00},
    {1.56640625, 0x01, 0x91, 0x00},
    {1.57031250, 0x01, 0x92, 0x00},
    {1.57421875, 0x01, 0x93, 0x00},
    {1.57812500, 0x01, 0x94, 0x00},
    {1.58203125, 0x01, 0x95, 0x00},
    {1.58593750, 0x01, 0x96, 0x00},
    {1.58984375, 0x01, 0x97, 0x00},
    {1.59375000, 0x01, 0x98, 0x00},
    {1.59765625, 0x01, 0x99, 0x00},
    {1.60156250, 0x01, 0x9a, 0x00},
    {1.60546875, 0x01, 0x9b, 0x00},
    {1.60937500, 0x01, 0x9c, 0x00},
    {1.61328125, 0x01, 0x9d, 0x00},
    {1.61718750, 0x01, 0x9e, 0x00},
    {1.62109375, 0x01, 0x9f, 0x00},
    {1.62500000, 0x01, 0xa0, 0x00},
    {1.62890625, 0x01, 0xa1, 0x00},
    {1.63281250, 0x01, 0xa2, 0x00},
    {1.63671875, 0x01, 0xa3, 0x00},
    {1.64062500, 0x01, 0xa4, 0x00},
    {1.64453125, 0x01, 0xa5, 0x00},
    {1.64843750, 0x01, 0xa6, 0x00},
    {1.65234375, 0x01, 0xa7, 0x00},
    {1.65625000, 0x01, 0xa8, 0x00},
    {1.66015625, 0x01, 0xa9, 0x00},
    {1.66406250, 0x01, 0xaa, 0x00},
    {1.66796875, 0x01, 0xab, 0x00},
    {1.67187500, 0x01, 0xac, 0x00},
    {1.67578125, 0x01, 0xad, 0x00},
    {1.67968750, 0x01, 0xae, 0x00},
    {1.68359375, 0x01, 0xaf, 0x00},
    {1.68750000, 0x01, 0xb0, 0x00},
    {1.69140625, 0x01, 0xb1, 0x00},
    {1.69531250, 0x01, 0xb2, 0x00},
    {1.69921875, 0x01, 0xb3, 0x00},
    {1.70312500, 0x01, 0xb4, 0x00},
    {1.70703125, 0x01, 0xb5, 0x00},
    {1.71093750, 0x01, 0xb6, 0x00},
    {1.71484375, 0x01, 0xb7, 0x00},
    {1.71875000, 0x01, 0xb8, 0x00},
    {1.72265625, 0x01, 0xb9, 0x00},
    {1.72656250, 0x01, 0xba, 0x00},
    {1.73046875, 0x01, 0xbb, 0x00},
    {1.73437500, 0x01, 0xbc, 0x00},
    {1.73828125, 0x01, 0xbd, 0x00},
    {1.74218750, 0x01, 0xbe, 0x00},
    {1.74609375, 0x01, 0xbf, 0x00},
    {1.75000000, 0x01, 0xc0, 0x00},
    {1.75390625, 0x01, 0xc1, 0x00},
    {1.75781250, 0x01, 0xc2, 0x00},
    {1.76171875, 0x01, 0xc3, 0x00},
    {1.76562500, 0x01, 0xc4, 0x00},
    {1.76953125, 0x01, 0xc5, 0x00},
    {1.77343750, 0x01, 0xc6, 0x00},
    {1.77734375, 0x01, 0xc7, 0x00},
    {1.78125000, 0x01, 0xc8, 0x00},
    {1.78515625, 0x01, 0xc9, 0x00},
    {1.78906250, 0x01, 0xca, 0x00},
    {1.79296875, 0x01, 0xcb, 0x00},
    {1.79687500, 0x01, 0xcc, 0x00},
    {1.80078125, 0x01, 0xcd, 0x00},
    {1.80468750, 0x01, 0xce, 0x00},
    {1.80859375, 0x01, 0xcf, 0x00},
    {1.81250000, 0x01, 0xd0, 0x00},
    {1.81640625, 0x01, 0xd1, 0x00},
    {1.82031250, 0x01, 0xd2, 0x00},
    {1.82421875, 0x01, 0xd3, 0x00},
    {1.82812500, 0x01, 0xd4, 0x00},
    {1.83203125, 0x01, 0xd5, 0x00},
    {1.83593750, 0x01, 0xd6, 0x00},
    {1.83984375, 0x01, 0xd7, 0x00},
    {1.84375000, 0x01, 0xd8, 0x00},
    {1.84765625, 0x01, 0xd9, 0x00},
    {1.85156250, 0x01, 0xda, 0x00},
    {1.85546875, 0x01, 0xdb, 0x00},
    {1.85937500, 0x01, 0xdc, 0x00},
    {1.86328125, 0x01, 0xdd, 0x00},
    {1.86718750, 0x01, 0xde, 0x00},
    {1.87109375, 0x01, 0xdf, 0x00},
    {1.87500000, 0x01, 0xe0, 0x00},
    {1.87890625, 0x01, 0xe1, 0x00},
    {1.88281250, 0x01, 0xe2, 0x00},
    {1.88671875, 0x01, 0xe3, 0x00},
    {1.89062500, 0x01, 0xe4, 0x00},
    {1.89453125, 0x01, 0xe5, 0x00},
    {1.89843750, 0x01, 0xe6, 0x00},
    {1.90234375, 0x01, 0xe7, 0x00},
    {1.90625000, 0x01, 0xe8, 0x00},
    {1.91015625, 0x01, 0xe9, 0x00},
    {1.91406250, 0x01, 0xea, 0x00},
    {1.91796875, 0x01, 0xeb, 0x00},
    {1.92187500, 0x01, 0xec, 0x00},
    {1.92578125, 0x01, 0xed, 0x00},
    {1.92968750, 0x01, 0xee, 0x00},
    {1.93359375, 0x01, 0xef, 0x00},
    {1.93750000, 0x01, 0xf0, 0x00},
    {1.94140625, 0x01, 0xf1, 0x00},
    {1.94531250, 0x01, 0xf2, 0x00},
    {1.94921875, 0x01, 0xf3, 0x00},
    {1.95312500, 0x01, 0xf4, 0x00},
    {1.95703125, 0x01, 0xf5, 0x00},
    {1.96093750, 0x01, 0xf6, 0x00},
    {1.96484375, 0x01, 0xf7, 0x00},
    {1.96875000, 0x01, 0xf8, 0x00},
    {1.97265625, 0x01, 0xf9, 0x00},
    {1.97656250, 0x01, 0xfa, 0x00},
    {1.98046875, 0x01, 0xfb, 0x00},
    {1.98437500, 0x01, 0xfc, 0x00},
    {1.98828125, 0x01, 0xfd, 0x00},
    {1.99218750, 0x01, 0xfe, 0x00},
    {1.99609375, 0x01, 0xff, 0x00},
    {2,        0x02, 0x00,  0x00},
    {2.125,    0x02, 0x20,  0x00},
    {2.25,     0x02, 0x40,  0x00},
    {2.375,    0x02, 0x60,  0x00},
    {2.5,      0x02, 0x80,  0x00},
    {2.625,    0x02, 0xa0,  0x00},
    {2.75,     0x02, 0xc0,  0x00},
    {2.875,    0x02, 0xe0,  0x00},
    {3,        0x03, 0x00,  0x00},
    {3.125,    0x03, 0x20,  0x00},
    {3.25,     0x03, 0x40,  0x00},
    {3.375,    0x03, 0x60,  0x00},
    {3.5,      0x03, 0x80,  0x00},
    {3.625,    0x03, 0xa0,  0x00},
    {3.75,     0x03, 0xc0,  0x00},
    {3.875,    0x03, 0xe0,  0x00},
    {4,        0x04, 0x00,  0x00},
    {4.25,     0x04, 0x40,  0x00},
    {4.5,      0x04, 0x80,  0x00},
    {4.75,     0x04, 0xc0,  0x00},
    {5,        0x05, 0x00,  0x00},
    {5.25,     0x05, 0x40,  0x00},
    {5.5,      0x05, 0x80,  0x00},
    {5.75,     0x05, 0xc0,  0x00},
    {6,        0x06, 0x00,  0x00},
    {6.25,     0x06, 0x40,  0x00},
    {6.5,      0x06, 0x80,  0x00},
    {6.75,     0x06, 0xc0,  0x00},
    {7,        0x07, 0x00,  0x00},
    {7.25,     0x07, 0x40,  0x00},
    {7.5,      0x07, 0x80,  0x00},
    {7.75,     0x07, 0xc0,  0x00},
    {8,        0x08, 0x00,  0x00},
    {8.5,      0x08, 0x80,  0x00},
    {9,        0x09, 0x00,  0x00},
    {9.5,      0x09, 0x80,  0x00},
    {10,       0x0A, 0x00,  0x00},
    {10.5,     0x0A, 0x80,  0x00},
    {11,       0x0B, 0x00,  0x00},
    {11.5,     0x0B, 0x80,  0x00},
    {12,       0x0C, 0x00,  0x00},
    {12.5,     0x0C, 0x80,  0x00},
    {13,       0x0D, 0x00,  0x00},
    {13.5,     0x0D, 0x80,  0x00},
    {14,       0x0E, 0x00,  0x00},
    {14.5,     0x0E, 0x80,  0x00},
    {15,       0x0F, 0x00,  0x00},
    {15.5,     0x0F, 0x80,  0x00},
    {15.99,    0X0F, 0xfd,  0x40},
};

AX_S32 os04a10_reg_read(ISP_PIPE_ID nPipeId, AX_U32 addr)
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

AX_S32 os04a10_reg_write(ISP_PIPE_ID nPipeId, AX_U32 addr, AX_U32 data)
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


AX_F32 os04a10_again2value(float gain, AX_U8 *again_in, AX_U8 *again_de)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!again_in || !again_de)
        return -1;

    count = sizeof(os04a10_again_table) / sizeof(os04a10_again_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > os04a10_again_table[i].gain) {
            continue;
        } else {
            *again_in = os04a10_again_table[i].again_in;
            *again_de = os04a10_again_table[i].again_de;
            SNS_DBG("again=%f, again_in=0x%x, again_de=0x%x\n", gain, *again_in, *again_de);
            return os04a10_again_table[i].gain;
        }
    }

    return -1;
}

AX_F32 os04a10_value2again(AX_U8 again_in, AX_U8 again_de)
{
    AX_U32 i = 0;
    AX_U32 count = 0;

    count = sizeof(os04a10_again_table) / sizeof(os04a10_again_table[0]);

    for (i = 0; i < count; i++) {
        if (again_in == os04a10_again_table[i].again_in &&
            again_de == os04a10_again_table[i].again_de) {

            SNS_DBG("get again=%f by 0x%x-0x%x\n",
                os04a10_again_table[i].gain, again_in, again_de);

            return os04a10_again_table[i].gain;
        }
    }

    SNS_ERR("get again fail by 0x%x-0x%x\n", again_in, again_de);

    return -1;
}

AX_F32 os04a10_dgain2value(float gain, AX_U8 *dgain_in, AX_U8 *dgain_de, AX_U8 *dgain_de2)
{
    AX_U32 i;
    AX_U32 count;
    AX_U32 ret_value = 0;

    if (!dgain_in || !dgain_de || !dgain_de2)
        return -1;

    count = sizeof(os04a10_dgain_table) / sizeof(os04a10_dgain_table[0]);

    for (i = 0; i < count; i++) {
        if (gain > os04a10_dgain_table[i].gain) {
            continue;
        } else {
            *dgain_in = os04a10_dgain_table[i].dgain_in;
            *dgain_de = os04a10_dgain_table[i].dgain_de;
            *dgain_de2 = os04a10_dgain_table[i].dgain_de2;
            SNS_DBG("dgain=%f, dgain_in=0x%x, dgain_de=0x%x dgain_de2=0x%x\n", gain, *dgain_in, *dgain_de, *dgain_de2);

            return os04a10_dgain_table[i].gain;
        }
    }

    return -1;
}

AX_F32 os04a10_value2dgain(AX_U8 dgain_in, AX_U8 dgain_de, AX_U8 dgain_de2)
{
    AX_U32 i = 0;
    AX_U32 count = 0;

    count = sizeof(os04a10_dgain_table) / sizeof(os04a10_dgain_table[0]);

    for (i = 0; i < count; i++) {
        if (dgain_in  == os04a10_dgain_table[i].dgain_in &&
            dgain_de  == os04a10_dgain_table[i].dgain_de &&
            dgain_de2 == os04a10_dgain_table[i].dgain_de2) {

            SNS_DBG("get dgain=%f by 0x%x-0x%x-0x%x\n",
                os04a10_dgain_table[i].gain, dgain_in, dgain_de, dgain_de2);

            return os04a10_dgain_table[i].gain;
        }
    }

    SNS_ERR("get dgain fail by 0x%x-0x%x-0x%x\n", dgain_in, dgain_de, dgain_de2);

    return -1;
}


AX_S32 os04a10_get_gain_table(AX_SNS_AE_GAIN_TABLE_T *params)
{
    AX_U32 i;
    AX_S32 ret = 0;
    if (!params)
        return -1;

    params->nAgainTableSize = sizeof(os04a10_again_table) / sizeof(os04a10_again_table[0]);
    params->nDgainTableSize = sizeof(os04a10_dgain_table) / sizeof(os04a10_dgain_table[0]);

    for (i = 0; i < params->nAgainTableSize ; i++) {
        nAgainTable[i] = os04a10_again_table[i].gain;
        params->pAgainTable = nAgainTable;
    }

    for (i = 0; i < params->nDgainTableSize ; i++) {
        nDgainTable[i] = os04a10_dgain_table[i].gain;
        params->pDgainTable = nDgainTable;
    }

    return ret;
}

AX_S32 os04a10_group_prepare(ISP_PIPE_ID nPipeId)
{
    AX_S32 result = 0;

    result |= os04a10_reg_write(nPipeId, 0x3200, 0x00);
    result |= os04a10_reg_write(nPipeId, 0x3201, 0x08);
    result |= os04a10_reg_write(nPipeId, 0x3202, 0x10);
    result |= os04a10_reg_write(nPipeId, 0x3203, 0x18);

    return result;
}

AX_S32 os04a10_write_settings(ISP_PIPE_ID nPipeId, AX_U32 setindex)
{
    AX_S32 i, errnum = 0;
    AX_U8 rBuf[1];
    AX_S32 reg_count = 0;
    const camera_i2c_reg_array *default_setting = NULL;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SNS_DBG("os04a10 setitng index: %d\n", setindex);

    default_setting = os04a10_settings_table[setindex];

    while ((default_setting + reg_count)->addr != 0x0000) {
        reg_count++;
    }

    SNS_DBG("os04a10 setitng index: %d, reg_count %d\n", setindex, reg_count);
    for (i = 0; i < reg_count; i++) {
        os04a10_reg_write(nPipeId, (default_setting + i)->addr, ((default_setting + i)->value));
#ifdef SENSOR_DEBUG
        usleep(2 * 1000);

        rBuf[0] = os04a10_reg_read(nPipeId, (default_setting + i)->addr);
        SNS_DBG(" addr: 0x%04x write:0x%02x read:0x%02x \r\n",
                (default_setting + i)->addr, (default_setting + i)->value, rBuf[0]);
#endif
    }

    return 0;
}

AX_U32 os04a10_get_hts(ISP_PIPE_ID nPipeId)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_U32 hts;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    hts_h = os04a10_reg_read(nPipeId, 0x380C);
    hts_l = os04a10_reg_read(nPipeId, 0x380D);

    hts = (AX_U32)(((hts_h & 0xF) << 8) | (AX_U32)(hts_l << 0));

    return hts;
}

AX_U32 os04a10_get_vs_hts(ISP_PIPE_ID nPipeId)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_U32 hts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts_h = os04a10_reg_read(nPipeId, 0x384C);
    hts_l = os04a10_reg_read(nPipeId, 0x384D);

    hts = (AX_U32)(((hts_h & 0xFF) << 8) | (AX_U32)(hts_l << 0));

    return hts;
}

AX_U32 os04a10_set_hts(ISP_PIPE_ID nPipeId, AX_U32 hts)
{
    AX_U8 hts_h;
    AX_U8 hts_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    hts_l = hts & 0xFF;
    hts_h = (hts & 0xFF00) >> 8;

    result |= os04a10_reg_write(nPipeId, 0x380C, hts_h);
    result |= os04a10_reg_write(nPipeId, 0x380D, hts_l);

    return result;
}

AX_U32 os04a10_get_vts(ISP_PIPE_ID nPipeId)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_U32 vts;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_h = os04a10_reg_read(nPipeId, OS04A10_VTS_H);
    vts_l = os04a10_reg_read(nPipeId, OS04A10_VTS_L);

    vts = (AX_U32)(((vts_h & 0xFF) << 8) | (AX_U32)(vts_l << 0));

    return vts;
}

AX_U32 os04a10_set_vts(ISP_PIPE_ID nPipeId, AX_U32 vts)
{
    AX_U8 vts_h;
    AX_U8 vts_l;
    AX_S32 result = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    vts_l = vts & 0xFF;
    vts_h = (vts & 0xFF00) >> 8;

    result |= os04a10_sns_update_regs_table(nPipeId, OS04A10_VTS_H, vts_h);
    result |= os04a10_sns_update_regs_table(nPipeId, OS04A10_VTS_L, vts_l);

    return result;
}

AX_U32 os04a10_get_sef1_expline(ISP_PIPE_ID nPipeId)
{
    AX_U8 exp_h = 0;
    AX_U8 exp_l = 0;
    AX_U32 expline = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    exp_h = os04a10_reg_read(nPipeId, OS04A10_SHORT_EXP_LINE_H);
    exp_l = os04a10_reg_read(nPipeId, OS04A10_SHORT_EXP_LINE_L);

    expline = exp_h << 8 | exp_l;

    SNS_DBG("get short frame expline:0x%x\n", expline);

    return expline;
}


AX_U32 os04a10_get_dcg_status(ISP_PIPE_ID nPipeId)
{
    AX_U8 dcg = 0;
    AX_U32 status = 0;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    dcg = os04a10_reg_read(nPipeId, 0x376c);
    if(dcg & 0x70) {
        status = AX_LCG_MODE;
    } else {
        status = AX_HCG_MODE;
    }

    SNS_DBG("get dcg status:%d(0x%x)\n", status, dcg);

    return status;
}


AX_U32 os04a10_get_l2s_offset(ISP_PIPE_ID nPipeId)
{
    AX_U32 offset = 0;

    if (nPipeId < 0 || (nPipeId >= SENSOR_MAX_NUM))
        return -1;

    /* The calculation formula is obtained from FAE, linegap=VS frmae exp time + offset, os04a10 is 4 */
    offset = os04a10_reg_read(nPipeId, 0x3797);

    offset = offset & 0x1f;

    offset = offset + 2;

    return offset;
}


AX_F32 os04a10_get_sclk(ISP_PIPE_ID nPipeId)
{
    AX_U8 pre_div0;
    AX_U8 pre_div;
    AX_U16 multiplier;
    AX_U8 post_div;
    AX_U8 sram_div;
    AX_U8 st_div;
    AX_U8 t_div;
    float mclk;
    float sclk;

    mclk = OS04A10_INCK_24M;

    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    pre_div0 = (os04a10_reg_read(nPipeId, 0x0322) & 0x1) + 1;

    pre_div = os04a10_reg_read(nPipeId, 0x0323) & 0x7;
    if (pre_div == 0) {
        pre_div = 1;
    } else if (pre_div == 1) {
        pre_div = 1.5;
    } else if (pre_div == 2) {
        pre_div = 2;
    } else if (pre_div == 3) {
        pre_div = 2.5;
    } else if (pre_div == 4) {
        pre_div = 3;
    } else if (pre_div == 5) {
        pre_div = 4;
    } else if (pre_div == 6) {
        pre_div = 6;
    } else if (pre_div == 7) {
        pre_div = 8;
    } else {
    }

    multiplier = (os04a10_reg_read(nPipeId, 0x0324) & 0x3) << 8;
    multiplier = multiplier | ((os04a10_reg_read(nPipeId, 0x0325)) & 0xFF);

    post_div = (os04a10_reg_read(nPipeId, 0x032f) & 0xF) + 1;
    sram_div = (os04a10_reg_read(nPipeId, 0x0327) & 0xF) + 1;
    st_div = (os04a10_reg_read(nPipeId, 0x0328) & 0xF) + 1;

    t_div = os04a10_reg_read(nPipeId, 0x032a) & 0x7;
    if (t_div == 0) {
        t_div = 1;
    } else if (t_div == 1) {
        t_div = 1.5;
    } else if (t_div == 2) {
        t_div = 2;
    } else if (t_div == 3) {
        t_div = 2.5;
    } else if (t_div == 4) {
        t_div = 3;
    } else if (t_div == 5) {
        t_div = 3.5;
    } else if (t_div == 6) {
        t_div = 4;
    } else if (t_div == 7) {
        t_div = 5;
    } else {
    }

    sclk = (((((((float)(mclk * 1000 * 1000) / pre_div0) / pre_div) * multiplier) / post_div) / st_div) / t_div);
    /*printf("%s pre_div0=0x%x, pre_div=0x%x, multiplier=0x%x, post_div=0x%x, sram_div=0x%x, st_div=0x%x, t_div=0x%x \n", \
          __func__, pre_div0, pre_div, multiplier, post_div, sram_div, st_div, t_div); */

    return sclk;
}



AX_S32 os04a10_sensor_hw_reset(unsigned int gpio_num, unsigned int gpio_out_val)
{
    FILE *fp = NULL;
    char file_name[50];
    char buf[10];

    sprintf(file_name, "/sys/class/gpio/gpio%d", gpio_num);
    SNS_DBG("file_name = %s\n", file_name);

    sprintf(file_name, "/sys/class/gpio/gpio%d/direction", gpio_num);
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    fprintf(fp, "out");
    fclose(fp);

    sprintf(file_name, "/sys/class/gpio/gpio%d/value", gpio_num);
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    if (gpio_out_val) {
        strcpy(buf, "1");
    } else {
        strcpy(buf, "0");
    }
    fprintf(fp, "%s", buf);
    fclose(fp);

    return 0;
}

/* sensor reset : only for AX620 demo/evb board */
AX_S32 os04a10_reset(ISP_PIPE_ID nPipeId, AX_S32 i2cDevNum)
{
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    FILE *fp = NULL;
    char file_name[50];
    int resetPinBase = 496;
    int nGpioNum = 0;

    nGpioNum = resetPinBase + i2cDevNum;

    if(i2cDevNum == 6) {
        nGpioNum = 498;
    }
    strcpy(file_name, "/sys/class/gpio/export");
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        printf("Cannot open %s.\n", file_name);
        return -1;
    }
    fprintf(fp, "%d", nGpioNum);
    fclose(fp);


    os04a10_sensor_hw_reset(nGpioNum, 0);
    usleep(5 * 1000);
    os04a10_sensor_hw_reset(nGpioNum, 1);
    usleep(10000);

    return 0;
}

AX_S32 os04a10_sns_update_regs_table(ISP_PIPE_ID nPipeId, AX_U32 nRegsAddr, AX_U8 nRegsValue)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gOs04a10AeRegsTable) / sizeof(gOs04a10AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        if (nRegsAddr == sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nData = nRegsValue;
            gOs04a10AeRegsTable[i].nRegValue = nRegsValue;
            break;
        }
    }

    if (nNum <= i) {
        SNS_ERR(" reg addr 0x%x not find.\n", nRegsAddr);
        return SNS_ERR_CODE_INVALID_ADDRESS;
    }

    return SNS_SUCCESS;
}

AX_S32 os04a10_sns_update_group_hold_regs(ISP_PIPE_ID nPipeId, AX_U32 nRegsIdx, AX_U8 nRegsValue)
{
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gOs04a10AeRegsTable) / sizeof(gOs04a10AeRegsTable[0]);
    SNS_CHECK_VALUE_RANGE_VALID(nRegsIdx, 0, nNum - 1);

    sns_obj->sztRegsInfo[0].sztI2cData[nRegsIdx].nData = nRegsValue;
    gOs04a10AeRegsTable[nRegsIdx].nRegValue = nRegsValue;

    SNS_DBG(" idx = %d, reg addr 0x%x, reg data 0x%x.\n", nRegsIdx, sns_obj->sztRegsInfo[0].sztI2cData[nRegsIdx].nRegAddr,
            nRegsValue);

    return SNS_SUCCESS;
}

AX_S32 os04a10_sns_refresh_all_regs_from_tbl(ISP_PIPE_ID nPipeId)
{
    AX_S32 i = 0;
    AX_U32 nNum = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);
    SENSOR_GET_CTX(nPipeId, sns_obj);

    nNum = sizeof(gOs04a10AeRegsTable) / sizeof(gOs04a10AeRegsTable[0]);

    for (i = 0; i < nNum; i++) {
        gOs04a10AeRegsTable[i].nRegValue = os04a10_reg_read(nPipeId, gOs04a10AeRegsTable[i].nRegAddr);
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gOs04a10AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gOs04a10AeRegsTable[i].nRegValue;

        SNS_DBG(" nRegAddr 0x%x, nRegValue 0x%x\n", gOs04a10AeRegsTable[i].nRegAddr, gOs04a10AeRegsTable[i].nRegValue);
    }

    /* linegap default vaule */
    if (IS_HDR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
        /* 2DOL line gap */
        sns_obj->sztRegsInfo[0].tSnsExpInfo.szLineGap[HDR_LONG_FRAME_IDX] =
            os04a10_get_sef1_expline(nPipeId) + os04a10_get_l2s_offset(nPipeId);
    }

    return SNS_SUCCESS;
}


static AX_VOID _sns_reg_info_init(ISP_PIPE_ID nPipeId, SNS_STATE_OBJ *sns_obj)
{
    AX_S32 i = 0;

    sns_obj->sztRegsInfo[0].eSnsType = ISP_SNS_CONNECT_I2C_TYPE;
    sns_obj->sztRegsInfo[0].tComBus.I2cDev = g_Os04A10BusInfo[nPipeId].I2cDev;
    sns_obj->sztRegsInfo[0].nRegNum = sizeof(gOs04a10AeRegsTable) / sizeof(gOs04a10AeRegsTable[0]);
    sns_obj->sztRegsInfo[0].tSnsExpInfo.nDelayNum = 2;

    for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
        sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nDevAddr = OS04A10_SLAVE_ADDR;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr = gOs04a10AeRegsTable[i].nRegAddr;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nAddrByteNum = OS04A10_ADDR_BYTE;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nData = gOs04a10AeRegsTable[i].nRegValue;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nDataByteNum = OS04A10_DATA_BYTE;
        sns_obj->sztRegsInfo[0].sztI2cData[i].nDelayFrmNum = gOs04a10AeRegsTable[i].nDelayFrmNum;
        if(IS_LINEAR_MODE(sns_obj->sns_mode_obj.eHDRMode)) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nIntPos = AX_SNS_L_FSOF;
        } else {
            sns_obj->sztRegsInfo[0].sztI2cData[i].nIntPos = AX_SNS_S_FSOF;
        }
        SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                gOs04a10AeRegsTable[i].nRegAddr, gOs04a10AeRegsTable[i].nRegValue);
    }

    sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP1_IDX].nData = 0x00;
    sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP2_IDX].nData = 0x01;
    sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP3_IDX].nData = 0x11;
    sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP4_IDX].nData = 0x05;
    sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP5_IDX].nData = 0x01;
    sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP6_IDX].nData = 0xa0;

    return;
}

static AX_VOID _sns_reg_info_update(ISP_PIPE_ID nPipeId, SNS_STATE_OBJ *sns_obj)
{
    AX_S32 i = 0;

    for (i = 0; i < sns_obj->sztRegsInfo[0].nRegNum; i++) {
        if (sns_obj->sztRegsInfo[0].sztI2cData[i].nData == sns_obj->sztRegsInfo[1].sztI2cData[i].nData) {
            sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_FALSE;
        } else {
            SNS_DBG("[%2d] nRegAddr 0x%x, nRegValue 0x%x\n", i,
                    sns_obj->sztRegsInfo[0].sztI2cData[i].nRegAddr, sns_obj->sztRegsInfo[0].sztI2cData[i].nData);

            if (AX_HCG_LCG_IDX != i) {
                sns_obj->sztRegsInfo[0].sztI2cData[i].bUpdate = AX_TRUE;
            } else {
                sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP1_IDX].bUpdate = AX_TRUE;
                sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP2_IDX].bUpdate = AX_TRUE;
                sns_obj->sztRegsInfo[0].sztI2cData[AX_HCG_LCG_IDX].bUpdate = AX_TRUE;
                sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP3_IDX].bUpdate = AX_TRUE;
                sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP4_IDX].bUpdate = AX_TRUE;
                sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP5_IDX].bUpdate = AX_TRUE;
                sns_obj->sztRegsInfo[0].sztI2cData[AX_GROUP6_IDX].bUpdate = AX_TRUE;
                break;
            }
        }
    }

    return;
}

AX_S32 os04a10_ae_get_sensor_reg_info(ISP_PIPE_ID nPipeId, AX_SNS_REGS_CFG_TABLE_T *ptSnsRegsInfo)
{
    AX_S32 nRet = 0;
    SNS_STATE_OBJ *sns_obj = AX_NULL;

    SNS_CHECK_PTR_VALID(ptSnsRegsInfo);
    SNS_CHECK_VALUE_RANGE_VALID(nPipeId, 0, DEF_VIN_PIPE_MAX_NUM - 1);

    SENSOR_GET_CTX(nPipeId, sns_obj);

    if ((AX_FALSE == sns_obj->bSyncInit) || (AX_FALSE == ptSnsRegsInfo->bConfig)) {
        /* sync config */
        SNS_DBG(" bSyncInit %d, bConfig %d\n", sns_obj->bSyncInit, ptSnsRegsInfo->bConfig);

        _sns_reg_info_init(nPipeId, sns_obj);
        sns_obj->bSyncInit = AX_TRUE;
    } else {
        _sns_reg_info_update(nPipeId, sns_obj);
    }

    ptSnsRegsInfo->bConfig = AX_FALSE;
    memcpy(ptSnsRegsInfo, &sns_obj->sztRegsInfo[0], sizeof(AX_SNS_REGS_CFG_TABLE_T));
    /* Save the current register table */
    memcpy(&sns_obj->sztRegsInfo[1], &sns_obj->sztRegsInfo[0], sizeof(AX_SNS_REGS_CFG_TABLE_T));

    return nRet;
}

