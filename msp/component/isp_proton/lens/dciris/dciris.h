/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef __DC_IRIS_H__
#define __DC_IRIS_H__

#define DCIRIS_PWM_CHIP_ID       (4)
#define DCIRIS_PWM_CHANNEL_ID    (1)
#define DCIRIS_PWM_PERIOD        (100000)

typedef struct _AX_LEN_DCIRIS_DRV_T_ {
    AX_S32 chip;
    AX_S32 channel;
    AX_S32 period;
    AX_F32 irisDuty;
}AX_LEN_DCIRIS_DRV_T;

#endif