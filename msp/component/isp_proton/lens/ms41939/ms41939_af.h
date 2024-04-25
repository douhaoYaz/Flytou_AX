/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef __MS41939_AF_H__
#define __MS41939_AF_H__

#include <stdlib.h>
#include <math.h>
#include "ax_base_type.h"
#include "spi.h"

typedef enum AXMS41939LogLevel_s
{
    AXMS41939_LOG_MIN    = 0,
    AXMS41939_LOG_DBG    = 1,
    AXMS41939_LOG_INFO   = 2,
    AXMS41939_LOG_NOTICE = 3,
    AXMS41939_LOG_WARN   = 4,
    AXMS41939_LOG_ERROR  = 5,
    AXMS41939_LOG_MAX
} AXMS41939LogLevel_t;

typedef enum AXMS41939LogTarget_s
{
    AXMS41939_LOG_TARGET_MIN    = 0,
    AXMS41939_LOG_TARGET_STDERR = 1,
    AXMS41939_LOG_TARGET_SYSLOG = 2,
    AXMS41939_LOG_TARGET_NULL   = 3,
    AXMS41939_LOG_TARGET_MAX
} AXMS41939LogTarget_t;

typedef enum AXMS41939MoveStatus_s
{
    AXMS41939_IDLE = 0,
    AXMS41939_MOVE_FINISH    = 1,
    AXMS41939_FIND_PI        = 2,
    AXMS41939_FIND_PI_ERR    = 3,
} AXMS41939MoveStatus_t;

#endif