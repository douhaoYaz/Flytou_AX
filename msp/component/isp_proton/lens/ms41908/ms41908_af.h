/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef __MS41908_AF_H__
#define __MS41908_AF_H__

#include <stdlib.h>
#include <math.h>
#include "spi.h"
#include "ax_base_type.h"

#define VD_FZ_GPIO 86
#define VD_IS_GPIO 83

typedef enum AXMS41908LogLevel_s
{
    AXMS41908_LOG_MIN    = 0,
    AXMS41908_LOG_DBG    = 1,
    AXMS41908_LOG_INFO   = 2,
    AXMS41908_LOG_NOTICE = 3,
    AXMS41908_LOG_WARN   = 4,
    AXMS41908_LOG_ERROR  = 5,
    AXMS41908_LOG_MAX
} AXMS41908LogLevel_t;

typedef enum AXMS41908LogTarget_s
{
    AXMS41908_LOG_TARGET_MIN    = 0,
    AXMS41908_LOG_TARGET_STDERR = 1,
    AXMS41908_LOG_TARGET_SYSLOG = 2,
    AXMS41908_LOG_TARGET_NULL   = 3,
    AXMS41908_LOG_TARGET_MAX
} AXMS41908LogTarget_t;

typedef enum AXMS41908MoveStatus_s
{
    AXMS41908_ZOOM1_IDLE = 0,
    AXMS41908_ZOOM1_MOVEFINISH = 1,
    AXMS41908_ZOOM1_FIND_PI = 2,
    AXMS41908_ZOOM1_FIND_PI_ERR = 3,
    AXMS41908_ZOOM2_IDLE = 4,
    AXMS41908_ZOOM2_MOVEFINISH = 5,
    AXMS41908_ZOOM2_FIND_PI = 6,
    AXMS41908_ZOOM2_FIND_PI_ERR = 7,
} AXMS41908MoveStatus_t;

AX_S32 MS41908_spi_write(AX_S32 spi_fd, AX_U8 reg, AX_U16 data);
AX_S32 ms41908_gpio_ctrl(AX_U8 gpio_num, AX_U8 gpio_out_val);
AX_S32 ms41908_actuator_init(AX_U8 pipe, AX_U8 bus_num, AX_U8 cs);

extern AX_S32 ms41908_spi_fd;
extern struct timespec delaytime;

#endif