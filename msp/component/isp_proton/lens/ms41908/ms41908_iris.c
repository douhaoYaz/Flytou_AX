/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#include "ax_lens_iris_struct.h"
#include "ms41908_af.h"
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

AX_S32 irisPos = 0;

static AX_S32 lens_iris_init(AX_U8 nPipeId)
{
    ms41908_actuator_init(nPipeId, 1, 0);
    return 0;
}

static AX_S32 lens_iris_exit(AX_U8 nPipeId)
{
    return 0;
}

static AX_S32 lens_iris_set_param(AX_U8 nPipeId, AX_IRIS_PARAMS_T *pIrisParam)
{
    return 0;
}

static AX_S32 lens_iris_get_param(AX_U8 nPipeId, AX_IRIS_PARAMS_T *pIrisParam)
{
    return 0;
}

static AX_S32 lens_dciris_set_param(AX_U8 nPipeId, AX_DCIRIS_PARAMS_T *pDcIrisParam)
{
    return 0;
}

static AX_S32 lens_dciris_get_param(AX_U8 nPipeId, AX_DCIRIS_PARAMS_T *pDcIrisParam)
{
    return 0;
}

static AX_S32 lens_piris_set_aperture_pos(AX_U8 nPipeId, AX_S32 nPos)
{
    MS41908_spi_write(ms41908_spi_fd, 0x00, nPos);
    MS41908_spi_write(ms41908_spi_fd, 0x01,0x6000);
    MS41908_spi_write(ms41908_spi_fd, 0x02,0x66f0);
    MS41908_spi_write(ms41908_spi_fd, 0x03,0x0e10);
    MS41908_spi_write(ms41908_spi_fd, 0x04,0xd640);
    MS41908_spi_write(ms41908_spi_fd, 0x05,0x0024);
    MS41908_spi_write(ms41908_spi_fd, 0x0B,0x0400);
    MS41908_spi_write(ms41908_spi_fd, 0x0a,0x0000);
    MS41908_spi_write(ms41908_spi_fd, 0x0e,0x0300);
    /*vd signal*/
    ms41908_gpio_ctrl(VD_IS_GPIO, 1);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(VD_IS_GPIO, 0);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);
    
    irisPos = nPos;
    return 0;
}

static AX_S32 lens_dciris_set_duty(AX_U8 nPipeId, AX_U32 nDuty)
{
    return 0;
}

static AX_S32 lens_dciris_get_duty(AX_U8 nPipeId, AX_U32 *pDuty)
{
    return 0;
}

static AX_S32 lens_piris_get_aperture_pos(AX_U8 nPipeId, AX_S32 *pPos)
{
    *pPos = irisPos;
    return 0;
}

static AX_S32 lens_piris_set_param(AX_U8 nPipeId, AX_PIRIS_PARAMS_T *pPIrisParam)
{
    return 0;
}

static AX_S32 lens_piris_get_param(AX_U8 nPipeId, AX_PIRIS_PARAMS_T *pPIrisParam)
{
    return 0;
}

AX_LENS_ACTUATOR_IRIS_FUNC_T gLensIrisMs41908Obj = {

    .pfn_iris_init                  = lens_iris_init,
    .pfn_iris_set_param             = lens_iris_set_param,
    .pfn_iris_get_param             = lens_iris_get_param,
    .pfn_dciris_set_param           = lens_dciris_set_param,
    .pfn_dciris_get_param           = lens_dciris_get_param,
    .pfn_piris_set_aperture_pos     = lens_piris_set_aperture_pos,
    .pfn_piris_get_aperture_pos     = lens_piris_get_aperture_pos,
    .pfn_piris_set_param            = lens_piris_set_param,
    .pfn_piris_get_param            = lens_piris_get_param,
    .pfn_iris_exit                  = lens_iris_exit,

};

