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
#include "dciris.h"
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

AX_LEN_DCIRIS_DRV_T gDcIrisDrvObj[ACTUATOR_MAX_NUM] = {0};

static AX_S32 lens_iris_init(AX_U8 nPipeId)
{
    if (nPipeId < 0 || (nPipeId >= ACTUATOR_MAX_NUM)) {
        return -1 ;
    }

    gDcIrisDrvObj[nPipeId].chip = DCIRIS_PWM_CHIP_ID;
    gDcIrisDrvObj[nPipeId].channel = DCIRIS_PWM_CHANNEL_ID;
    gDcIrisDrvObj[nPipeId].period = DCIRIS_PWM_PERIOD;
    gDcIrisDrvObj[nPipeId].irisDuty = 0;

    return 0;
}

static AX_S32 lens_iris_exit(AX_U8 nPipeId)
{
    if (nPipeId < 0 || (nPipeId >= ACTUATOR_MAX_NUM)) {
        return -1 ;
    }

    memset(&gDcIrisDrvObj[nPipeId], 0, sizeof(AX_LEN_DCIRIS_DRV_T));

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
    return 0;
}

static AX_S32 pwm_config(const AX_CHAR *pwm_path, const AX_CHAR *attr, const AX_CHAR *val)
{
    AX_CHAR file_path[100];
    AX_S32 len;
    AX_S32 fd;

    sprintf(file_path, "%s/%s", pwm_path, attr);
    if (0 > (fd = open(file_path, O_WRONLY))) {
        perror("open error");
        return fd;
    }

    len = strlen(val);
    if (len != write(fd, val, len)) {
        perror("write error");
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static AX_S32 lens_dciris_set_duty(AX_U8 nPipeId, AX_F32 nDuty)
{
    AX_S32 chip;
    AX_S32 channel;
    AX_S32 period;
    AX_S32 duty_drv;
    AX_CHAR period_str[16] = {0};
    AX_CHAR duty_drv_str[16] = {0};
    AX_CHAR pwm_path[100] = {0};

    if (nPipeId < 0 || (nPipeId >= ACTUATOR_MAX_NUM)) {
        return -1 ;
    }

    chip = gDcIrisDrvObj[nPipeId].chip;
    channel = gDcIrisDrvObj[nPipeId].channel;
    period = gDcIrisDrvObj[nPipeId].period;
    duty_drv = (AX_S32)(nDuty * period / 100);

    sprintf(pwm_path, "/sys/class/pwm/pwmchip%d/pwm%d", chip, channel);

    if (access(pwm_path, F_OK)) {
        AX_CHAR temp[100] = {0};
        AX_CHAR channel_str[16] = {0};
        AX_S32 fd;
        AX_S32 len;

        sprintf(temp, "/sys/class/pwm/pwmchip%d/export", chip);
        if (0 > (fd = open(temp, O_WRONLY))) {
            perror("open error");
            return -1;
        }

        sprintf(channel_str, "%d", channel);
        len = strlen(channel_str);
        if (len != write(fd, channel_str, len)) {
            perror("write error");
            close(fd);
            return -1;
        }

        close(fd);
    }

    sprintf(period_str, "%d", period);
    sprintf(duty_drv_str, "%d", duty_drv);

    if (pwm_config(pwm_path, "period", period_str))
        return -1;

    if (pwm_config(pwm_path, "duty_cycle", duty_drv_str))
        return -1;

    if (pwm_config(pwm_path, "enable", "1"))
        return -1;

    gDcIrisDrvObj[nPipeId].irisDuty = nDuty;

    return 0;
}

static AX_S32 lens_dciris_get_duty(AX_U8 nPipeId, AX_F32 *pDuty)
{
    if (nPipeId < 0 || (nPipeId >= ACTUATOR_MAX_NUM)) {
        return -1 ;
    }

    *pDuty = gDcIrisDrvObj[nPipeId].irisDuty;
    return 0;
}


static AX_S32 lens_piris_get_aperture_pos(AX_U8 nPipeId, AX_S32 *pPos)
{
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

AX_LENS_ACTUATOR_IRIS_FUNC_T gLensDcIrisObj = {

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
    .pfn_dciris_set_duty            = lens_dciris_set_duty,
    .pfn_dciris_get_duty            = lens_dciris_get_duty,
};

