/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/
#include "ax_base_type.h"
#include "ax_lens_af_struct.h"
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <syslog.h>
#include <time.h>
#include "ms41939_af.h"

const char ms41939_g_acName[] = "VcmDriver_MS41939";
#define RSTB_GPIO 79
#define VD_FZ_GPIO 82
#define PI_GPIO 68
#define HIGH_TO_LOW_DIRECTOR 0x1D00
#define LOW_TO_HIGH_DIRECTOR 0x1C00
#define PPS_VALUE 600

AX_S32 ms41939_spi_fd = 0;
AX_U8 ms41939_rstb_flag = 0;
AX_S32 focus_previous_position = 0;
AX_U8 ms41939_move_status = 0;

struct timespec delaytime = {0, 300};

/* private api */
#define axms41939_err(fmt, ...)      axms41939_print(AXMS41939_LOG_ERROR,  "[AXMS41939]: "fmt, ##__VA_ARGS__)
#define axms41939_warn(fmt, ...)     axms41939_print(AXMS41939_LOG_WARN,   "[AXMS41939]: "fmt, ##__VA_ARGS__)
#define axms41939_notice(fmt, ...)   axms41939_print(AXMS41939_LOG_NOTICE, "[AXMS41939]: "fmt, ##__VA_ARGS__)
#define axms41939_info(fmt, ...)     axms41939_print(AXMS41939_LOG_INFO,   "[AXMS41939]: "fmt, ##__VA_ARGS__)
#define axms41939_dbg(fmt, ...)      axms41939_print(AXMS41939_LOG_DBG,    "[AXMS41939]: "fmt, ##__VA_ARGS__)
static int g_validPipeCnt = 0;
static int g_ms41939LogLevel  = AXMS41939_LOG_WARN;
static int g_ms41939LogTarget = AXMS41939_LOG_TARGET_SYSLOG;

extern AX_LENS_ACTUATOR_AF_FUNC_T gLensAfZoomMs41908Obj;

AX_S32 axms41939_print(AX_U32 level, AX_CHAR *format, ...)
{
    AX_S32 r = 0;

    if (level >= g_ms41939LogLevel) {
        va_list vlist;
        va_start(vlist, format);
        char buf[2048] = {0};
        int len = vsnprintf(buf, sizeof(buf), format, vlist);
        if (len < sizeof(buf)) {
            buf[len] = '\0';
        } else {
            buf[sizeof(buf) - 1] = '\0';
        }
        va_end(vlist);
        if (g_ms41939LogTarget == AXMS41939_LOG_TARGET_STDERR) {
            fprintf(stderr, "%s", buf);
        } else if (g_ms41939LogTarget == AXMS41939_LOG_TARGET_SYSLOG) {
            syslog(level, "%s", buf);
        }
    }
    return r;
}

AX_S32 axms41939_init_log(AX_U8 pipe)
{
    if (pipe >= ACTUATOR_MAX_NUM) {
        return -1;
    }

    // Only Init Log Level & Target Only Once
    if (g_validPipeCnt == 1) {
        char *log_level  = getenv("AXMS41939_LOG_level");
        if (log_level) {
            g_ms41939LogLevel  = (AXMS41939LogLevel_t)atoi(log_level);
        } else {
            g_ms41939LogLevel = AXMS41939_LOG_WARN;
        }

        char *log_target = getenv("AXMS41939_LOG_target");
        if (log_target) {
            g_ms41939LogTarget = (AXMS41939LogTarget_t)atoi(log_target);
        } else {
            g_ms41939LogTarget = AXMS41939_LOG_TARGET_SYSLOG;
        }
    }
    return 0;
}

AX_S32 ms41939_gpio_ctrl(AX_U8 gpio_num, AX_U8 gpio_out_val)
{
    FILE *fp = NULL;
    char file_name[50];
    char buf[10];
    AX_U8 pi_level;
    char str[2];

    sprintf(file_name, "/sys/class/gpio/gpio%d", gpio_num);
    if (0 != access(file_name, F_OK)) {
        sprintf(file_name, "/sys/class/gpio/export");
        fp = fopen(file_name, "w");
        if (fp == NULL) {
            axms41939_err("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "%d", gpio_num);
        fclose(fp);

        sprintf(file_name, "/sys/class/gpio/gpio%d/direction", gpio_num);
        fp = fopen(file_name, "w");
        if (fp == NULL) {
            axms41939_err("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "out");
        fclose(fp);
    }

    sprintf(file_name, "/sys/class/gpio/gpio%d/value", gpio_num);
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        axms41939_err("Cannot open %s.\n", file_name);
        return -1;
    }
    if (gpio_out_val) {
        strcpy(buf, "1");
    } else {
        strcpy(buf, "0");
    }
    fprintf(fp, "%s", buf);
    fclose(fp);

    sprintf(file_name, "/sys/class/gpio/gpio%d/value", gpio_num);
    fp = fopen(file_name, "r");
    if (fp == NULL) {
        axms41939_err("Cannot open %s.\n", file_name);
        return -1;
    }
    if(fgets(str, 2, fp) != NULL ) {
        pi_level = (AX_U8)atoi(str);
        axms41939_dbg("pi_level %d gpio_num %d \n", pi_level,gpio_num);
    }
    fclose(fp);
    return 0;
}

AX_S32 ms41939_rstb(AX_U8 rstb_gpio)
{
    ms41939_gpio_ctrl(rstb_gpio, 0);
    nanosleep(&delaytime, NULL);
    ms41939_gpio_ctrl(rstb_gpio, 1);
    ms41939_rstb_flag = 1;

    return 0;
}

AX_S32 MS41939_spi_write(AX_S32 spi_fd, AX_U8 reg, AX_U16 data)
{
    AX_S32 ret;
    ret = SPI_write(ms41939_spi_fd, reg, data);
    return ret;
}

AX_S32 MS41939_chip_init(AX_U8 pipe)
{
    //RSTB
    AX_LENS_ACTUATOR_AF_FUNC_T *actuatorObj = &gLensAfZoomMs41908Obj;
    AX_U8 rstb_flag = actuatorObj->pfn_af_zoom_rstb_status(pipe);
    if(!rstb_flag)
    {
        ms41939_rstb(RSTB_GPIO);
        axms41939_dbg("%s rstb_flag %d \n",__func__, rstb_flag);
    }

    /*send cmd*/
    MS41939_spi_write(ms41939_spi_fd, 0x0B,0x0080);
    MS41939_spi_write(ms41939_spi_fd, 0x20,0x1e01);
    MS41939_spi_write(ms41939_spi_fd, 0x21,0x0087);
    MS41939_spi_write(ms41939_spi_fd, 0x27,0x0001);
    MS41939_spi_write(ms41939_spi_fd, 0x28,0xc8c8);
    MS41939_spi_write(ms41939_spi_fd, 0x29,0x1500);
    MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);

    /*vd signal*/
    ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);

    axms41939_dbg("%s finish! \n",__func__);
    return 0;
}

AX_S32 MS41939_spi_init(AX_U8 bus_num, AX_U8 cs)
{
    return SPI_init(bus_num, cs);
}

AX_U8 ms41939_get_gpio_values(AX_U8 pi_gpio_num)
{
    AX_U8 pi_level = 0;
    FILE *fp = NULL;
    char file_name[50];
    char str[2];

    sprintf(file_name, "/sys/class/gpio/gpio%d", pi_gpio_num);
    if (0 != access(file_name, F_OK)) {
        sprintf(file_name, "/sys/class/gpio/export");
        fp = fopen(file_name, "w");
        if (fp == NULL) {
            axms41939_dbg("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "%d", pi_gpio_num);
        fclose(fp);

        sprintf(file_name, "/sys/class/gpio/gpio%d/direction", pi_gpio_num);
        fp = fopen(file_name, "w");
        if (fp == NULL) {
            axms41939_dbg("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "in");
        fclose(fp);
    }
    sprintf(file_name, "/sys/class/gpio/gpio%d/value", pi_gpio_num);
    fp = fopen(file_name, "r");
    if (fp == NULL) {
        axms41939_dbg("Cannot open %s.\n", file_name);
        return -1;
    }
    if(fgets(str, 2, fp) != NULL ) {
        pi_level = (AX_U8)atoi(str);
        axms41939_dbg("pi_level %d gpio_num %d \n", pi_level, pi_gpio_num);
    }
    fclose(fp);
    return pi_level;
}

AX_U8 get_MS41939_move_status(AX_U8 nPipeId)
{
    AX_U8 status;
    status = ms41939_move_status;
    return status;
}

AX_U8 set_MS41939_move_status(AXMS41939MoveStatus_t status)
{
    ms41939_move_status = (AX_U8)status;
    return 0;
}

AX_S32 ms41939_LenPI_Position_Check(AX_U8 pi_gpio_num, AX_U8 vd_gpio_num)
{
    AX_U8 current_pi_level;
    AX_U8 HighArea_pi_level;
    AX_U8 LowArea_pi_level;
    AX_S32 focus_highStep = 690;
    AX_S32 focus_LowStep = 3048;
    AX_U8 step = 0;
    AX_U8 set_reg_enable = 0;
    AX_U8 tmp_level = 0;

    current_pi_level = ms41939_get_gpio_values(pi_gpio_num);
    HighArea_pi_level = current_pi_level;
    LowArea_pi_level = current_pi_level;
    axms41939_dbg("pi_gpio_num %d \n", pi_gpio_num);

    /*high area*/
    while(current_pi_level == 1)
    {
        if((step == 0) && (pi_gpio_num == PI_GPIO))
        {
            if(!set_reg_enable)
            {
                MS41939_spi_write(ms41939_spi_fd, 0x29,0x1530);
                MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
            focus_highStep = focus_highStep - (48 / 8);
            if(focus_highStep < 0)
            {
                axms41939_err("can not find low level pi\n");
                set_MS41939_move_status(AXMS41939_FIND_PI_ERR);
                break;
            }
        }

        if((step == 1) && (pi_gpio_num == PI_GPIO))
        {
            if(set_reg_enable)
            {
                MS41939_spi_write(ms41939_spi_fd, 0x29,0x1418);
                MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 0;
            }
        }

        if((step == 2) && (pi_gpio_num == PI_GPIO))
        {
            if(!set_reg_enable)
            {
                MS41939_spi_write(ms41939_spi_fd, 0x29,0x1508);
                MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
        }
        axms41939_dbg("step %d \n", step);
        tmp_level = HighArea_pi_level;
        ms41939_gpio_ctrl(vd_gpio_num, 1);
        nanosleep(&delaytime, NULL);
        ms41939_gpio_ctrl(vd_gpio_num, 0);
        usleep(19500);
        HighArea_pi_level = ms41939_get_gpio_values(pi_gpio_num);
        if(HighArea_pi_level != tmp_level)
        {
            if(step == 2)
            {
                axms41939_dbg("%s find pi postion ok! \n",__func__);
                set_MS41939_move_status(AXMS41939_FIND_PI);
                break;
            }
            step++;
        }
        continue;
    }

    /*low area*/
    while(current_pi_level == 0)
    {
        if((step == 0) && (pi_gpio_num == PI_GPIO))
        {
            /*300pps*/
            if(!set_reg_enable)
            {
                MS41939_spi_write(ms41939_spi_fd, 0x29,0x1430);
                MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
            focus_LowStep = focus_LowStep - (48 / 8);
            if(focus_LowStep < 0)
            {
                axms41939_err("%s can not find high level pi \n",__func__);
                set_MS41939_move_status(AXMS41939_FIND_PI_ERR);
                break;
            }
        }

        if((step == 1) && (pi_gpio_num == PI_GPIO))
        {
            /*300PPS*/
            if(set_reg_enable)
            {
                MS41939_spi_write(ms41939_spi_fd, 0x29,0x1518);
                MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 0;
            }
        }

        if((step == 2) && (pi_gpio_num == PI_GPIO))
        {
            /*300PPS*/
            if(!set_reg_enable)
            {
                MS41939_spi_write(ms41939_spi_fd, 0x29,0x1408);
                MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
        }
        axms41939_dbg("step %d \n", step);
        tmp_level = LowArea_pi_level;
        ms41939_gpio_ctrl(vd_gpio_num, 1);
        nanosleep(&delaytime, NULL);
        ms41939_gpio_ctrl(vd_gpio_num, 0);
        usleep(19500);
        LowArea_pi_level = ms41939_get_gpio_values(pi_gpio_num);
        if(LowArea_pi_level != tmp_level)
        {
            if(step == 2)
            {
                axms41939_dbg("%s find pi postion ok! \n",__func__);
                set_MS41939_move_status(AXMS41939_FIND_PI);
                break;
            }
            step++;
        }
        continue;
    }
    return 0;
}

AX_S32 ms41939_actuator_init(AX_U8 pipe, AX_U8 bus_num, AX_U8 cs)
{
    AX_S32 axRet = 0;
    AX_U8 pi_gpio_num;

    axRet = axms41939_init_log(pipe);
    if (0 != axRet) {
        axms41939_err("axms41939 Log Init Failed!\n");
    }

    /*spi init*/
    ms41939_spi_fd = MS41939_spi_init(bus_num, cs);
    if(ms41939_spi_fd < 0)
        axms41939_err("axms41939 spi init Failed!\n");

    /*MS41939 INIT*/
    axRet = MS41939_chip_init(pipe);
    axRet = MS41939_chip_init(pipe);

    /*find pi position*/
    pi_gpio_num = PI_GPIO;
    axRet = ms41939_LenPI_Position_Check(pi_gpio_num, VD_FZ_GPIO);
    MS41939_spi_write(ms41939_spi_fd, 0x29,0x1500);
    MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
    ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);
    focus_previous_position = 0;
    axms41939_dbg("%s finish! cs %d bus_num %d \n",__func__, cs, bus_num);
    return axRet;
}

AX_U8 get_ms41939_rstb_status(AX_U8 nPipeId)
{
    axms41939_dbg("%s ms41939_rstb_flag %d \n",__func__, ms41939_rstb_flag);
    return ms41939_rstb_flag;
}

AX_S32 ms41939_actuator_deinit(AX_U8 pipe)
{
    if(ms41939_spi_fd != 0)
        close(ms41939_spi_fd);
    return 0;
}

AX_S32 ms41939_Ctrl_VCM_Move(AX_U8 nPipeId,AX_S32 pos, AX_U32 pps)
{
    AX_U32 step_value = 0;
    AX_U32 move_direction = 0;
    AX_U32 step_num;
    AX_U32 move_speed;
    AX_S32 tmp;
    AX_U32 vd_num;

    if(focus_previous_position != pos)
    {
        axms41939_dbg("previous_position %d move pos %d \n",
            focus_previous_position, pos);
        tmp = pos - focus_previous_position;
        focus_previous_position = pos;
        if(tmp > 0)
        {
            move_direction = 0x15;/*high to low*/
        } else if(tmp < 0)
        {
            tmp = ~(tmp - 1);
            move_direction = 0x14;/*low to high*/
        }

        step_num = pps * 8;
        step_num = step_num / 50;
        step_value = step_num;
        step_value = (move_direction << 8) | step_value;
        move_speed = 27000000 / (50 * 24 * step_num);

        MS41939_spi_write(ms41939_spi_fd, 0x29,step_value);
        MS41939_spi_write(ms41939_spi_fd, 0x2a,move_speed);
        vd_num = (tmp * 8) / step_num;
        axms41939_dbg("%s step_value %x move_speed %x vd_num %d \n",__func__,
            step_value, move_speed, vd_num);
        for(int i = 0; i < vd_num; i++)
        {
            ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
            nanosleep(&delaytime, NULL);
            ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
            usleep(19500);
        }
        MS41939_spi_write(ms41939_spi_fd, 0x29,0x1500);
        MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
        ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
        set_MS41939_move_status(AXMS41939_MOVE_FINISH);
    }
    axms41939_dbg("%s position is not changed\n", __func__);
    return 0;
}

AX_S32 ms41939_Ctrl_VCM_Move_direction(AX_U8 nPipeId,AX_S32 pos, AX_S32 direction)
{
    AX_U32 step_value = 0;
    AX_U32 move_direction = 0;
    AX_U32 step_num = 0;
    AX_U32 remain_num;
    AX_U32 move_speed;
    AX_U32 vd_num;

    set_MS41939_move_status(AXMS41939_IDLE);
    axms41939_dbg("direction %d move pos %d \n",
        direction, pos);
    if(direction > 0)
    {
        move_direction = 0x15;/*high to low*/
    } else if(direction < 0)
    {
        move_direction = 0x14;/*low to high*/
    }

    step_num = (PPS_VALUE * 8) / 50;//48
    move_speed = 27000000 / (50 * 24 * step_num);
    if((pos * 4) <= step_num)
    {
        vd_num = 1;
        step_num = pos * 4;
        remain_num = 0;
        step_value = (move_direction << 8) | step_num;
        MS41939_spi_write(ms41939_spi_fd, 0x29, step_value);
        MS41939_spi_write(ms41939_spi_fd, 0x2a, move_speed);
    } else {
        vd_num = (pos * 4) / step_num;
        remain_num = (pos * 4 ) - (step_num * vd_num);
        step_value = (move_direction << 8) | step_num;
        MS41939_spi_write(ms41939_spi_fd, 0x29, step_value);
        MS41939_spi_write(ms41939_spi_fd, 0x2a, move_speed);
    }
    axms41939_dbg("%s step_value %x move_speed %x vd_num %d remain_num %d \n",__func__,
        step_value, move_speed, vd_num,remain_num);
    for(int i = 0; i < vd_num; i++)
    {
        ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
        usleep(19500);
    }
    if(remain_num)
    {
        step_value = (move_direction << 8) | remain_num;
        MS41939_spi_write(ms41939_spi_fd, 0x29, step_value);
        MS41939_spi_write(ms41939_spi_fd, 0x2a, move_speed);
        ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
        usleep(19500);
    }
    MS41939_spi_write(ms41939_spi_fd, 0x29,0x1500);
    MS41939_spi_write(ms41939_spi_fd, 0x2a,0x01D4);
    ms41939_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41939_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);
    set_MS41939_move_status(AXMS41939_MOVE_FINISH);
    return 0;
}

AX_LENS_ACTUATOR_AF_FUNC_T gLensAfFocusMs41939Obj = {

    .pfn_af_focus_init                          = ms41939_actuator_init,
    .pfn_af_focus_rstb_status                   = get_ms41939_rstb_status,
    .pfn_af_focus_get_status                    = get_MS41939_move_status,
    .pfn_af_focus_to_dest_pos                   = ms41939_Ctrl_VCM_Move,
    .pfn_af_focus_to_dest_pos_direction         = ms41939_Ctrl_VCM_Move_direction,
    .pfn_af_focus_exit                          = ms41939_actuator_deinit,
    .pfn_af_zoom_init                           = AX_NULL,
    .pfn_af_zoom_rstb_status                    = AX_NULL,
    .pfn_af_zoom1_get_status                    = AX_NULL,
    .pfn_af_zoom2_get_status                    = AX_NULL,
    .pfn_af_zoom1_to_dest_pos                   = AX_NULL,
    .pfn_af_zoom2_to_dest_pos                   = AX_NULL,
    .pfn_af_zoom1_to_dest_pos_direction         = AX_NULL,
    .pfn_af_zoom2_to_dest_pos_direction         = AX_NULL,
    .pfn_af_zoom_exit                           = AX_NULL,

};
