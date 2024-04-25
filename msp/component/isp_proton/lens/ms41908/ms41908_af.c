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
#include "ms41908_af.h"

const char ms41908_g_acName[] = "VcmDriver_MS41908";
#define RSTB_GPIO 79
#define ZOOM1_PI_GPIO 77
#define ZOOM2_PI_GPIO 76
#define HIGH_TO_LOW_DIRECTOR 0x1D00
#define LOW_TO_HIGH_DIRECTOR 0x1C00
#define PPS_VALUE 300
#define CLK_RATE 27
#define VD_HZ 100

AX_S32 ms41908_spi_fd = 0;
AX_S32 zoom1_previous_position = 0;
AX_S32 zoom2_previous_position = 0;
AX_U8 ms41908_rstb_flag = 0;
AX_U8 ms41908_zoom1_move_status = 0;
AX_U8 ms41908_zoom2_move_status = 0;

struct timespec delaytime = {0, 300};

/* private api */
#define axms41908_err(fmt, ...)      axms41908_print(AXMS41908_LOG_ERROR,  "[AXMS41908]: "fmt, ##__VA_ARGS__)
#define axms41908_warn(fmt, ...)     axms41908_print(AXMS41908_LOG_WARN,   "[AXMS41908]: "fmt, ##__VA_ARGS__)
#define axms41908_notice(fmt, ...)   axms41908_print(AXMS41908_LOG_NOTICE, "[AXMS41908]: "fmt, ##__VA_ARGS__)
#define axms41908_info(fmt, ...)     axms41908_print(AXMS41908_LOG_INFO,   "[AXMS41908]: "fmt, ##__VA_ARGS__)
#define axms41908_dbg(fmt, ...)      axms41908_print(AXMS41908_LOG_DBG,    "[AXMS41908]: "fmt, ##__VA_ARGS__)
static int g_validPipeCnt = 0;
static int g_ms41908LogLevel  = AXMS41908_LOG_WARN;
static int g_ms41908LogTarget = AXMS41908_LOG_TARGET_SYSLOG;

extern AX_LENS_ACTUATOR_AF_FUNC_T gLensAfFocusMs41939Obj;

AX_S32 axms41908_print(AX_U32 level, AX_CHAR *format, ...)
{
    AX_S32 r = 0;

    if (level >= g_ms41908LogLevel) {
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
        if (g_ms41908LogTarget == AXMS41908_LOG_TARGET_STDERR) {
            fprintf(stderr, "%s", buf);
        } else if (g_ms41908LogTarget == AXMS41908_LOG_TARGET_SYSLOG) {
            syslog(level, "%s", buf);
        }
    }
    return r;
}

AX_S32 axms41908_init_log(AX_U8 pipe)
{
    if (pipe >= ACTUATOR_MAX_NUM) {
        return -1;
    }

    // Only Init Log Level & Target Only Once
    if (g_validPipeCnt == 1) {
        char *log_level  = getenv("AXMS41908_LOG_level");
        if (log_level) {
            g_ms41908LogLevel  = (AXMS41908LogLevel_t)atoi(log_level);
        } else {
            g_ms41908LogLevel = AXMS41908_LOG_WARN;
        }

        char *log_target = getenv("AXMS41908_LOG_target");
        if (log_target) {
            g_ms41908LogTarget = (AXMS41908LogTarget_t)atoi(log_target);
        } else {
            g_ms41908LogTarget = AXMS41908_LOG_TARGET_SYSLOG;
        }
    }
    return 0;
}

AX_S32 MS41908_spi_init(AX_U8 bus_num, AX_U8 cs)
{
    return SPI_init(bus_num, cs);
}

AX_S32 MS41908_spi_write(AX_S32 spi_fd, AX_U8 reg, AX_U16 data)
{
    AX_S32 ret;
    ret = SPI_write(ms41908_spi_fd, reg, data);
    return ret;
}

AX_S32 ms41908_gpio_ctrl(AX_U8 gpio_num, AX_U8 gpio_out_val)
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
            axms41908_err("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "%d", gpio_num);
        fclose(fp);

        sprintf(file_name, "/sys/class/gpio/gpio%d/direction", gpio_num);
        fp = fopen(file_name, "w");
        if (fp == NULL) {
            axms41908_err("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "out");
        fclose(fp);
    }

    sprintf(file_name, "/sys/class/gpio/gpio%d/value", gpio_num);
    fp = fopen(file_name, "w");
    if (fp == NULL) {
        axms41908_err("Cannot open %s.\n", file_name);
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
        axms41908_err("Cannot open %s.\n", file_name);
        return -1;
    }
    if(fgets(str, 2, fp) != NULL ) {
        pi_level = (AX_U8)atoi(str);
        axms41908_dbg("pi_level %d gpio_num %d \n", pi_level,gpio_num);
    }
    fclose(fp);
    return 0;
}

AX_S32 ms41908_rstb(AX_U8 rstb_gpio)
{
    ms41908_gpio_ctrl(rstb_gpio, 0);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(rstb_gpio, 1);
    ms41908_rstb_flag = 1;

    return 0;
}

AX_S32 MS41908_chip_init(AX_U8 pipe)
{
    //RSTB
    AX_LENS_ACTUATOR_AF_FUNC_T *actuatorObj = &gLensAfFocusMs41939Obj;
    AX_U8 rstb_flag = actuatorObj->pfn_af_focus_rstb_status(pipe);
    if(!rstb_flag)
    {
        ms41908_rstb(RSTB_GPIO);
        axms41908_dbg("%s rstb_flag %d \n",__func__, rstb_flag);
    }
    /*send cmd*/
    MS41908_spi_write(ms41908_spi_fd, 0x00,0x0100);
    MS41908_spi_write(ms41908_spi_fd, 0x01,0x6000);
    MS41908_spi_write(ms41908_spi_fd, 0x02,0x66f0);
    MS41908_spi_write(ms41908_spi_fd, 0x03,0x0e10);
    MS41908_spi_write(ms41908_spi_fd, 0x04,0xd640);
    MS41908_spi_write(ms41908_spi_fd, 0x05,0x0024);
    MS41908_spi_write(ms41908_spi_fd, 0x0B,0x0400);
    MS41908_spi_write(ms41908_spi_fd, 0x0a,0x0000);
    MS41908_spi_write(ms41908_spi_fd, 0x0e,0x0300);

    MS41908_spi_write(ms41908_spi_fd, 0x20,0x1e01);
    MS41908_spi_write(ms41908_spi_fd, 0x21,0x0087);
    MS41908_spi_write(ms41908_spi_fd, 0x0B,0x0080);
    MS41908_spi_write(ms41908_spi_fd, 0x22,0x0001);
    MS41908_spi_write(ms41908_spi_fd, 0x23,0xc8c8);
    MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
    MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
    MS41908_spi_write(ms41908_spi_fd, 0x27,0x0001);
    MS41908_spi_write(ms41908_spi_fd, 0x28,0xc8c8);
    MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
    MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);

    /*vd signal*/
    ms41908_gpio_ctrl(VD_IS_GPIO, 1);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(VD_IS_GPIO, 0);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);
    axms41908_dbg("%s finish\n",__func__);
    return 0;
}

AX_U8 ms41908_get_gpio_values(AX_U8 pi_gpio_num)
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
            axms41908_err("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "%d", pi_gpio_num);
        fclose(fp);

        sprintf(file_name, "/sys/class/gpio/gpio%d/direction", pi_gpio_num);
        fp = fopen(file_name, "w");
        if (fp == NULL) {
            axms41908_err("Cannot open %s.\n", file_name);
            return -1;
        }
        fprintf(fp, "in");
        fclose(fp);
    }
    sprintf(file_name, "/sys/class/gpio/gpio%d/value", pi_gpio_num);
    fp = fopen(file_name, "r");
    if (fp == NULL) {
        axms41908_err("Cannot open %s.\n", file_name);
        return -1;
    }
    if(fgets(str, 2, fp) != NULL ) {
        pi_level = (AX_U8)atoi(str);
        axms41908_dbg("pi_level %d gpio_num %d \n", pi_level, pi_gpio_num);
    }
    fclose(fp);
    return pi_level;
}

AX_U8 get_MS41908_zoom1_move_status(AX_U8 nPipeId)
{
    AX_U8 status;
    status = ms41908_zoom1_move_status;
    return status;
}

AX_S32 set_MS41908_zoom1_move_status(AXMS41908MoveStatus_t status)
{
    ms41908_zoom1_move_status = (AX_U8)status;
    return 0;
}

AX_U8 get_MS41908_zoom2_move_status(AX_U8 nPipeId)
{
    AX_U8 status;
    status = ms41908_zoom2_move_status;
    return status;
}

AX_S32 set_MS41908_zoom2_move_status(AXMS41908MoveStatus_t status)
{
    ms41908_zoom2_move_status = (AX_U8)status;
    return 0;
}

AX_S32 ms41908_LenPI_Position_Check(AX_U8 pi_gpio_num, AX_U8 vd_gpio_num)
{
    AX_U8 current_pi_level;
    AX_U8 HighArea_pi_level;
    AX_U8 LowArea_pi_level;
    AX_S32 zoom1_highStep = 174;
    AX_S32 zoom1_LowStep = 1344;
    AX_S32 zoom2_highStep = 270;
    AX_S32 zoom2_LowStep = 528;
    AX_U8 step = 0;
    AX_U8 set_reg_enable = 0;
    AX_U8 tmp_level = 0;

    current_pi_level = ms41908_get_gpio_values(pi_gpio_num);
    HighArea_pi_level = current_pi_level;
    LowArea_pi_level = current_pi_level;
    axms41908_dbg("pi_gpio_num %d \n", pi_gpio_num);

    /*high area*/
    while(current_pi_level == 1)
    {
        if((step == 0) && (pi_gpio_num == ZOOM1_PI_GPIO))
        {
            /*zoom1 300PPS*/
            if (!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C00);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C30);
                MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
            zoom1_highStep = zoom1_highStep - (48 / 8);
            if(zoom1_highStep < 0)
            {
                axms41908_err("can not find low level pi \n");
                set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_FIND_PI_ERR);
                break;
            }
            axms41908_dbg("zoom1_highStep %d \n",zoom1_highStep);
        } else if ((step == 0) && (pi_gpio_num == ZOOM2_PI_GPIO)) {
            /*zoom2*/
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D30);
                MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1D00);
                set_reg_enable = 1;
            }
            zoom2_highStep = zoom2_highStep - (48 / 8);
            if(zoom2_highStep < 0)
            {
                axms41908_err("can not find low level pi \n");
                set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_FIND_PI_ERR);
                break;
            }
            axms41908_dbg("zoom2_highStep %d \n",zoom2_highStep);
        }

        if((step == 1) && (pi_gpio_num == ZOOM1_PI_GPIO))
        {
            /*zoom1*/
            if(set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1D18);
                MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 0;
            }
        } else if ((step == 1) && (pi_gpio_num == ZOOM2_PI_GPIO)) {
            /*zoom2*/
            if(set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C18);
                MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
                set_reg_enable = 0;
            }
        }

        if((step == 2) && (pi_gpio_num == ZOOM1_PI_GPIO))
        {
            /*zoom1*/
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C00);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C08);
                MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
        } else if((step == 2) && (pi_gpio_num == ZOOM2_PI_GPIO)) {
            /*zoom2*/
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D08);
                MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1D00);
                set_reg_enable = 1;
            }
        }

        axms41908_dbg("step %d pi_gpio %d \n", step, pi_gpio_num);
        tmp_level = HighArea_pi_level;
        ms41908_gpio_ctrl(vd_gpio_num, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(vd_gpio_num, 0);
        usleep(19500);
        HighArea_pi_level = ms41908_get_gpio_values(pi_gpio_num);
        if(HighArea_pi_level != tmp_level)
        {
            if(step == 2)
            {
                if(pi_gpio_num == ZOOM2_PI_GPIO)
                {
                    set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_FIND_PI);
                }
                if(pi_gpio_num == ZOOM1_PI_GPIO)
                {
                    set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_FIND_PI);
                }
                axms41908_dbg("find pi postion ok! \n");
                break;
            }
            step++;
        }
        continue;
    }

    /*low area*/
    while(current_pi_level == 0)
    {
        if((step == 0) && (pi_gpio_num == ZOOM1_PI_GPIO))
        {
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1D30);
                MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
            zoom1_LowStep = zoom1_LowStep - (48 / 8);
            if(zoom1_LowStep < 0)
            {
                axms41908_err("can not find low level pi \n");
                set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_FIND_PI_ERR);
                break;
            }
            axms41908_dbg("zoom1_LowStep %d \n",zoom1_LowStep);
        } else if((step == 0) && (pi_gpio_num == ZOOM2_PI_GPIO))
        {
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C30);
                MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
                set_reg_enable = 1;
            }
            zoom2_LowStep = zoom2_LowStep - (48 / 8);
            if(zoom2_LowStep < 0)
            {
                axms41908_err("can not find low level pi \n");
                set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_FIND_PI_ERR);
                break;
            }
            axms41908_dbg("zoom2_LowStep %d \n",zoom2_LowStep);
        }

        if((step == 1) && (pi_gpio_num == ZOOM1_PI_GPIO))
        {
            /*zoom1*/
            if(set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C00);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C18);
                MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 0;
            }
        } else if((step == 1) && (pi_gpio_num == ZOOM2_PI_GPIO)) {
            /*zoom2*/
            if(set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D18);
                MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1D00);
                set_reg_enable = 0;
            }
        }

        if((step == 2) && (pi_gpio_num == ZOOM1_PI_GPIO))
        {
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1D08);
                MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
                set_reg_enable = 1;
            }
        } else if((step == 2) && (pi_gpio_num == ZOOM2_PI_GPIO)) {
            if(!set_reg_enable)
            {
                MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C08);
                MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
                MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
                set_reg_enable = 1;
            }
        }
        axms41908_dbg("step %d pi_gpio %d \n", step, pi_gpio_num);
        tmp_level = LowArea_pi_level;
        ms41908_gpio_ctrl(vd_gpio_num, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(vd_gpio_num, 0);
        usleep(19500);
        LowArea_pi_level = ms41908_get_gpio_values(pi_gpio_num);
        if(LowArea_pi_level != tmp_level)
        {
            if(step == 2)
            {
                if(pi_gpio_num == ZOOM2_PI_GPIO)
                {
                    set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_FIND_PI);
                }
                if(pi_gpio_num == ZOOM1_PI_GPIO)
                {
                    set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_FIND_PI);
                }
                axms41908_dbg("find pi postion ok! \n");
                break;
            }
            step++;
        }
        continue;
    }
    return 0;
}

AX_S32 ms41908_actuator_init(AX_U8 pipe, AX_U8 bus_num, AX_U8 cs)
{
    AX_S32 axRet = 0;
    AX_U8 pi_gpio_num;
    static AX_BOOL is_init = AX_FALSE;

    if (is_init == AX_TRUE) {
        axms41908_info("init already\n");
        return 0;
    }

    axRet = axms41908_init_log(pipe);
    if (0 != axRet) {
        axms41908_err("axms41908 Log Init Failed!\n");
    }

    /*spi init*/
    ms41908_spi_fd = MS41908_spi_init(bus_num, cs);
    if(ms41908_spi_fd < 0)
        axms41908_err("axms41908 spi init Failed!\n");

    /*MS41908 INIT*/
    axRet = MS41908_chip_init(pipe);

    /*zoom1 find pi position*/
    pi_gpio_num = ZOOM1_PI_GPIO;
    ms41908_LenPI_Position_Check(pi_gpio_num, VD_FZ_GPIO);
    MS41908_spi_write(ms41908_spi_fd, 0x29, 0x1C00);
    MS41908_spi_write(ms41908_spi_fd, 0x2a, 0x01D4);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 0);

    /*zoom2 find pi position*/
    pi_gpio_num = ZOOM2_PI_GPIO;
    ms41908_LenPI_Position_Check(pi_gpio_num, VD_FZ_GPIO);
    MS41908_spi_write(ms41908_spi_fd, 0x24, 0x1D00);
    MS41908_spi_write(ms41908_spi_fd, 0x25, 0x01D4);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
    zoom1_previous_position = 0;
    zoom2_previous_position = 0;
    axms41908_dbg("%s finish\n",__func__);

    is_init = AX_TRUE;
    return axRet;
}

AX_U8 get_ms41908_rstb_status(AX_U8 pipe)
{
    axms41908_dbg("%s ms41908_rstb_flag %d \n",__func__, ms41908_rstb_flag);
    return ms41908_rstb_flag;
}

AX_S32 ms41908_actuator_deinit(AX_U8 nPipeId)
{
    if(ms41908_spi_fd != 0)
        close(ms41908_spi_fd);
    return 0;
}

AX_S32 MS41908_zoom1_move(AX_U8 nPipeId, AX_S32 pos,AX_U32 pps)
{
    AX_U32 step_value = 0;
    AX_U32 moto_direction = 0;
    AX_U32 step_num;
    AX_U32 moto_speed;
    AX_S32 tmp;
    AX_U32 vd_num;

    if(zoom1_previous_position != pos)
    {
        axms41908_dbg(" zoom1_previous_position %d pos %d \n",
            zoom1_previous_position, pos);
        tmp = pos - zoom1_previous_position;
        zoom1_previous_position = pos;
        if(tmp < 0)
        {
            moto_direction = 0x1D;/*low to high*/
            tmp = ~(tmp - 1);
        } else if(tmp > 0)
        {
            moto_direction = 0x1C;/*high to low*/
        }

        step_num = pps * 8;
        step_num = step_num / 50;
        step_value = step_num;
        step_value = (moto_direction << 8) | step_value;
        moto_speed = 27000000 / (50 * 24 * step_num);

        MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C00);
        MS41908_spi_write(ms41908_spi_fd, 0x29,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x2a,moto_speed);
        vd_num = (tmp * 8) / step_num;
        axms41908_dbg("%s step_value %x step_num %d moto_speed %x vd_num %d \n",__func__,
            step_value, step_num, moto_speed, vd_num);
        for(int i = 0; i < vd_num; i++)
        {
            ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
            nanosleep(&delaytime, NULL);
            ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
            usleep(19500);
        }
        MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
        MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
        MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
        MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
        set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_MOVEFINISH);
    }
    axms41908_dbg("%s position is not changed\n", __func__);
    return 0;
}

AX_S32 MS41908_zoom2_move(AX_U8 nPipeId,AX_S32 pos,AX_U32 pps)
{
    AX_U32 step_value = 0;
    AX_U32 moto_direction = 0;
    AX_U32 step_num;
    AX_S32 tmp;
    AX_U32 vd_num;
    AX_U32 moto_speed;

    if(zoom2_previous_position != pos)
    {
        axms41908_dbg("zoom2_previous_position %d pos %d \n",
            zoom2_previous_position, pos);
        tmp = pos - zoom2_previous_position;
        zoom2_previous_position = pos;
        if(tmp < 0)
        {
            moto_direction = 0x1D;/*high to low*/
            tmp = ~(tmp - 1);
        } else if(tmp > 0)
        {
            moto_direction = 0x1C;/*low to high*/
        }
        step_num = pps * 8;
        step_num = step_num / 50;
        step_value = step_num;
        step_value = (moto_direction << 8) | step_value;
        moto_speed = 27000000 / (50 * 24 * step_num);

        MS41908_spi_write(ms41908_spi_fd, 0x24,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x25,moto_speed);
        MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
        vd_num = (tmp * 8) / step_num;
        axms41908_dbg("%s step_value %x step_num %d moto_speed %x vd_num %d\n",__func__,
            step_value, step_num, moto_speed, vd_num);
        for(int i = 0; i < vd_num; i++)
        {
            ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
            nanosleep(&delaytime, NULL);
            ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
            usleep(19500);
        }
        MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
        MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
        MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
        MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
        set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_MOVEFINISH);
    }
    axms41908_dbg("%s position is not changed\n",__func__);
    return 0;
}

AX_S32 MS41908_zoom2_move_direction(AX_U8 nPipeId, AX_S32 pos,AX_S32 direction)
{
    AX_U32 step_value = 0;
    AX_U32 moto_direction = 0;
    AX_U32 step_num;
    AX_U32 vd_num;
    AX_U32 moto_speed;
    AX_U32 remain_num;

    set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_IDLE);
    axms41908_dbg("direction %d pos %d \n",
        direction, pos);
    if(direction < 0)
    {
        moto_direction = 0x1D;/*high to low*/
    } else if(direction > 0)
    {
        moto_direction = 0x1C;/*low to high*/
    }

    step_num = (PPS_VALUE * 8) / 50;
    moto_speed = 27000000 / (50 * 24 * step_num);
    if((pos * 8) <= step_num)
    {
        vd_num = 1;
        step_num = (pos * 8);
        remain_num = 0;
        step_value = (moto_direction << 8) | step_num;
        MS41908_spi_write(ms41908_spi_fd, 0x24,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x25,moto_speed);
        MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
    } else {
        vd_num = (pos * 8) / step_num;
        remain_num = (pos * 8 ) - (step_num * vd_num);
        step_value = (moto_direction << 8) | step_num;
        MS41908_spi_write(ms41908_spi_fd, 0x24,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x25,moto_speed);
        MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
    }
    axms41908_dbg("%s step_value %x step_num %d moto_speed %x vd_num %d remain_num %d \n",__func__,
        step_value, step_num, moto_speed, vd_num, remain_num);
    for(int i = 0; i < vd_num; i++)
    {
        ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
        usleep(19500);
    }
    if(remain_num)
    {
        step_value = (moto_direction << 8) | remain_num;
        MS41908_spi_write(ms41908_spi_fd, 0x24,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x25,moto_speed);
        MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
        usleep(19500);
    }
    MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
    MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
    MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
    MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);
    set_MS41908_zoom2_move_status(AXMS41908_ZOOM2_MOVEFINISH);
    return 0;
}

AX_S32 MS41908_zoom1_move_direction(AX_U8 nPipeId,AX_S32 pos, AX_S32 direction)
{
    AX_U32 step_value = 0;
    AX_U32 moto_direction = 0;
    AX_U32 step_num;
    AX_U32 moto_speed;
    AX_U32 vd_num;
    AX_U32 remain_num;

    set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_IDLE);
    axms41908_dbg(" direction %d pos %d \n",
        direction, pos);
    if(direction < 0)
    {
        moto_direction = 0x1D;/*low to high*/
    } else if(direction > 0)
    {
        moto_direction = 0x1C;/*high to low*/
    }

    step_num = (PPS_VALUE * 8) / 50;
    moto_speed = 27000000 / (50 * 24 * step_num);
    if((pos * 8) <= step_num)
    {
        vd_num = 1;
        step_num = (pos * 8);
        remain_num = 0;
        step_value = (moto_direction << 8) | step_num;
        MS41908_spi_write(ms41908_spi_fd, 0x24, 0x1C00);
        MS41908_spi_write(ms41908_spi_fd, 0x29, step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x2a, moto_speed);
    } else{
        vd_num = (pos * 8) / step_num;
        remain_num = (pos * 8 ) - (step_num * vd_num);
        step_value = (moto_direction << 8) | step_num;
        MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C00);
        MS41908_spi_write(ms41908_spi_fd, 0x29,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x2a,moto_speed);
    }
    axms41908_dbg("%s step_value %x step_num %d moto_speed %x vd_num %d remain_num %d \n",__func__,
        step_value, step_num, moto_speed, vd_num,remain_num);
    for(int i = 0; i < vd_num; i++)
    {
        ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
        usleep(19500);
    }
    if(remain_num)
    {
        step_value = (moto_direction << 8) | remain_num;
        MS41908_spi_write(ms41908_spi_fd, 0x24,0x1C00);
        MS41908_spi_write(ms41908_spi_fd, 0x29,step_value);
        MS41908_spi_write(ms41908_spi_fd, 0x2a,moto_speed);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
        nanosleep(&delaytime, NULL);
        ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
        usleep(19500);
    }
    MS41908_spi_write(ms41908_spi_fd, 0x24,0x1D00);
    MS41908_spi_write(ms41908_spi_fd, 0x25,0x01D4);
    MS41908_spi_write(ms41908_spi_fd, 0x29,0x1C00);
    MS41908_spi_write(ms41908_spi_fd, 0x2a,0x01D4);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 1);
    nanosleep(&delaytime, NULL);
    ms41908_gpio_ctrl(VD_FZ_GPIO, 0);
    usleep(19500);
    set_MS41908_zoom1_move_status(AXMS41908_ZOOM1_MOVEFINISH);
    return 0;
}

AX_LENS_ACTUATOR_AF_FUNC_T gLensAfZoomMs41908Obj = {

    .pfn_af_focus_init                          = AX_NULL,
    .pfn_af_focus_rstb_status                   = AX_NULL,
    .pfn_af_focus_get_status                    = AX_NULL,
    .pfn_af_focus_to_dest_pos                   = AX_NULL,
    .pfn_af_focus_to_dest_pos_direction         = AX_NULL,
    .pfn_af_focus_exit                          = AX_NULL,
    .pfn_af_zoom_init                           = ms41908_actuator_init,
    .pfn_af_zoom_rstb_status                    = get_ms41908_rstb_status,
    .pfn_af_zoom1_get_status                    = get_MS41908_zoom1_move_status,
    .pfn_af_zoom2_get_status                    = get_MS41908_zoom2_move_status,
    .pfn_af_zoom1_to_dest_pos                   = MS41908_zoom1_move,
    .pfn_af_zoom2_to_dest_pos                   = MS41908_zoom2_move,
    .pfn_af_zoom1_to_dest_pos_direction         = MS41908_zoom1_move_direction,
    .pfn_af_zoom2_to_dest_pos_direction         = MS41908_zoom2_move_direction,
    .pfn_af_zoom_exit                           = ms41908_actuator_deinit,

};
