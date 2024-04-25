/**********************************************************************************
 *
 * Copyright (c) 2019-2020 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/

#ifndef _AX_ACTUATOR_STRUCT_H_
#define _AX_ACTUATOR_STRUCT_H_

#include "ax_base_type.h"

typedef AX_S32 ACTUATOR_DEV;

#define ACTUATOR_MAX_NUM 8
typedef struct Actuator_s {
    const AX_CHAR *pszName;

    /* actuator ctrl */
    AX_S32 (*pfn_actuator_init)(AX_U8 pipe);
    AX_S32 (*pfn_actuator_exit)(AX_U8 pipe);
    AX_S32 (*pfn_actuator_act)(AX_U8 pipe, AX_S32 code);
    AX_S32 (*pfn_lendrv_foucs_init)(AX_U8 pipe, AX_U8 bus_num, AX_U8 cs);
    AX_S32 (*pfn_lendrv_zoom_init)(AX_U8 pipe, AX_U8 bus_num, AX_U8 cs);
    AX_S32 (*pfn_lendrv_focusToDestPos)(AX_S32 pos, AX_U32 pps);
    AX_U8  (*pfn_lendrv_focus_rstb_status)(AX_U8 pipe);
    AX_S32 (*pfn_lendrv_zoom1ToDestPos)(AX_S32 pos,AX_U32 pps);
    AX_S32 (*pfn_lendrv_zoom2ToDestPos)(AX_S32 pos,AX_U32 pps);
    AX_U8  (*pfn_lendrv_zoom_rstb_status)(AX_U8 pipe);
    AX_U8  (*pfn_lendrv_focus_get_status)();
    AX_U8  (*pfn_lendrv_zoom1_get_status)();
    AX_U8  (*pfn_lendrv_zoom2_get_status)();
    AX_S32 (*pfn_lendrv_focusToDestPos_Direction)(AX_S32 pos, AX_S32 direction);
    AX_S32 (*pfn_lendrv_zoom2ToDestPos_Direction)(AX_S32 pos, AX_S32 direction);
    AX_S32 (*pfn_lendrv_zoom1ToDestPos_Direction)(AX_S32 pos, AX_S32 direction);
} Actuator_t;

typedef struct ACTUATOR_CTX_S {
    Actuator_t *pActuator;
    ACTUATOR_DEV act_id;
} ACTCTX_OBJ;

extern ACTCTX_OBJ *act_ctx[ACTUATOR_MAX_NUM];

#endif //_AX_ACTUATOR_STRUCT_H_