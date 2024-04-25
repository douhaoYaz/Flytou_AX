/*!
 *  @file: mcv_attribute.h
 *  @brief: mcv attribute file
 *  @version: 1.0.0
 *  @author:
 *  @date:
 */
/******************************************************************************
@note
    Copyright 2017, Megvii Corporation, Limited
                        ALL RIGHTS RESERVED
******************************************************************************/

#pragma once

#include "mcv_common_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *MCV_ATTR_HANDLE_T;

typedef struct MCV_ATTR_HANDLE_PARAM_ST {
    char *pipline_config_key;
    uint32_t buffer_depth;    // buffer depth
    uint32_t meta_info_size;
    MCV_META_INFO_S *meta_info;
} MCV_ATTR_HANDLE_PARAM_S;

typedef struct MCV_ATTR_PARAM_ST {
    char *object_category;    // face,body,vehicle,cycle,plate
    uint32_t analyze_attribute_size;
    MCV_ANALYZE_ATTR_E *analyze_attribute;    // target attribute mask
    int reserved[16];                         // reserved
} MCV_ATTR_PARAM_S;

typedef struct MCV_ATTR_FRAME_ST {
    MCV_FRAME_S *frame;
    uint64_t track_id;
    MCV_RECT4f_S rect;    // offset relative to image
    int reserved[16];     // reserved
} MCV_ATTR_FRAME_S;

typedef struct MCV_ATTR_RESULT_ST {
    uint64_t frame_id;
    uint64_t track_id;
    MCV_RET_CODE_E result_code;
    char *object_category;    // face,body,vehicle,cycle,plate
    // feature result
    uint32_t feature_list_size;
    MCV_FEATURE_S *feature_list;
    // other kind result, data transmission
    uint32_t meta_info_size;
    MCV_META_INFO_S *meta_info;
    void *user_data;
    int reserved[16];    // reserved
} MCV_ATTR_RESULT_S;

typedef struct MCV_ATTR_CAPABILITY_ST {
    uint32_t attr_size;
    uint32_t attr_index[MCV_ANALYZE_ATTR_PLATE_ATTRIBUTE];
} MCV_ATTR_CAPABILITY_S;

/**
 * @fn          mcv_creat_attr_handle
 * @brief       create attr handle
 * @param[in]   handle_param //attr handle param
 * @param[out]  handle // attr handle
 * @return      MCV_RET_SUCCESS:success,MCV_RET_FAILURE:failure
 */
MCV_API MCV_RET_CODE_E mcv_create_attr_handle(const MCV_ATTR_HANDLE_PARAM_S *handle_param,
                                              MCV_ATTR_HANDLE_T *handle);

/**
 * @fn          mcv_release_attr_handle
 * @brief       release attr handle
 * @param[in]   handle //attr handle
 * @param[out]  null
 * @return      MCV_RET_SUCCESS:success,MCV_RET_FAILURE:failure
 */
MCV_API MCV_RET_CODE_E mcv_release_attr_handle(MCV_ATTR_HANDLE_T handle);
/**
 * @fn          mcv_send_attr_frame
 * @brief       send attr frame
 * @param[in]   handle //attr handle
 *              param  //attr param
 *              frame //video frame
 * @param[out]  null
 * @return      MCV_RET_SUCCESS:success,MCV_RET_FAILURE:failure,MCV_RET_FRAME_BUFFER_FULL:frame
 * buffer full buffer full
 */
MCV_API MCV_RET_CODE_E mcv_send_attr_frame(MCV_ATTR_HANDLE_T handle, const MCV_ATTR_PARAM_S *param,
                                           const MCV_ATTR_FRAME_S *frame);

/**
 * @fn          mcv_get_attr_result
 * @brief       get algorithm result
 * @param[in]   handle //stream handle
 * @param[out]  algorithm_result //algorithm result
 * @param[in]   timedwait_millisecond // >0:timed_wait;   =0: do not wait;   <0: wait forever
 * @return      MCV_RET_SUCCESS:success,MCV_RET_FAILURE:failure,MCV_RET_EMPTY_RESULT:result is empty
 */
MCV_API MCV_RET_CODE_E mcv_get_attr_result(MCV_ATTR_HANDLE_T handle,
                                           MCV_ATTR_RESULT_S **algorithm_result,
                                           int timedwait_millisecond);

/**
 * @fn          mcv_expand_rect_for_attr
 * @brief       expand rect for attr
 * @param[in]   object_category //object category
 *                     src_rect // src rect
 *                     dst_rect  // dst rect
 *                     max_width //src image max width
 *                     max_height //src image max height
 * @param[out] null
 * @return      MCV_RET_SUCCESS:success,MCV_RET_FAILURE:failure
 */
MCV_API MCV_RET_CODE_E mcv_expand_rect_for_attr(const char *object_category,
                                                const MCV_RECT4f_S *src_rect,
                                                MCV_RECT4f_S *dst_rect,
                                                const unsigned int max_width,
                                                const unsigned int max_height);

/**
 * @fn          mcv_get_attr_capability
 * @brief       get attr capability
 * @param[in]   handle // stream handle
 * @param[out]  attr_capability // attr supported by curr ppl
 * @return      MCV_RET_SUCCESS:success,MCV_RET_FAILURE:failure
 */
MCV_API MCV_RET_CODE_E mcv_get_attr_capability(MCV_ATTR_HANDLE_T handle,
                                               MCV_ATTR_CAPABILITY_S *attr_capability);

#ifdef __cplusplus
}
#endif
