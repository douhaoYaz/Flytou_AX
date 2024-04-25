/**********************************************************************************
 *
 * Copyright (c) 2019-2022 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdarg.h>
#include "YuvHandler.h"
#include "ax_interpreter_external_api.h"
#include "ax_skel_api.h"
#include "ax_sys_api.h"
#include "ax_vdec_api.h"
#include "ax_venc_api.h"
#include "ax_buffer_tool.h"
#include "skel_log.h"
#include "attrParser.h"
#include "statMgr.h"
#include "frameMgr.h"

#define SAMPLE_SKEL_ALIGN (64)

#ifndef ALIGN_UP
#define ALIGN_UP(x, align) ((((x) + ((align) - 1)) / (align)) * (align))
#endif

#ifndef ALIGN_DOWN
#define ALIGN_DOWN(x, align) (((x) / (align)) * (align))
#endif

#ifndef AX_MAX
#define AX_MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef AX_MIN
#define AX_MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define SKEL_SAMPLE_OUTPUT_BODY_PATH "body"
#define SKEL_SAMPLE_OUTPUT_VEHICLE_PATH "vehicle"
#define SKEL_SAMPLE_OUTPUT_CYCLE_PATH "cycle"
#define SKEL_SAMPLE_OUTPUT_FACE_PATH "face"
#define SKEL_SAMPLE_OUTPUT_PLATE_PATH "plate"
#define SKEL_SAMPLE_OUTPUT_LOG_FILE "output.txt"

AX_VOID LogSaveToFile(FILE *file, const char *fmt, ...) {
    if (file) {
        va_list args;
        char szLog[1024] = {0};

        va_start(args, fmt);

        AX_U32 len = vsnprintf(szLog, sizeof(szLog), (char *)fmt, args);
        if (len < (AX_U32)sizeof(szLog)) {
            szLog[len] = '\0';
        } else {
            szLog[sizeof(szLog) - 1] = '\0';
        }

        fwrite((AX_U8 *)szLog, 1, strlen(szLog), file);

        va_end(args);
    }
}

#define OUTPUT_LOG_SAVE(fmt, ...) LogSaveToFile(fpResultFile, fmt "\n", ##__VA_ARGS__)

// 在验证YUV视频流结果时，需要在视频发送结束时，需要送一些黑帧，以便可以拿到完整的视频推图输出结果。
// 否则视频最后那些帧的推图及属性输出将可能无法获取，
// 所以送的这些黑帧，仅需要推图结果及属性输出即可，无需关注检测结果
#define BLACK_VIDEO_FRAME_COUNT 60    // Send Black Frame after send video complete to get result of full video

#define SKEL_SAMPLE_OBJECT_SIZE 512

struct skeleton {
    int connection[2];
    int left_right_neutral;
};

struct skeleton pairs[] = {{{15, 13}, 0}, {{13, 11}, 0}, {{16, 14}, 0}, {{14, 12}, 0}, {{11, 12}, 0},
                           {{5, 11}, 0},  {{6, 12}, 0},  {{5, 6}, 0},   {{5, 7}, 0},   {{6, 8}, 0},
                           {{7, 9}, 0},   {{8, 10}, 0},  {{1, 2}, 0},   {{0, 1}, 0},   {{0, 2}, 0},
                           {{1, 3}, 0},   {{2, 4}, 0},   {{0, 5}, 0},   {{0, 6}, 0}};

#define DETECT_SKEL_POINT_COUNT 256

typedef struct _AI_Detection_Box_t {
    AX_F32 fX, fY, fW, fH;
} AI_Detection_Box_t;

typedef struct _AI_Detection_Point_t {
    AX_F32 fX, fY;
} AI_Detection_Point_t;

typedef struct _AI_Detection_SkelResult_t {
    AX_U8 nPointNum;
    AI_Detection_Point_t tPoint[DETECT_SKEL_POINT_COUNT];
    AI_Detection_Box_t tBox;
} AI_Detection_SkelResult_t;

static AX_U64 get_tick_count(AX_VOID) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static AX_VOID ShowUsage(AX_VOID) {
    printf("usage: ./%s <options> ...\n", SAMPLE_SKEL_NAME);
    printf("options:\n");
    printf("-i, \tInput File(jpg/yuv)\n");
    printf("-r, \tInput File Resolution(wxh)(yuv: should be input, jpg: allow empty)\n");
    printf("-w, \tWrite result image to new jpg file(path name, default: empty, not write result image)\n");
    printf("-o, \tSave ouput result(path name)\n");
    printf("-m, \tModels deployment path(path name)\n");
    printf("-t, \tRepeat times((unsigned int), default=1)\n");
    printf("-I, \tInterval sending time per frame((unsigned int)ms, default=0)\n");
    printf("-c, \tConfidence(Body)((float: 0-1), default=0)\n");
    printf("-H, \tHuman track size limit((unsigned int), default=3)\n");
    printf("-V, \tVehicle track size limit((unsigned int), default=0)\n");
    printf("-C, \tCylcle track size limit((unsigned int), default=0)\n");
    printf("-j, \tPush jenc buffer size((unsigned int), default=0)\n");
    printf("-d, \tCache list depth((unsigned int), default=1)\n");
    printf("-f, \tSrc framerate for yuv video((unsigned int), default=25)\n");
    printf("-F, \tDst framerate for yuv video((unsigned int), default=25)\n");
    printf("-P, \tPush face pitch filter((unsigned int(0-180)), default=180)\n");
    printf("-Y, \tPush face yaw filter((unsigned int(0-180)), default=180)\n");
    printf("-R, \tPush face roll filter((unsigned int(0-180), default=180)\n");
    printf("-B, \tPush face blur filter((float: 0-1), default=1)\n");
    printf("-U, \tPush panorama enable((unsigned int), default=0)\n");
    printf("-T, \tPush strategy interval time((unsigned int)ms, default=2000)\n");
    printf("-N, \tPush strategy counts((unsigned int), default=1)\n");
    printf("-S, \tPush strategy same frame((unsigned int), default=0(cross frame)\n");
    printf("-u, \tSkel push strategy((unsigned int), default=3)\n"
                "\t\t1: fast push strategy\n"
                "\t\t2: push strategy\n"
                "\t\t3: best push strategy\n");
    printf("-p, \tSkel PPL((unsigned int), default=1)\n"
                "\t\t1: AX_SKEL_PPL_BODY\n"
                "\t\t2: AX_SKEL_PPL_POSE\n"
                "\t\t3: AX_SKEL_PPL_FH\n"
                "\t\t4: AX_SKEL_PPL_HVCFP\n"
                "\t\t5: AX_SKEL_PPL_FACE_FEATURE\n"
                "\t\t6: AX_SKEL_PPL_HVCP\n");
    printf("-v, \tLog level((unsigned int), default=5)\n"
                "\t\t0: LOG_EMERGENCY_LEVEL\n"
                "\t\t1: LOG_ALERT_LEVEL\n"
                "\t\t2: LOG_CRITICAL_LEVEL\n"
                "\t\t3: LOG_ERROR_LEVEL\n"
                "\t\t4: LOG_WARN_LEVEL\n"
                "\t\t5: LOG_NOTICE_LEVEL\n"
                "\t\t6: LOG_INFO_LEVEL\n"
                "\t\t7: LOG_DEBUG_LEVEL\n");
    printf("-h, \tprint this message\n");
}

AX_S32 ParseConfigParam(const AX_SKEL_CONFIG_S *pstConfig) {
    if (pstConfig->nSize > 0 && pstConfig->pstItems) {
        for (size_t i = 0; i < pstConfig->nSize; i++) {
            if (pstConfig->pstItems[i].pstrType && pstConfig->pstItems[i].pstrValue) {
                // cmd: "body_max_target_count", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                if (strcmp(pstConfig->pstItems[i].pstrType, "body_max_target_count") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %d", pstConfig->pstItems[i].pstrType, (AX_U8)pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "vehicle_max_target_count", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "vehicle_max_target_count") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %d", pstConfig->pstItems[i].pstrType, (AX_U8)pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "cycle_max_target_count", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "cycle_max_target_count") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %d", pstConfig->pstItems[i].pstrType, (AX_U8)pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "body_confidence", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "body_confidence") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "face_confidence", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "face_confidence") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "vehicle_confidence", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "vehicle_confidence") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "cycle_confidence", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "cycle_confidence") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "plate_confidence", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "plate_confidence") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "crop_encoder_qpLevel", value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "crop_encoder_qpLevel") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf =
                            (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s: %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "body_min_size",  value_type: AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "body_min_size") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S)) {
                        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *pstConf =
                            (AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s %dx%d", pstConfig->pstItems[i].pstrType, pstConf->nWidth, pstConf->nHeight);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "face_min_size",  value_type: AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "face_min_size") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S)) {
                        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *pstConf =
                            (AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s %dx%d", pstConfig->pstItems[i].pstrType, pstConf->nWidth, pstConf->nHeight);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "vehicle_min_size",  value_type: AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "vehicle_min_size") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S)) {
                        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *pstConf =
                            (AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s %dx%d", pstConfig->pstItems[i].pstrType, pstConf->nWidth, pstConf->nHeight);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "cycle_min_size",  value_type: AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "cycle_min_size") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S)) {
                        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *pstConf =
                            (AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s %dx%d", pstConfig->pstItems[i].pstrType, pstConf->nWidth, pstConf->nHeight);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "plate_min_size",  value_type: AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "plate_min_size") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S)) {
                        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *pstConf =
                            (AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s %dx%d", pstConfig->pstItems[i].pstrType, pstConf->nWidth, pstConf->nHeight);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "detect_roi_polygon",  value_type: AX_SKEL_ROI_POLYGON_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "detect_roi_polygon") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_ROI_POLYGON_CONFIG_S)) {
                        AX_SKEL_ROI_POLYGON_CONFIG_S *pstConf = (AX_SKEL_ROI_POLYGON_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%d]: nPointNum[%d]", pstConfig->pstItems[i].pstrType, pstConf->bEnable, pstConf->nPointNum);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_strategy",  value_type: AX_SKEL_PUSH_STRATEGY_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_strategy") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_PUSH_STRATEGY_S)) {
                        AX_SKEL_PUSH_STRATEGY_S *pstConf = (AX_SKEL_PUSH_STRATEGY_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [mode:%d, times:%d, count:%d, same:%d]", pstConfig->pstItems[i].pstrType, pstConf->ePushMode, pstConf->nIntervalTimes,
                                pstConf->nPushCounts, pstConf->bPushSameFrame);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "body_crop_encoder",  value_type: AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "body_crop_encoder") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *pstConf = (AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%f, %f, %f, %f]", pstConfig->pstItems[i].pstrType,
                                pstConf->fScaleLeft, pstConf->fScaleRight,
                                pstConf->fScaleTop, pstConf->fScaleBottom);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "vehicle_crop_encoder",  value_type: AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "vehicle_crop_encoder") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *pstConf = (AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%f, %f, %f, %f]", pstConfig->pstItems[i].pstrType,
                                pstConf->fScaleLeft, pstConf->fScaleRight,
                                pstConf->fScaleTop, pstConf->fScaleBottom);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "cycle_crop_encoder",  value_type: AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "cycle_crop_encoder") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *pstConf = (AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%f, %f, %f, %f]", pstConfig->pstItems[i].pstrType,
                                pstConf->fScaleLeft, pstConf->fScaleRight,
                                pstConf->fScaleTop, pstConf->fScaleBottom);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "face_crop_encoder",  value_type: AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "face_crop_encoder") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *pstConf = (AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%f, %f, %f, %f]", pstConfig->pstItems[i].pstrType,
                                pstConf->fScaleLeft, pstConf->fScaleRight,
                                pstConf->fScaleTop, pstConf->fScaleBottom);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "plate_crop_encoder",  value_type: AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "plate_crop_encoder") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *pstConf = (AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%f, %f, %f, %f]", pstConfig->pstItems[i].pstrType,
                                pstConf->fScaleLeft, pstConf->fScaleRight,
                                pstConf->fScaleTop, pstConf->fScaleBottom);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "resize_panorama_encoder_config",  value_type: AX_SKEL_RESIZE_CONFIG *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "resize_panorama_encoder_config") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_RESIZE_CONFIG_S)) {
                        AX_SKEL_RESIZE_CONFIG_S *pstConf = (AX_SKEL_RESIZE_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [%f, %f]", pstConfig->pstItems[i].pstrType, pstConf->fW, pstConf->fH);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_panorama",  value_type: AX_SKEL_PUSH_PANORAMA_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_panorama") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_PUSH_PANORAMA_CONFIG_S)) {
                        AX_SKEL_PUSH_PANORAMA_CONFIG_S *pstConf = (AX_SKEL_PUSH_PANORAMA_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [Enable: %d, Quality: %d]", pstConfig->pstItems[i].pstrType,
                                pstConf->bEnable, pstConf->nQuality);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_quality_body",  value_type: AX_SKEL_ATTR_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_quality_body") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S)) {
                        AX_SKEL_ATTR_FILTER_CONFIG_S *pstConf = (AX_SKEL_ATTR_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [Q: %f]", pstConfig->pstItems[i].pstrType, pstConf->stCommonAttrFilterConfig.fQuality);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_quality_vehicle",  value_type: AX_SKEL_ATTR_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_quality_vehicle") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S)) {
                        AX_SKEL_ATTR_FILTER_CONFIG_S *pstConf = (AX_SKEL_ATTR_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [Q: %f]", pstConfig->pstItems[i].pstrType, pstConf->stCommonAttrFilterConfig.fQuality);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_quality_cycle",  value_type: AX_SKEL_ATTR_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_quality_cycle") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S)) {
                        AX_SKEL_ATTR_FILTER_CONFIG_S *pstConf = (AX_SKEL_ATTR_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [Q: %f]", pstConfig->pstItems[i].pstrType, pstConf->stCommonAttrFilterConfig.fQuality);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_quality_face",  value_type: AX_SKEL_ATTR_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_quality_face") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S)) {
                        AX_SKEL_ATTR_FILTER_CONFIG_S *pstConf = (AX_SKEL_ATTR_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [W: %d, H: %d, P: %f, Y: %f, R: %f, B: %f]", pstConfig->pstItems[i].pstrType,
                                pstConf->stFaceAttrFilterConfig.nWidth, pstConf->stFaceAttrFilterConfig.nHeight,
                                pstConf->stFaceAttrFilterConfig.stPoseblur.fPitch, pstConf->stFaceAttrFilterConfig.stPoseblur.fYaw,
                                pstConf->stFaceAttrFilterConfig.stPoseblur.fRoll, pstConf->stFaceAttrFilterConfig.stPoseblur.fBlur);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_quality_plate",  value_type: AX_SKEL_ATTR_FILTER_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_quality_plate") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S)) {
                        AX_SKEL_ATTR_FILTER_CONFIG_S *pstConf = (AX_SKEL_ATTR_FILTER_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s [Q: %f]", pstConfig->pstItems[i].pstrType, pstConf->stCommonAttrFilterConfig.fQuality);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_bind_enable",  value_type: AX_SKEL_COMMON_ENABLE_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_bind_enable") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_ENABLE_CONFIG_S)) {
                        AX_SKEL_COMMON_ENABLE_CONFIG_S *pstConf = (AX_SKEL_COMMON_ENABLE_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s : %d", pstConfig->pstItems[i].pstrType, pstConf->bEnable);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "track_enable",  value_type: AX_SKEL_COMMON_ENABLE_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "track_enable") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_ENABLE_CONFIG_S)) {
                        AX_SKEL_COMMON_ENABLE_CONFIG_S *pstConf = (AX_SKEL_COMMON_ENABLE_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s : %d", pstConfig->pstItems[i].pstrType, pstConf->bEnable);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "push_enable",  value_type: AX_SKEL_COMMON_ENABLE_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "push_enable") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_ENABLE_CONFIG_S)) {
                        AX_SKEL_COMMON_ENABLE_CONFIG_S *pstConf = (AX_SKEL_COMMON_ENABLE_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s : %d", pstConfig->pstItems[i].pstrType, pstConf->bEnable);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                // cmd: "venc_attr_config",  value_type: AX_SKEL_COMMON_THRESHOLD_CONFIG_S *
                else if (strcmp(pstConfig->pstItems[i].pstrType, "venc_attr_config") == 0) {
                    if (pstConfig->pstItems[i].nValueSize == sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S)) {
                        AX_SKEL_COMMON_THRESHOLD_CONFIG_S *pstConf = (AX_SKEL_COMMON_THRESHOLD_CONFIG_S *)pstConfig->pstItems[i].pstrValue;
                        ALOGI("SKEL get %s : %f", pstConfig->pstItems[i].pstrType, pstConf->fValue);
                    } else {
                        ALOGE("SKEL %s size(%d) no match", pstConfig->pstItems[i].pstrType, pstConfig->pstItems[i].nValueSize);
                    }
                }
                else {
                    ALOGE("SKEL cmd: %s not support", pstConfig->pstItems[i].pstrType);
                }
            }
        }
    }

    return 0;
}

FILE* LoadFile(const AX_CHAR *pFile, AX_U64 *pLen)
{
    /* Reading input file */
    FILE *f_in = fopen(pFile, "rb");

    if (f_in) {
        if (pLen) {
            fseeko(f_in, 0L, SEEK_END);
            AX_U64 nFileSize = ftello(f_in);
            rewind(f_in);

            *pLen = (AX_U64)nFileSize;
        }
    }

    return f_in;
}

int ReLoadFile(FILE* pFile)
{
    if (pFile) {
        rewind(pFile);
        return 0;
    }

    return -1;
}

int UnLoadFile(FILE* pFile)
{
    if (pFile) {
        fclose(pFile);
        return 0;
    }

    return -1;
}

int LoadFileToMem(FILE *pFile, AX_U64 nPhyAddr, AX_VOID *pVirAddr, AX_S32 nSize)
{
    AX_S32 nReadSize = 0;

    if (pFile) {
        if (nPhyAddr != 0 && pVirAddr) {
            memset((AX_U8 *)pVirAddr, 0x00, nSize);
            nReadSize = fread((AX_U8 *)pVirAddr, 1, nSize, pFile);
        }
        else {
            fseeko(pFile, nSize, SEEK_CUR);
        }
    }

    return nReadSize;
}

int LoadFileToMemExt(const AX_CHAR *pFile, AX_U64 *pPhyAddr, AX_VOID **ppVirAddr, AX_U64 *pLen)
{
    /* Reading input file */
    FILE *f_in = fopen(pFile, "rb");
    if (f_in == NULL) {
        ALOGE("Unable to open input file\n");
        return -1;
    }

    /* file i/o pointer to full */
    fseeko(f_in, 0L, SEEK_END);
    AX_U64 nFileSize = ftello(f_in);
    rewind(f_in);

    if (pPhyAddr && ppVirAddr) {
        AX_U64 nPhyAddr = 0;
        AX_VOID *pVirAddr = NULL;

        AX_S32 nRet = AX_SYS_MemAlloc(&nPhyAddr, (AX_VOID **)&pVirAddr, nFileSize, 256, (AX_S8 *)"SKEL_TEST");

        if (nRet != 0) {
            fclose(f_in);
            ALOGE("AX_SYS_MemAlloc failed");
            return -1;
        }

        fread((AX_U8 *)pVirAddr, 1, nFileSize, f_in);

        *pPhyAddr = nPhyAddr;
        *ppVirAddr = pVirAddr;
    }

    if (pLen) {
        *pLen = nFileSize;
    }

    fclose(f_in);

    return 0;
}

int NV12ToStrideNV12(AX_U64 nSrcPhyAddr,
                        AX_VOID *pSrcVirAddr,
                        AX_U32 nSrcSize,
                        AX_U32 nSrcStride,
                        AX_U32 nSrcHeight,
                        AX_U64 nDstPhyAddr,
                        AX_VOID *pDstVirAddr,
                        AX_U32 nDstSize,
                        AX_U32 nDstStride,
                        AX_U32 nDstHeight
                        ) {
    AX_U64 nStartTime = get_tick_count();

    // src check
    if (!pSrcVirAddr) {
        return -1;
    }

    if (nSrcSize != nSrcStride * nSrcHeight * 3 /2) {
        return -1;
    }

    // dst check
    if (!pDstVirAddr) {
        return -1;
    }

    if (nDstSize != nDstStride * nDstHeight * 3 /2) {
        return -1;
    }

    if (nDstSize < nSrcSize) {
        return -1;
    }

    if (nDstStride < nSrcStride) {
        return -1;
    }

    if (nDstHeight != nSrcHeight) {
        return -1;
    }

    AX_VOID *src = pSrcVirAddr;
    AX_VOID *dst = pDstVirAddr;
    for(AX_U32 i = 0; i < nSrcHeight * 3 / 2; i++) {
        memcpy(dst, src, nSrcStride);
        src += nSrcStride;
        dst += nDstStride;
    }

    AX_U64 nProcessElasped = get_tick_count() - nStartTime;

    ALOGD("NV12ToStrideNV12 %dx%d=>%dx%d elapse: %lld ms", nSrcStride, nSrcHeight, nDstStride, nDstHeight, nProcessElasped);

    return 0;
}

AX_S32 FramePoolInit(AX_VDEC_GRP VdGrp, AX_U32 FrameSize, AX_POOL *PoolId)
{
    AX_S32 s32Ret = AX_SUCCESS;
    /* vdec use pool to alloc output buffer */
    AX_POOL_CONFIG_T stPoolConfig = {0};
    AX_POOL s32PoolId;

    memset(&stPoolConfig, 0, sizeof(AX_POOL_CONFIG_T));
    stPoolConfig.MetaSize = 512;
    stPoolConfig.BlkCnt = 1;
    stPoolConfig.BlkSize = FrameSize;
    stPoolConfig.CacheMode = POOL_CACHE_MODE_NONCACHE;
    memset(stPoolConfig.PartitionName, 0, sizeof(stPoolConfig.PartitionName));
    strcpy((AX_CHAR *)stPoolConfig.PartitionName, "anonymous");

    s32PoolId = AX_POOL_CreatePool(&stPoolConfig);
    if (AX_INVALID_POOLID == s32PoolId) {
        ALOGE("Create pool err.");
        return AX_ERR_VDEC_NULL_PTR;
    }

    *PoolId = s32PoolId;

    s32Ret = AX_VDEC_AttachPool(VdGrp, s32PoolId);
    if (s32Ret != AX_SUCCESS) {
        AX_POOL_MarkDestroyPool(s32PoolId);
        ALOGE("Attach pool err. %x", s32Ret);
    }

    return s32Ret;
}

int JpegDecodeOneFrame(const AX_CHAR *inputFile, AX_U32 *nStride, AX_U32 *nWidth, AX_U32 *nHeight, AX_U64 *pPhyAddr, AX_VOID **ppVirAddr, AX_U64 *pLen)
{
    AX_U64 nFileSize = 0;
    AX_U64 streamPhyAddr = 0;
    AX_VOID *pStreamVirAddr = NULL;
    AX_U64 outPhyAddrDst = 0;
    AX_VOID *poutVirAddrDst = NULL;
    AX_S32 widthAlign = 3840;
    AX_S32 heightAlign = 2160;
    AX_U32 noutDstSize = widthAlign * heightAlign * 3 / 2;

    AX_S32 nRet = LoadFileToMemExt(inputFile, &streamPhyAddr, &pStreamVirAddr, &nFileSize);

    if (nRet != 0
        || streamPhyAddr == 0
        || pStreamVirAddr == NULL) {
        ALOGE("Load input file fail");
        return -1;
    }

    AX_VDEC_GRP VdecGrp = 0;
    AX_VDEC_GRP_ATTR_S GrpAttr = {
        .enType = PT_JPEG,
        .enMode = VIDEO_MODE_FRAME,
        .u32PicWidth = 4096,
        .u32PicHeight = 4096,
        .u32StreamBufSize = 10 * 1024 * 1024,
        .u32FrameBufSize = 10 * 1024 * 1024,
        .u32FrameBufCnt = 1,
    };

    nRet = AX_VDEC_CreateGrp(VdecGrp, &GrpAttr);

    if (nRet != AX_SUCCESS) {
        ALOGE("AX_VDEC_CreateChn error: %d", nRet);
        return -1;
    }

    /* jdec used pool memory,for 3840*2160 */
    AX_S32 nFrameSize = AX_VDEC_GetPicBufferSize(3840, 2160, PT_JPEG);

    AX_POOL PoolId = 0;
    nRet = FramePoolInit(VdecGrp, nFrameSize, &PoolId);

    if (nRet != AX_SUCCESS) {
        ALOGE("AX_VDEC_StartRecvStream error: %d", nRet);
        AX_VDEC_DestroyGrp(VdecGrp);
        return -1;
    }

    nRet = AX_VDEC_StartRecvStream(VdecGrp);
    if (nRet != AX_SUCCESS) {
        ALOGE("AX_VDEC_StartRecvStream error: %d", nRet);
        AX_VDEC_DestroyGrp(VdecGrp);
        return -1;
    }

    AX_VDEC_STREAM_S stream;
    memset(&stream, 0x00, sizeof(stream));

    stream.u32Len = (AX_U32)nFileSize;
    stream.pu8Addr = (AX_U8 *)pStreamVirAddr;
    stream.bEndOfStream = AX_FALSE;
    AX_VDEC_SendStream(VdecGrp, &stream, -1);

    AX_VIDEO_FRAME_INFO_S frameInfo;
    memset(&frameInfo, 0x00, sizeof(frameInfo));

    nRet = AX_VDEC_GetFrame(VdecGrp, &frameInfo, -1);

    if (frameInfo.stVFrame.enImgFormat != AX_YUV420_SEMIPLANAR) {
        ALOGE("unsupport jpeg format: %d", frameInfo.stVFrame.enImgFormat);
        AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);
        goto EXIT;
    }

    // poutVirAddrDst = (AX_VOID *)malloc(noutDstSize);
    nRet = AX_SYS_MemAlloc(&outPhyAddrDst, (AX_VOID **)&poutVirAddrDst, noutDstSize, 256, (AX_S8 *)"SKEL_TEST");

    if (nRet != 0) {
        ALOGE("AX_SYS_MemAlloc failed");
        AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);
    }
    else {
        AX_VOID *p_lu = NULL;
        AX_VOID *p_ch = NULL;
        AX_VOID *pLumaVirAddr = NULL;
        AX_VOID *pChromaVirAddr = NULL;
        AX_U32 lumaMapSize = 0;
        AX_U32 chromaMapSize = 0;

        if (nStride) {
            *nStride = ALIGN_UP(frameInfo.stVFrame.u32PicStride[0], SAMPLE_SKEL_ALIGN);
        }

        if (nWidth) {
            *nWidth = frameInfo.stVFrame.u32Width;
        }

        if (nHeight) {
            *(nHeight) = frameInfo.stVFrame.u32Height;
        }

        lumaMapSize = frameInfo.stVFrame.u32PicStride[0] * ALIGN_UP(frameInfo.stVFrame.u32Height, 16);
        pLumaVirAddr = AX_SYS_Mmap(frameInfo.stVFrame.u64PhyAddr[0], lumaMapSize);

        if (!pLumaVirAddr) {
            ALOGE("AX_SYS_Mmap luma failed");
            AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);
            nRet = -1;
            goto EXIT;
        }

        chromaMapSize = frameInfo.stVFrame.u32PicStride[0] * ALIGN_UP(frameInfo.stVFrame.u32Height, 16) / 2;
        pChromaVirAddr = AX_SYS_Mmap(frameInfo.stVFrame.u64PhyAddr[1], chromaMapSize);

        if (!pChromaVirAddr) {
            ALOGE("AX_SYS_Mmap chroma failed");
            AX_SYS_Munmap(pLumaVirAddr, lumaMapSize);
            AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);
            nRet = -1;
            goto EXIT;
        }

        if (noutDstSize < lumaMapSize + chromaMapSize) {
            ALOGE("jpg resolution error: exceed 4096x4096");
            AX_SYS_Munmap(pLumaVirAddr, lumaMapSize);
            AX_SYS_Munmap(pChromaVirAddr, chromaMapSize);
            AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);
            nRet = -1;
            goto EXIT;
        }

        p_lu = pLumaVirAddr;
        p_ch = pChromaVirAddr;

        AX_U32 coded_width = frameInfo.stVFrame.u32Width;
        AX_U32 coded_height = frameInfo.stVFrame.u32Height;
        AX_U32 pic_stride = frameInfo.stVFrame.u32PicStride[0];
        AX_U32 coded_width_ch = frameInfo.stVFrame.u32Width;
        AX_U32 coded_h_ch = frameInfo.stVFrame.u32Height / 2;
        AX_U32 pic_stride_ch = frameInfo.stVFrame.u32PicStride[1];

        AX_U32 nIndex = 0;
        for (size_t i = 0; i < coded_height; i++) {
            memcpy(poutVirAddrDst + nIndex, p_lu, coded_width);
            p_lu += pic_stride;
            nIndex += ALIGN_UP(pic_stride, SAMPLE_SKEL_ALIGN);
        }

        for (size_t  i = 0; i < coded_h_ch; i++) {
            memcpy(poutVirAddrDst + nIndex, p_ch, coded_width_ch);
            p_ch += pic_stride_ch;
            nIndex += ALIGN_UP(pic_stride_ch, SAMPLE_SKEL_ALIGN);
        }

        if (pLen) {
            *pLen = (AX_U64)nIndex;
        }

        AX_SYS_Munmap(pLumaVirAddr, lumaMapSize);
        AX_SYS_Munmap(pChromaVirAddr, chromaMapSize);
        AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);

        stream.u32Len = 0;
        stream.pu8Addr = NULL;
        stream.bEndOfStream = AX_TRUE;
        AX_VDEC_SendStream(VdecGrp, &stream, -1);

        AX_VDEC_GetFrame(VdecGrp, &frameInfo, -1);

        AX_VDEC_ReleaseFrame(VdecGrp, &frameInfo);

        nRet = 0;
    }

EXIT:
    if (nRet != 0) {
        if (outPhyAddrDst != 0) {
            AX_SYS_MemFree(outPhyAddrDst, poutVirAddrDst);
        }
    }
    else {
        *pPhyAddr = outPhyAddrDst;
        *ppVirAddr = poutVirAddrDst;
    }

    AX_VDEC_StopRecvStream(VdecGrp);
    AX_VDEC_DetachPool(VdecGrp);
    AX_VDEC_DestroyGrp(VdecGrp);
    AX_POOL_MarkDestroyPool(PoolId);

    return nRet;
}

int EncodeOneFrameToJpeg(const AX_CHAR *dstFile, AX_U32 nStride, AX_U32 nWidth, AX_U32 nHeight, AX_U64 nPhyAddr, AX_VOID *pVirAddr, AX_U32 nLen) {
    AX_BOOL bRet = AX_FALSE;
    AX_BOOL bJencChnCreate = AX_FALSE;
    const AX_U8 nJencChn = 15;
    AX_S32 ret = AX_SUCCESS;
    AX_VIDEO_FRAME_INFO_S tFrame = {0};
    AX_U32 u32Width = 0;
    AX_U32 u32Height = 0;
    AX_VENC_RECV_PIC_PARAM_S tRecvParam = {0};
    AX_BOOL bJencStreamGet = AX_FALSE;

    AX_VENC_CHN_ATTR_S stVencChnAttr;
    memset(&stVencChnAttr, 0, sizeof(AX_VENC_CHN_ATTR_S));

    tFrame.stVFrame.u32Width = nWidth;
    tFrame.stVFrame.u32Height = nHeight;

    u32Width = tFrame.stVFrame.u32Width;
    u32Height = tFrame.stVFrame.u32Height;

    stVencChnAttr.stVencAttr.u32MaxPicWidth = MAX_JENC_PIC_WIDTH;
    stVencChnAttr.stVencAttr.u32MaxPicHeight = MAX_JENC_PIC_HEIGHT;

    stVencChnAttr.stVencAttr.u32PicWidthSrc = u32Width;
    stVencChnAttr.stVencAttr.u32PicHeightSrc = u32Height;

    stVencChnAttr.stVencAttr.u32CropOffsetX = 0;
    stVencChnAttr.stVencAttr.u32CropOffsetY = 0;
    stVencChnAttr.stVencAttr.u32CropWidth = u32Width;
    stVencChnAttr.stVencAttr.u32CropHeight = u32Height;
    stVencChnAttr.stVencAttr.u32BufSize = u32Width * u32Height * 3 / 4; /*stream buffer size*/

    stVencChnAttr.stVencAttr.enLinkMode = AX_NONLINK_MODE;

    stVencChnAttr.stVencAttr.enType = PT_JPEG;

    ret = AX_VENC_CreateChn(nJencChn, &stVencChnAttr);
    if (AX_SUCCESS != ret) {
        ALOGE("[%d] AX_VENC_CreateChn(%d X %d, size=%d) fail, ret=0x%x", nJencChn, u32Width, u32Height, stVencChnAttr.stVencAttr.u32BufSize, ret);
        goto JENC_EXIT;
    }

    bJencChnCreate = AX_TRUE;

    AX_VENC_JPEG_PARAM_S stJpegParam;
    memset(&stJpegParam, 0, sizeof(AX_VENC_JPEG_PARAM_S));
    ret = AX_VENC_GetJpegParam(nJencChn, &stJpegParam);
    if (AX_SUCCESS != ret) {
        ALOGE("[%d] AX_VENC_GetJpegParam fail, %x", nJencChn, ret);
        goto JENC_EXIT;
    }

    stJpegParam.u32Qfactor = 90;

    ret = AX_VENC_SetJpegParam(nJencChn, &stJpegParam);
    if (AX_SUCCESS != ret) {
        ALOGE("[%d] AX_VENC_SetJpegParam fail, %x", nJencChn, ret);
        goto JENC_EXIT;
    }

    AX_VENC_StartRecvFrame(nJencChn, &tRecvParam);

    tFrame.stVFrame.u32PicStride[0] = nStride;
    tFrame.stVFrame.u32PicStride[1] = tFrame.stVFrame.u32PicStride[0];
    tFrame.stVFrame.u32PicStride[2] = 0;
    tFrame.stVFrame.u64PhyAddr[0] = nPhyAddr;
    tFrame.stVFrame.u64PhyAddr[1] = tFrame.stVFrame.u64PhyAddr[0] + tFrame.stVFrame.u32PicStride[0] * tFrame.stVFrame.u32Height;
    tFrame.stVFrame.u32FrameSize = nLen;
    tFrame.stVFrame.enImgFormat = AX_YUV420_SEMIPLANAR;
    ret = AX_VENC_SendFrame(nJencChn, &tFrame, -1);

    if (AX_SUCCESS != ret) {
        ALOGE("[%d] AX_VENC_SendFrame fail, ret=0x%x", nJencChn, ret);
        goto JENC_EXIT;
    }

    AX_VENC_STREAM_S stVencStream;
    memset(&stVencStream, 0x00, sizeof(stVencStream));
    ret = AX_VENC_GetStream(nJencChn, &stVencStream, -1);
    if (AX_SUCCESS != ret) {
        ALOGE("AX_VENC_GetStream fail, ret=0x%x", ret);
        goto JENC_EXIT;
    }

    bJencStreamGet = AX_TRUE;

    bRet = AX_TRUE;

JENC_EXIT:
    if (bRet && dstFile) {
        FILE *fp_w = fopen(dstFile, "wb");

        if (fp_w) {
            ALOGN("\tWrite new JPG result image to file: %s", dstFile);
            fwrite((AX_U8 *)stVencStream.stPack.pu8Addr, stVencStream.stPack.u32Len, 1, fp_w);
            fclose(fp_w);
        }
    }

    if (bJencStreamGet) {
        AX_VENC_ReleaseStream(nJencChn, &stVencStream);
    }

    if (bJencChnCreate) {
        AX_VENC_StopRecvFrame(nJencChn);
        AX_VENC_DestroyChn(nJencChn);
    }

    return bRet;
}

AX_BOOL FrameSkipCtrl(AX_S32 nSrcFrameRate, AX_S32 nDstFrameRate, AX_S32 nFrameSeqNum) {
    if (nFrameSeqNum < 1) {
        nFrameSeqNum = 1;
    }

    if (nSrcFrameRate == nDstFrameRate) {
        return AX_FALSE;
    }

    if (nDstFrameRate > nSrcFrameRate) {
        return AX_FALSE;
    }

    if ((nFrameSeqNum * nDstFrameRate / (nSrcFrameRate)) > ((nFrameSeqNum - 1) * nDstFrameRate / (nSrcFrameRate))) {
        return AX_FALSE;
    } else {
        return AX_TRUE;
    }
}

AX_S32 main(AX_S32 argc, AX_CHAR *argv[]) {
    AX_S32 nRet = 0;
    AX_S32 c;
    AX_S32 isExit = 0;
    FILE *InputFileHandle = NULL;
    const AX_CHAR *InputFile = NULL;
    const AX_CHAR *InputResolution = NULL;
    AX_S32 InputFileFrameCnt = 1;
    const AX_CHAR *ReWritePath = NULL;
    const AX_CHAR *SaveResultPath = NULL;
    const AX_CHAR *ModelsPath = NULL;
    FILE *fpResultFile = NULL;
    AX_S32 nRepeatTimes = 1;
    AX_S32 nInterval = 0;
    AX_S32 nPPL = 1;
    AX_S32 nPushStrategy = 3;
    AX_S32 nPushStrategyInterval = 2000;
    AX_S32 nPushStrategyCount = 1;
    AX_S32 nPushStrategySameFrame = 0;
    AX_S32 nPushPanora = 0;
    AX_U32 nStride = 1920;
    AX_U32 nWidth = 1920;
    AX_U32 nHeight = 1080;
    AX_U32 nJencBufSize = 0;
    AX_U32 nFrameDepth = 1;
    AX_U32 nCacheListDepth = 1;
    AX_S32 nSrcFrameRate = 25;
    AX_S32 nDstFrameRate = 25;
    AX_F32 fPushFacePitchfilter = 180;
    AX_F32 fPushFaceYawfilter = 180;
    AX_F32 fPushFaceRollfilter = 180;
    AX_F32 fPushFaceBlurfilter = 1.0;
    AX_VOID *OneYUVDataVir = NULL;
    AX_U64 OneYUVDataPhy = 0;
    AX_U32 nOneSize = 0;
    AX_VOID *YUVDataVir = NULL;
    AX_U64 YUVDataPhy = 0;
    AX_VOID *YUVDataWrVir = NULL;
    AX_U64 YUVDataWrPhy = 0;
    AX_SKEL_HANDLE pHandle = NULL;
    AX_SKEL_FRAME_S stFrame = {0};
    AX_SKEL_RESULT_S *pstResult = NULL;
    AX_U64 nFileSize = 0;
    AX_U32 nFrameSize = nStride * nHeight * 3 / 2;
    AX_U64 nStartTime = 0;
    AX_U64 nInitElasped = 0;
    AX_U64 nCreateElasped = 0;
    AX_U64 nProcessElasped = 0;
    AX_U64 nResultElasped = 0;
    AX_U64 nResultElaspedMin = (AX_U64)-1;
    AX_U64 nResultElaspedMax = 0;
    AX_U64 nResultElaspedTotal = 0;
    AX_F32 fConfidence = 0.0;
    AX_U32 nHumantracksize = 3;
    AX_U32 nVehicletracksize = 0;
    AX_U32 nCycletracksize = 0;
    AX_BOOL bJpgFile = AX_FALSE;
    AX_U64 nSkelFrameId = 1;

#if defined(SAMPLE_SKEL_BUILD_VERSION)
    printf("SKEL sample: %s build: %s %s\n", SAMPLE_SKEL_BUILD_VERSION, __DATE__, __TIME__);
#endif

    while ((c = getopt(argc, argv, "i:r:I:w:o:m:t:u:T:N:S:U:p:v:c:j:d:f:F:P:Y:R:B:H:V:C:h::")) != -1) {
        isExit = 0;
        switch (c) {
            case 'i':
                InputFile = (const AX_CHAR *)optarg;
                break;
            case 'r':
                InputResolution = (const AX_CHAR *)optarg;
                break;
            case 'I':
                nInterval = atoi(optarg);
                break;
            case 'w':
                ReWritePath = (const AX_CHAR *)optarg;
                break;
            case 'o':
                SaveResultPath = (const AX_CHAR *)optarg;
                break;
            case 'm':
                ModelsPath = (const AX_CHAR *)optarg;
                break;
            case 't':
                nRepeatTimes = atoi(optarg);
                break;
            case 'u':
                nPushStrategy = atoi(optarg);
                break;
            case 'T':
                nPushStrategyInterval = atoi(optarg);
                break;
            case 'N':
                nPushStrategyCount = atoi(optarg);
                break;
            case 'S':
                nPushStrategySameFrame = atoi(optarg);
                break;
            case 'U':
                nPushPanora = atoi(optarg);
                break;
            case 'p':
                nPPL = atoi(optarg);
                break;
            case 'j':
                nJencBufSize = atoi(optarg);
                break;
            case 'd':
                nCacheListDepth = atoi(optarg);
                break;
            case 'f':
                nSrcFrameRate = atoi(optarg);
                break;
            case 'F':
                nDstFrameRate = atoi(optarg);
                break;
            case 'P':
                fPushFacePitchfilter = atof(optarg);
                break;
            case 'Y':
                fPushFaceYawfilter = atof(optarg);
                break;
            case 'R':
                fPushFaceRollfilter = atof(optarg);
                break;
            case 'B':
                fPushFaceBlurfilter = atof(optarg);
                break;
            case 'c':
                fConfidence = atof(optarg);
                break;
            case 'H':
                nHumantracksize = atoi(optarg);
                break;
            case 'V':
                nVehicletracksize = atoi(optarg);
                break;
            case 'C':
                nCycletracksize = atoi(optarg);
                break;
            case 'v':
                log_level = atoi(optarg);
                break;
            case 'h':
                isExit = 1;
                break;
            default:
                isExit = 1;
                break;
        }
    }

    if (InputFile) {
        AX_U32 nInputFileLen = strlen(InputFile);

        if (nInputFileLen > 4 && (strcasecmp(&InputFile[nInputFileLen - 4], ".jpg") == 0)) {
            bJpgFile = AX_TRUE;
        }
    }

    if (isExit || !InputFile || (!bJpgFile  && !InputResolution) || (nPPL < AX_SKEL_PPL_BODY || nPPL >= (AX_SKEL_PPL_MAX))
        || (log_level < 0 || log_level >= SKEL_LOG_MAX)
        || (fConfidence < 0 || fConfidence > 1)
        || (nPushStrategy < 1 || nPushStrategy > 3)) {
        ShowUsage();
        exit(0);
    }

    if (nRepeatTimes <= 0) {
        nRepeatTimes = 1;
    }

    if (nInterval < 0) {
        nInterval = 0;
    }

    if (nCacheListDepth == 0) {
        nCacheListDepth = 1;
    }

    if (nPushStrategyCount == 0) {
        nPushStrategyCount = 1;
    }

    nInterval = nInterval * 1000;

    if (access(InputFile, 0) != 0) {
        ALOGE("%s not exist", InputFile);
        exit(0);
    }

    // clear output
    if (SaveResultPath) {
        AX_CHAR szPath[512] = {0};
        snprintf(szPath, 511, "rm %s -rf", SaveResultPath);
        system(szPath);

        // create path
        snprintf(szPath, 511, "mkdir %s -p", SaveResultPath);
        system(szPath);

        snprintf(szPath, 511, "%s/%s", SaveResultPath, SKEL_SAMPLE_OUTPUT_BODY_PATH);
        mkdir(szPath, S_IRWXU | S_IRWXG | S_IRWXO);

        snprintf(szPath, 511, "%s/%s", SaveResultPath, SKEL_SAMPLE_OUTPUT_VEHICLE_PATH);
        mkdir(szPath, S_IRWXU | S_IRWXG | S_IRWXO);

        snprintf(szPath, 511, "%s/%s", SaveResultPath, SKEL_SAMPLE_OUTPUT_CYCLE_PATH);
        mkdir(szPath, S_IRWXU | S_IRWXG | S_IRWXO);

        snprintf(szPath, 511, "%s/%s", SaveResultPath, SKEL_SAMPLE_OUTPUT_FACE_PATH);
        mkdir(szPath, S_IRWXU | S_IRWXG | S_IRWXO);

        snprintf(szPath, 511, "%s/%s", SaveResultPath, SKEL_SAMPLE_OUTPUT_PLATE_PATH);
        mkdir(szPath, S_IRWXU | S_IRWXG | S_IRWXO);
    }

    if (ReWritePath) {
        AX_CHAR szPath[512] = {0};
        snprintf(szPath, 511, "rm %s -rf", ReWritePath);
        system(szPath);

        // create path
        snprintf(szPath, 511, "mkdir %s -p", ReWritePath);
        system(szPath);
    }

    if (InputResolution) {
        AX_CHAR *temp_p = strstr(InputResolution, "x");

        if (!temp_p || strlen(temp_p) <= 1) {
            ShowUsage();
            exit(0);
        }

        nWidth = atoi(InputResolution);
        nStride = ALIGN_UP(nWidth, SAMPLE_SKEL_ALIGN);
        nHeight = atoi(temp_p + 1);
    }

    nRet = AX_SYS_Init();
    if (0 != nRet) {
        ALOGE("AX_SYS_Init() fail, ret = 0x%x", nRet);
        exit(0);
    }

    AX_VDEC_Init();

    AX_VENC_MOD_ATTR_S stModAttr = {.enVencType = VENC_MULTI_ENCODER};
    AX_VENC_Init(&stModAttr);

    AX_NPU_SDK_EX_ATTR_T npuAttr;
    npuAttr.eHardMode = AX_NPU_VIRTUAL_1_1;
    nRet = AX_NPU_SDK_EX_Init_with_attr(&npuAttr);
    if (0 != nRet) {
        ALOGE("AX_NPU_SDK_EX_Init_with_attr() fail, ret = 0x%x", nRet);
        goto EXIT0;
    }

    if (bJpgFile) {
        JpegDecodeOneFrame(InputFile, &nStride, &nWidth, &nHeight, &YUVDataPhy, &YUVDataVir, &nFileSize);

        if (nRet != 0
            || YUVDataPhy == 0
            || YUVDataVir == NULL) {
            ALOGE("Load input file fail");
            goto EXIT1;
        }

        nOneSize = nWidth * nHeight * 3 / 2;
        nFrameSize = nStride * nHeight * 3 / 2;
        InputFileFrameCnt = 1;
    }
    else {
        // create frame mgr
        FrameMgrCreate(nFrameDepth + nCacheListDepth);

        nOneSize = nWidth * nHeight * 3 / 2;
        nFrameSize = nStride * nHeight * 3 / 2;

        InputFileHandle = LoadFile(InputFile, &nFileSize);

        if (!InputFileHandle
            || (nFileSize % nOneSize) != 0) {
            ALOGE("%s file is not %dx%d", InputFile, nWidth, nHeight);
            goto EXIT1;
        }

        InputFileFrameCnt = nFileSize / nOneSize;

        if (InputFileFrameCnt == 1) {
            FrameMgrGet(&YUVDataPhy, &YUVDataVir, nFrameSize, nSkelFrameId);

            if (nRet != 0
                || YUVDataPhy == 0
                || YUVDataVir == NULL) {
                ALOGE("Load input file fail");
                goto EXIT1;
            }

            if (nOneSize != nFrameSize) {
                nRet = AX_SYS_MemAlloc(&OneYUVDataPhy, (AX_VOID **)&OneYUVDataVir, nOneSize, 256, (AX_S8 *)"SKEL_TEST");

                nRet = LoadFileToMem(InputFileHandle, OneYUVDataPhy, OneYUVDataVir, nOneSize);

                NV12ToStrideNV12(OneYUVDataPhy,
                                OneYUVDataVir,
                                nOneSize,
                                nWidth,
                                nHeight,
                                YUVDataPhy,
                                YUVDataVir,
                                nFrameSize,
                                nStride,
                                nHeight
                                );
            }
            else {
                nRet = LoadFileToMem(InputFileHandle, YUVDataPhy, YUVDataVir, nFrameSize);
            }
        }
    }

    if (nWidth%2 == 1
        || nHeight%2 == 1) {
        ALOGE("wxh(%dx%d) should be even", nWidth, nHeight);
        goto EXIT1;
    }

    nRet = AX_SYS_MemAlloc(&YUVDataWrPhy, &YUVDataWrVir, nFrameSize, 256, (const AX_S8 *)"SKEL_TEST");

    if (!YUVDataWrVir) {
        ALOGE("malloc fail nRet=0x%x", nRet);
        goto EXIT1;
    }

    if (SaveResultPath) {
        AX_CHAR szPath[512] = {0};
        snprintf(szPath, 511, "%s/%s", SaveResultPath, SKEL_SAMPLE_OUTPUT_LOG_FILE);
        fpResultFile = fopen(szPath, "a+");

        if (!fpResultFile) {
            ALOGE("%s file not exist", szPath);
            goto EXIT1;
        }
    }

    nStartTime = get_tick_count();

    AX_SKEL_INIT_PARAM_S stInitParam = {0};

    if (ModelsPath) {
        stInitParam.pStrModelDeploymentPath = ModelsPath;
    }
    else {
        if (nPPL == AX_SKEL_PPL_BODY
            || nPPL == AX_SKEL_PPL_POSE
            || nPPL == AX_SKEL_PPL_HVCP) {
            stInitParam.pStrModelDeploymentPath = "/opt/etc/skelModels";
        }
        else {
            stInitParam.pStrModelDeploymentPath = "/opt/etc/models";
        }
    }

    nRet = AX_SKEL_Init(&stInitParam);

    nInitElasped = get_tick_count() - nStartTime;

    if (0 != nRet) {
        ALOGE("SKEL init fail, ret = 0x%x", nRet);

        goto EXIT1;
    }

    // get version
    {
        const AX_SKEL_VERSION_INFO_S *pstVersion = NULL;

        nRet = AX_SKEL_GetVersion(&pstVersion);

        if (0 != nRet) {
            ALOGI("SKEL get version fail, ret = 0x%x", nRet);

            if (pstVersion) {
                AX_SKEL_Release((AX_VOID *)pstVersion);
            }
            goto EXIT1;
        }

        ALOGI("SKEL version: %s", pstVersion->pstrVersion);

        if (pstVersion) {
            AX_SKEL_Release((AX_VOID *)pstVersion);
        }
    }

    // get capability
    {
        const AX_SKEL_CAPABILITY_S *pstCapability = NULL;

        nRet = AX_SKEL_GetCapability(&pstCapability);

        if (0 != nRet) {
            ALOGI("SKEL get capability fail, ret = 0x%x", nRet);

            if (pstCapability) {
                AX_SKEL_Release((AX_VOID *)pstCapability);
            }
            goto EXIT1;
        }

        for (size_t i = 0; i < pstCapability->nPPLConfigSize; i++) {
            ALOGI("SKEL capability[%d]: (ePPL: %d, PPLConfigKey: %s)", i, pstCapability->pstPPLConfig[i].ePPL,
                  pstCapability->pstPPLConfig[i].pstrPPLConfigKey);
        }

        if (pstCapability) {
            AX_SKEL_Release((AX_VOID *)pstCapability);
        }
    }

    AX_SKEL_HANDLE_PARAM_S stHandleParam = {0};

    stHandleParam.ePPL = (AX_SKEL_PPL_E)nPPL;
    stHandleParam.nFrameDepth = nFrameDepth;
    stHandleParam.nFrameCacheDepth = nCacheListDepth;
    stHandleParam.nWidth = nWidth;
    stHandleParam.nHeight = nHeight;

    AX_SKEL_CONFIG_S stConfig = {0};
    AX_SKEL_CONFIG_ITEM_S stItems[16] = {0};
    AX_U8 itemIndex = 0;
    stConfig.nSize = 0;
    stConfig.pstItems = &stItems[0];

    // venc_attr_config
    // default will be system definition: w*h*3/8
    AX_SKEL_COMMON_THRESHOLD_CONFIG_S stVencAttrConfigThreshold = {0};
    if (nJencBufSize > 0) {
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"venc_attr_config";
        stVencAttrConfigThreshold.fValue = (AX_F32) nJencBufSize;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stVencAttrConfigThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;
    }

    stConfig.nSize = itemIndex;

    stHandleParam.stConfig = stConfig;

    nStartTime = get_tick_count();

    nRet = AX_SKEL_Create(&stHandleParam, &pHandle);

    nCreateElasped = get_tick_count() - nStartTime;

    if (0 != nRet) {
        ALOGE("SKEL Create Handle fail, ret = 0x%x", nRet);

        goto EXIT2;
    }

    if (fConfidence == 0
        && (AX_SKEL_PPL_E)nPPL <= AX_SKEL_PPL_POSE) {
        fConfidence = 0.5;
    }

    // set config
    {
        AX_SKEL_CONFIG_S stConfig = {0};
        AX_SKEL_CONFIG_ITEM_S stItems[64] = {0};
        AX_U8 itemIndex = 0;
        stConfig.nSize = 0;
        stConfig.pstItems = &stItems[0];

        // body_max_target_count
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stBodyMaxTargetCount = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"body_max_target_count";
        stBodyMaxTargetCount.fValue = (AX_F32)nHumantracksize;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stBodyMaxTargetCount;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;

        // vehicle_max_target_count
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stVehicleMaxTargetCount = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"vehicle_max_target_count";
        stVehicleMaxTargetCount.fValue = (AX_F32)nVehicletracksize;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stVehicleMaxTargetCount;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;

        // cycle_max_target_count
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stCycleMaxTargetCount = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"cycle_max_target_count";
        stCycleMaxTargetCount.fValue = (AX_F32)nCycletracksize;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stCycleMaxTargetCount;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;

        // body_confidence
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stBodyConfidence = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"body_confidence";
        stBodyConfidence.fValue = fConfidence;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stBodyConfidence;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;

#if 0
        // face_confidence
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stFaceConfidence = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"face_confidence";
        stFaceConfidence.fValue = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stFaceConfidence;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // vehicle_confidence
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stVehicleConfidence = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"vehicle_confidence";
        stVehicleConfidence.fValue = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stVehicleConfidence;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // cycle_confidence
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stCycleConfidence = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"cycle_confidence";
        stCycleConfidence.fValue = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stCycleConfidence;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // plate_confidence
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stPlateConfidence = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"plate_confidence";
        stPlateConfidence.fValue = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPlateConfidence;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // crop_encoder_qpLevel
        AX_SKEL_COMMON_THRESHOLD_CONFIG_S stCropEncoderQpLevelThreshold = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"crop_encoder_qpLevel";
        stCropEncoderQpLevelThreshold.fValue = 75;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stCropEncoderQpLevelThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_COMMON_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // body_min_size
        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S stBodyMinSize = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"body_min_size";
        stBodyMinSize.nWidth = 0;
        stBodyMinSize.nHeight = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stBodyMinSize;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // face_min_size
        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S stFaceMinSize = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"face_min_size";
        stFaceMinSize.nWidth = 0;
        stFaceMinSize.nHeight = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stFaceMinSize;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // vehicle_min_size
        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S stVehicleMinSize = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"vehicle_min_size";
        stVehicleMinSize.nWidth = 0;
        stVehicleMinSize.nHeight = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stVehicleMinSize;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // cycle_min_size
        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S stCycleMinSize = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"cycle_min_size";
        stCycleMinSize.nWidth = 0;
        stCycleMinSize.nHeight = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stCycleMinSize;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // plate_min_size
        AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S stPlateMinSize = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"plate_min_size";
        stPlateMinSize.nWidth = 0;
        stPlateMinSize.nHeight = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPlateMinSize;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_OBJECT_SIZE_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // detect_roi_polygon
        AX_SKEL_ROI_POLYGON_CONFIG_S stDetectRoi = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"detect_roi_polygon";
        stDetectRoi.bEnable = AX_FALSE;
        stDetectRoi.nPointNum = 4;
        AX_SKEL_POINT_S stPoint[4] = {{0, 0}, {nWidth, 0}, {nWidth, nHeight}, {0, nHeight}};
        stDetectRoi.pstPoint = (AX_SKEL_POINT_S *)stPoint;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stDetectRoi;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_ROI_POLYGON_CONFIG_S);
        itemIndex++;
#endif

        // push_strategy
        AX_SKEL_PUSH_STRATEGY_S stPushStrategy = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_strategy";
        stPushStrategy.ePushMode = (AX_SKEL_PUSH_MODE_E)nPushStrategy;
        stPushStrategy.nIntervalTimes = (AX_U32)nPushStrategyInterval;
        stPushStrategy.nPushCounts = (AX_U32)nPushStrategyCount;
        stPushStrategy.bPushSameFrame = (AX_BOOL)nPushStrategySameFrame;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPushStrategy;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_PUSH_STRATEGY_S);
        itemIndex++;

#if 0
        // body_crop_encoder
        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S stBodyCropEncoderThreshold = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"body_crop_encoder";
        stBodyCropEncoderThreshold.fScaleLeft = 0;
        stBodyCropEncoderThreshold.fScaleRight = 0;
        stBodyCropEncoderThreshold.fScaleTop = 0;
        stBodyCropEncoderThreshold.fScaleBottom = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stBodyCropEncoderThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // vehicle_crop_encoder
        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S stVehicleCropEncoderThreshold = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"vehicle_crop_encoder";
        stVehicleCropEncoderThreshold.fScaleLeft = 0;
        stVehicleCropEncoderThreshold.fScaleRight = 0;
        stVehicleCropEncoderThreshold.fScaleTop = 0;
        stVehicleCropEncoderThreshold.fScaleBottom = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stVehicleCropEncoderThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // cycle_crop_encoder
        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S stCycleCropEncoderThreshold = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"cycle_crop_encoder";
        stCycleCropEncoderThreshold.fScaleLeft = 0;
        stCycleCropEncoderThreshold.fScaleRight = 0;
        stCycleCropEncoderThreshold.fScaleTop = 0;
        stCycleCropEncoderThreshold.fScaleBottom = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stCycleCropEncoderThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // face_crop_encoder
        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S stFaceCropEncoderThreshold = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"face_crop_encoder";
        stFaceCropEncoderThreshold.fScaleLeft = 0;
        stFaceCropEncoderThreshold.fScaleRight = 0;
        stFaceCropEncoderThreshold.fScaleTop = 0;
        stFaceCropEncoderThreshold.fScaleBottom = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stFaceCropEncoderThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // plate_crop_encoder
        AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S stPlateCropEncoderThreshold = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"plate_crop_encoder";
        stPlateCropEncoderThreshold.fScaleLeft = 0;
        stPlateCropEncoderThreshold.fScaleRight = 0;
        stPlateCropEncoderThreshold.fScaleTop = 0;
        stPlateCropEncoderThreshold.fScaleBottom = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPlateCropEncoderThreshold;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_CROP_ENCODER_THRESHOLD_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // resize_panorama_encoder_config
        AX_SKEL_RESIZE_CONFIG_S stPanoramaResizeConfig = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"resize_panorama_encoder_config";
        stPanoramaResizeConfig.fW = 0;
        stPanoramaResizeConfig.fH = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPanoramaResizeConfig;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_RESIZE_CONFIG_S);
        itemIndex++;
#endif

        // push_panorama
        AX_SKEL_PUSH_PANORAMA_CONFIG_S stPushPanoramaConfig = {0};
        if (nPushPanora != 0) {
            stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_panorama";
            stPushPanoramaConfig.bEnable = (AX_BOOL)nPushPanora;
            stPushPanoramaConfig.nQuality = 75;
            stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPushPanoramaConfig;
            stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_PUSH_PANORAMA_CONFIG_S);
            itemIndex++;
        }

#if 0
        // push_quality_body
        AX_SKEL_ATTR_FILTER_CONFIG_S stBodyAttrFilter = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_quality_body";
        stBodyAttrFilter.stCommonAttrFilterConfig.fQuality = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stBodyAttrFilter;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // push_quality_vehicle
        AX_SKEL_ATTR_FILTER_CONFIG_S stVehicleAttrFilter = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_quality_vehicle";
        stVehicleAttrFilter.stCommonAttrFilterConfig.fQuality = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stVehicleAttrFilter;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S);
        itemIndex++;
#endif

#if 0
        // push_quality_cycle
        AX_SKEL_ATTR_FILTER_CONFIG_S stCycleAttrFilter = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_quality_cycle";
        stCycleAttrFilter.stCommonAttrFilterConfig.fQuality = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stCycleAttrFilter;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S);
        itemIndex++;
#endif

        // push_quality_face
        AX_SKEL_ATTR_FILTER_CONFIG_S stFaceAttrFilter = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_quality_face";
        stFaceAttrFilter.stFaceAttrFilterConfig.nWidth = 0;
        stFaceAttrFilter.stFaceAttrFilterConfig.nHeight = 0;
        stFaceAttrFilter.stFaceAttrFilterConfig.stPoseblur.fPitch = fPushFacePitchfilter;
        stFaceAttrFilter.stFaceAttrFilterConfig.stPoseblur.fYaw = fPushFaceYawfilter;
        stFaceAttrFilter.stFaceAttrFilterConfig.stPoseblur.fRoll = fPushFaceRollfilter;
        stFaceAttrFilter.stFaceAttrFilterConfig.stPoseblur.fBlur = fPushFaceBlurfilter;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S);
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stFaceAttrFilter;
        itemIndex++;

#if 0
        // push_quality_plate
        AX_SKEL_ATTR_FILTER_CONFIG_S stPlateAttrFilter = {0};
        stConfig.pstItems[itemIndex].pstrType = (AX_CHAR *)"push_quality_plate";
        stPlateAttrFilter.stCommonAttrFilterConfig.fQuality = 0;
        stConfig.pstItems[itemIndex].pstrValue = (AX_VOID *)&stPlateAttrFilter;
        stConfig.pstItems[itemIndex].nValueSize = sizeof(AX_SKEL_ATTR_FILTER_CONFIG_S);
        itemIndex++;
#endif
        stConfig.nSize = itemIndex;

        nRet = AX_SKEL_SetConfig(pHandle, &stConfig);

        if (0 != nRet) {
            ALOGE("SKEL AX_SKEL_SetConfig, ret = 0x%x", nRet);

            goto EXIT2;
        }
    }

    // get config
    {
        const AX_SKEL_CONFIG_S *pstConfig = NULL;

        nRet = AX_SKEL_GetConfig(pHandle, &pstConfig);

        if (0 != nRet) {
            ALOGE("SKEL AX_SKEL_GetConfig, ret = 0x%x", nRet);

            if (pstConfig) {
                AX_SKEL_Release((AX_VOID *)pstConfig);
            }

            goto EXIT2;
        }

        ParseConfigParam(pstConfig);

        if (pstConfig) {
            AX_SKEL_Release((AX_VOID *)pstConfig);
        }
    }

    ALOGN("Task infomation:");
    ALOGN("\tInput file: %s", InputFile);
    ALOGN("\tInput file resolution: %dx%d", nWidth, nHeight);
    ALOGN("\tRepeat times: %d", nRepeatTimes);
    ALOGN("SKEL Init Elapse:");
    ALOGN("\tAX_SKEL_Init: %lld ms", nInitElasped);
    ALOGN("\tAX_SKEL_Create: %lld ms", nCreateElasped);

    OUTPUT_LOG_SAVE("==============================%s process result:==============================", InputFile);

    if (InputFileFrameCnt > 1) {
        InputFileFrameCnt = InputFileFrameCnt + BLACK_VIDEO_FRAME_COUNT;
    }

    for (AX_U32 nRepeat = 0; nRepeat < nRepeatTimes; nRepeat++) {
        ALOGN("#####SKEL Process times: %d", nRepeat + 1);

        OUTPUT_LOG_SAVE("#####SKEL Process times: %d", nRepeat + 1);

        STAT_OBJECT_NUM_T tObjectTrackNum = {0};
        STAT_OBJECT_NUM_T tObjectPushNum = {0};
        for (AX_U32 nFrameCnt = 0; nFrameCnt < InputFileFrameCnt; nFrameCnt++) {
            if (InputFileFrameCnt != 1) {
                if (FrameSkipCtrl(nSrcFrameRate, nDstFrameRate, nSkelFrameId)) {
                    nSkelFrameId ++;

                    LoadFileToMem(InputFileHandle, 0, NULL, nOneSize);

                    if (nInterval > 0) {
                        usleep(nInterval);
                    }
                    continue;
                }
                else {
                    if (!FrameMgrGet(&YUVDataPhy, &YUVDataVir, nFrameSize, nSkelFrameId)) {
                        ALOGE("FrameMgrGet fail");

                        goto EXIT1;
                    }

                    if (nOneSize != nFrameSize) {
                        LoadFileToMem(InputFileHandle, OneYUVDataPhy, OneYUVDataVir, nOneSize);

                        NV12ToStrideNV12(OneYUVDataPhy,
                                        OneYUVDataVir,
                                        nOneSize,
                                        nWidth,
                                        nHeight,
                                        YUVDataPhy,
                                        YUVDataVir,
                                        nFrameSize,
                                        nStride,
                                        nHeight
                                        );
                    }
                    else {
                        LoadFileToMem(InputFileHandle, YUVDataPhy, YUVDataVir, nOneSize);
                    }
                }
            }

            stFrame.stFrame.u32Width = nWidth;
            stFrame.stFrame.u32Height = nHeight;
            stFrame.stFrame.enImgFormat = AX_YUV420_SEMIPLANAR;
            stFrame.stFrame.u32FrameSize = nFrameSize;
            stFrame.stFrame.u32PicStride[0] = nStride;
            stFrame.stFrame.u32PicStride[1] = nStride;
            stFrame.stFrame.u32PicStride[2] = nStride;
            stFrame.stFrame.u64PhyAddr[0] = YUVDataPhy;
            stFrame.stFrame.u64PhyAddr[1] = YUVDataPhy + nStride * nHeight;
            stFrame.stFrame.u64PhyAddr[2] = 0;
            stFrame.stFrame.u64VirAddr[0] = (AX_U64)((AX_U32)YUVDataVir);
            stFrame.stFrame.u64VirAddr[1] = (AX_U64)((AX_U32)YUVDataVir + nStride * nHeight);
            stFrame.stFrame.u64VirAddr[2] = (AX_U64)((AX_U32)0);

            ALOGN("*****SKEL Frame(%d) Process", nFrameCnt + 1);

            OUTPUT_LOG_SAVE("*****SKEL Frame(%d) Process Start", nFrameCnt + 1);

            stFrame.nFrameId = nSkelFrameId ++;
            nStartTime = get_tick_count();

            while (1) {
                nRet = AX_SKEL_SendFrame(pHandle, &stFrame, 0);

                if (nRet == 0
                    || nRet != AX_ERR_SKEL_QUEUE_FULL) {
                    break;
                }

                usleep(1000);
            }

            nProcessElasped = get_tick_count() - nStartTime;

            if (0 != nRet) {
                ALOGE("SKEL Process fail, ret = 0x%x", nRet);

                goto EXIT3;
            }

            ALOGN("SKEL Process Elapse:");
            ALOGN("\tAX_SKEL_SendFrame: %lld ms", nProcessElasped);

            nStartTime = get_tick_count();

            nRet = AX_SKEL_GetResult(pHandle, &pstResult, -1);

            nResultElasped = get_tick_count() - nStartTime;

            if (0 != nRet) {
                ALOGE("SKEL get result fail, ret = 0x%x", nRet);

                goto EXIT3;
            }

            ALOGN("\tAX_SKEL_GetResult: %lld ms", nResultElasped);

            nProcessElasped = (nProcessElasped + nResultElasped);
            nResultElaspedTotal += nProcessElasped;

            if (nProcessElasped > nResultElaspedMax) {
                nResultElaspedMax = nProcessElasped;
            }

            if (nProcessElasped < nResultElaspedMin) {
                nResultElaspedMin = nProcessElasped;
            }

            ALOGN("SKEL Process Result:");

            ALOGI("\tFrameId: %lld", pstResult->nFrameId);
            ALOGI("\tnOriginal WxH: %dx%d", pstResult->nOriginalWidth, pstResult->nOriginalHeight);

            ALOGN("\tObject Num: %d", pstResult->nObjectSize);

            AX_U32 nSkelSize = 0;
            AI_Detection_SkelResult_t Skels[SKEL_SAMPLE_OBJECT_SIZE] = {0};
            for (size_t i = 0; i < pstResult->nObjectSize; i++) {
                AX_SKEL_OBJECT_ITEM_S *pstItems = &pstResult->pstObjectItems[i];

                ALOGI("\t\tFrameId: %lld", pstItems->nFrameId);
                ALOGI("\t\tTrackId: %lld, TrackState: %d", pstItems->nTrackId, pstItems->eTrackState);

                ALOGN("\t\tRect[%d] %s: [%f, %f, %f, %f], Confidence: %f", i, pstItems->pstrObjectCategory,
                      pstItems->stRect.fX,
                      pstItems->stRect.fY, pstItems->stRect.fW,
                      pstItems->stRect.fH, pstItems->fConfidence);

                StatTrackMgr(pstItems, &tObjectTrackNum);
                StatPushMgr(pstItems, &tObjectPushNum);

                // get detect box only new or update state
                if ((pstItems->eTrackState == AX_SKEL_TRACK_STATUS_NEW
                    || pstItems->eTrackState == AX_SKEL_TRACK_STATUS_UPDATE)
                    && nSkelSize < SKEL_SAMPLE_OBJECT_SIZE) {
                    Skels[nSkelSize].tBox.fX = pstItems->stRect.fX;
                    Skels[nSkelSize].tBox.fY = pstItems->stRect.fY;
                    Skels[nSkelSize].tBox.fW = pstItems->stRect.fW;
                    Skels[nSkelSize].tBox.fH = pstItems->stRect.fH;

                    OUTPUT_LOG_SAVE("\t\tObject[%d] %s: [%f, %f, %f, %f], Confidence: %f",
                                    i, pstItems->pstrObjectCategory,
                                    pstItems->stRect.fX, pstItems->stRect.fY,
                                    pstItems->stRect.fW, pstItems->stRect.fH,
                                    pstItems->fConfidence);

                    ALOGN("\t\t[%d]Point Set Size: %d", i, pstItems->nPointSetSize);

                    // point
                    Skels[nSkelSize].nPointNum = AX_MIN(DETECT_SKEL_POINT_COUNT, pstItems->nPointSetSize);
                    for (size_t j = 0; j < Skels[i].nPointNum; j++) {
                        ALOGI("\t\t\tPoint[%d] %s: [%f, %f] Confidence: %f", j, pstItems->pstPointSet[j].pstrObjectCategory,
                            pstItems->pstPointSet[j].stPoint.fX,
                            pstItems->pstPointSet[j].stPoint.fY,
                            pstItems->pstPointSet[j].fConfidence);
                            Skels[nSkelSize].tPoint[j].fX = pstItems->pstPointSet[j].stPoint.fX;
                            Skels[nSkelSize].tPoint[j].fY = pstItems->pstPointSet[j].stPoint.fY;

                        OUTPUT_LOG_SAVE("\t\t\tPoint[%d] %s: [%f, %f] Confidence: %f\n",
                                        j,
                                        pstItems->pstPointSet[j].pstrObjectCategory,
                                        pstItems->pstPointSet[j].stPoint.fX,
                                        pstItems->pstPointSet[j].stPoint.fY,
                                        pstItems->pstPointSet[j].fConfidence);
                    }

                    nSkelSize ++;
                }

                // crop frame
                if (AX_SKEL_TRACK_STATUS_SELECT == pstItems->eTrackState) {
                    ATTRINFO_T AttrInfo;
                    memset(&AttrInfo, 0x00, sizeof(AttrInfo));
                    AttrParser(pstItems, &AttrInfo);

                    if (pstItems->bCropFrame
                        && pstItems->stCropFrame.pFrameData
                        && 0 < pstItems->stCropFrame.nFrameDataSize
                        && SaveResultPath) {
                        // save attribute
                        AX_CHAR arrDat[256] = {0};
                        AX_CHAR strFile[256] = {0};
                        if (AttrInfo.bExist && AttrInfo.eType == ATTR_TYPE_FACE) {
                            sprintf(arrDat, "frame%lld_crop_%s[%lld]_Gender[%s]_Age[%d]_Mask[%s]_Score[%f]_%dx%d",
                                    pstItems->stCropFrame.nFrameId,
                                    pstItems->pstrObjectCategory,
                                    pstItems->nTrackId,
                                    AttrInfo.tFaceInfo.szGender,
                                    AttrInfo.tFaceInfo.nAge,
                                    AttrInfo.tFaceInfo.szMask,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth,
                                    pstItems->stCropFrame.nFrameHeight);

                            OUTPUT_LOG_SAVE("\t\t[FACE ATTR] %s", arrDat);
                        }
                        else if (AttrInfo.bExist && AttrInfo.eType == ATTR_TYPE_PLATE) {
                            sprintf(arrDat, "frame%lld_crop_%s[%lld]_Valid[%d]_Num[%s]_Color[%s]_Score[%f]_%dx%d",
                                    pstItems->stCropFrame.nFrameId,
                                    pstItems->pstrObjectCategory,
                                    pstItems->nTrackId,
                                    AttrInfo.tPlateInfo.bValid,
                                    AttrInfo.tPlateInfo.szNum,
                                    AttrInfo.tPlateInfo.szColor,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth,
                                    pstItems->stCropFrame.nFrameHeight);

                            OUTPUT_LOG_SAVE("\t\t[PLATE ATTR] %s", arrDat);
                        }
                        else if (AttrInfo.bExist && AttrInfo.eType == ATTR_TYPE_VEHICLE) {
                            sprintf(arrDat, "frame%lld_crop_%s[%lld]_[plate_Valid[%d]_Num[%s]_Color[%s]]_Score[%f]_%dx%d",
                                    pstItems->stCropFrame.nFrameId,
                                    pstItems->pstrObjectCategory,
                                    pstItems->nTrackId,
                                    AttrInfo.tPlateInfo.bValid,
                                    AttrInfo.tPlateInfo.szNum,
                                    AttrInfo.tPlateInfo.szColor,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth,
                                    pstItems->stCropFrame.nFrameHeight);

                            OUTPUT_LOG_SAVE("\t\t[VEHICLE ATTR] %s", arrDat);
                        }
                        else {
                            sprintf(arrDat, "frame%lld_crop_%s[%lld]_Score[%f]_%dx%d",
                                    pstItems->stCropFrame.nFrameId,
                                    pstItems->pstrObjectCategory,
                                    pstItems->nTrackId,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth,
                                    pstItems->stCropFrame.nFrameHeight);
                        }

                        sprintf(strFile, "%s/%s/%s.jpg",
                                SaveResultPath,
                                pstItems->pstrObjectCategory,
                                arrDat);

                        FILE *fp_w = fopen(strFile, "wb");

                        if (fp_w) {
                            ALOGI("Write crop jpg to file: %s", strFile);
                            fwrite((AX_U8 *)pstItems->stCropFrame.pFrameData, 1, pstItems->stCropFrame.nFrameDataSize, fp_w);
                            fclose(fp_w);
                        }
                    }

                    // panora frame
                    if (pstItems->bPanoraFrame
                        && pstItems->stPanoraFrame.pFrameData
                        && 0 < pstItems->stPanoraFrame.nFrameDataSize
                        && SaveResultPath) {
                        AX_CHAR arrDat[256] = {0};
                        AX_CHAR strFile[256] = {0};
                        if (AttrInfo.bExist && AttrInfo.eType == ATTR_TYPE_FACE) {
                            sprintf(arrDat, "frame%lld_panora_%s[%lld]_Gender[%s]_Age[%d]_Mask[%s]_Score[%f]_%dx%d.jpg",
                                    pstItems->stCropFrame.nFrameId, pstItems->pstrObjectCategory, pstItems->nTrackId,
                                    AttrInfo.tFaceInfo.szGender, AttrInfo.tFaceInfo.nAge, AttrInfo.tFaceInfo.szMask,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth, pstItems->stCropFrame.nFrameHeight);
                        }
                        else if (AttrInfo.bExist && AttrInfo.eType == ATTR_TYPE_PLATE) {
                            sprintf(arrDat, "frame%lld_panora_%s[%lld]_Valid[%d]_Num[%s]_Color[%s]_Score[%f]_%dx%d.jpg",
                                    pstItems->stCropFrame.nFrameId, pstItems->pstrObjectCategory, pstItems->nTrackId,
                                    AttrInfo.tPlateInfo.bValid, AttrInfo.tPlateInfo.szNum, AttrInfo.tPlateInfo.szColor,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth, pstItems->stCropFrame.nFrameHeight);
                        }
                        else if (AttrInfo.bExist && AttrInfo.eType == ATTR_TYPE_VEHICLE) {
                            sprintf(arrDat, "frame%lld_panora_%s[%lld]_[plate_Valid[%d]_Num[%s]_Color[%s]]_Score[%f]_%dx%d.jpg",
                                    pstItems->stCropFrame.nFrameId, pstItems->pstrObjectCategory, pstItems->nTrackId,
                                    AttrInfo.tPlateInfo.bValid, AttrInfo.tPlateInfo.szNum, AttrInfo.tPlateInfo.szColor,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth, pstItems->stCropFrame.nFrameHeight);
                        }
                        else {
                            sprintf(arrDat, "frame%lld_panora_%s[%lld]_Score[%f]_%dx%d.jpg",
                                    pstItems->stCropFrame.nFrameId, pstItems->pstrObjectCategory, pstItems->nTrackId,
                                    pstItems->fConfidence,
                                    pstItems->stCropFrame.nFrameWidth, pstItems->stCropFrame.nFrameHeight);
                        }

                        sprintf(strFile, "%s/%s/%s.jpg",
                                SaveResultPath,
                                pstItems->pstrObjectCategory,
                                arrDat);

                        FILE *fp_w = fopen(strFile, "wb");

                        if (fp_w) {
                            ALOGI("Write panora jpg to file: %s", strFile);
                            fwrite((AX_U8 *)pstItems->stPanoraFrame.pFrameData, 1, pstItems->stPanoraFrame.nFrameDataSize, fp_w);
                            fclose(fp_w);
                        }
                    }
                }

                // feature
                ALOGI("\t\tFeature Size: %d", pstItems->nFeatureSize);
                if (pstItems->pstFeatureItem
                    && 0 < pstItems->nFeatureSize
                    && SaveResultPath) {
                    AX_CHAR strFile[256] = {0};
                    sprintf(strFile, "%s/%s/frame%lld_feature_%s_%d.db",
                        SaveResultPath, pstItems->pstrObjectCategory,
                        pstItems->stCropFrame.nFrameId,
                        pstItems->pstrObjectCategory, i);
                    FILE *fp_w = fopen(strFile, "wb");

                    if (fp_w) {
                        ALOGI("\t\t\tWrite feature to file: %s, nValueSize: %d",
                                strFile,
                                pstItems->pstFeatureItem[0].nValueSize);
                        fwrite((AX_U8 *)pstItems->pstFeatureItem[0].pstrValue, 1, pstItems->pstFeatureItem[0].nValueSize, fp_w);
                        fclose(fp_w);
                    }
                }

                // meta
                ALOGI("\t\tMeta Size: %d", pstItems->nMetaInfoSize);
                if (pstItems->pstMetaInfo
                    && 0 < pstItems->nMetaInfoSize) {
                    for (size_t i = 0; i < pstItems->nMetaInfoSize; i++) {
                        ALOGI("\t\tMetaInfo[%d] %s: %s", i, pstItems->pstMetaInfo[i].pstrType, pstItems->pstMetaInfo[i].pstrValue);
                    }
                }

                // binds
                ALOGI("\t\tBind Size: %d", pstItems->nObjectBindSize);
                if (pstItems->pstObjectBind
                    && 0 < pstItems->nObjectBindSize) {
                    for (size_t i = 0; i < pstItems->nObjectBindSize; i++) {
                        ALOGI("\t\t[%s] [TrackId] %lld bind to ObjectBind[%d]: %s [TrackId] %lld",
                            pstItems->pstrObjectCategory, pstItems->nTrackId, i,
                            pstItems->pstObjectBind[i].pstrObjectCategoryBind, pstItems->pstObjectBind[i].nTrackId);
                    }
                }
            }

            ALOGI("\tnCacheListSize: %d", pstResult->nCacheListSize);
            if (pstResult->pstCacheList) {
                for (size_t i = 0; i < pstResult->nCacheListSize; i++) {
                    ALOGI("\t\tCacheList[%d] FrameId: %lld", i, pstResult->pstCacheList[i].nFrameId);
                }
            }

            if (ReWritePath && pstResult->nObjectSize > 0) {
                memcpy(YUVDataWrVir, YUVDataVir, nFrameSize);

                YUV_IMAGE_S YUVImage = {0};
                YUVImage.pImage = YUVDataWrVir;
                YUVImage.nWidth = nWidth;
                YUVImage.nHeight = nHeight;
                YUVImage.stride = nStride;
                YUVImage.nSize = nFrameSize;
                YUVImage.eType = AX_YUV420_SEMIPLANAR;

                for (size_t i = 0; i < nSkelSize; i++) {
                    // draw rect
                    AX_S16 x0 = Skels[i].tBox.fX;
                    AX_S16 y0 = Skels[i].tBox.fY;
                    AX_U16 w = Skels[i].tBox.fW;
                    AX_U16 h = Skels[i].tBox.fH;

                    DrawRect(&YUVImage, x0, y0, w, h, YUV_WHITE);

                    if (Skels[i].nPointNum > 0) {
                        // draw point
                        for (size_t j = 0; j < Skels[i].nPointNum; j++) {
                            x0 = Skels[i].tPoint[j].fX;
                            y0 = Skels[i].tPoint[j].fY;

                            DrawPoint(&YUVImage, x0, y0, 4, x0 * (4 - 1), y0 * (4 - 1), YUV_DARK_GREEN);
                        }

                        // draw line
                        if (nPPL == AX_SKEL_PPL_POSE) {
                            for (size_t j = 0; j < sizeof(pairs) / sizeof(struct skeleton); j++) {
                                YUV_COLOR LineColor = YUV_RED;
                                switch (pairs[i].left_right_neutral) {
                                    case 0:
                                        LineColor = YUV_BLUE;
                                        break;
                                    case 1:
                                        LineColor = YUV_DARK_GREEN;
                                        break;
                                    default:
                                        LineColor = YUV_RED;
                                        break;
                                }

                                AX_S32 x1 = (AX_S32)(Skels[i].tPoint[pairs[j].connection[0]].fX);
                                AX_S32 y1 = (AX_S32)(Skels[i].tPoint[pairs[j].connection[0]].fY);
                                AX_S32 x2 = (AX_S32)(Skels[i].tPoint[pairs[j].connection[1]].fX);
                                AX_S32 y2 = (AX_S32)(Skels[i].tPoint[pairs[j].connection[1]].fY);

                                x1 = AX_MIN(x1, (AX_S32)(nWidth - 1));
                                x1 = AX_MAX(x1, 0);
                                y1 = AX_MIN(y1, (AX_S32)(nHeight - 1));
                                y1 = AX_MAX(y1, 0);
                                x2 = AX_MIN(x2, (AX_S32)(nWidth - 1));
                                x2 = AX_MAX(x2, 0);
                                y2 = AX_MIN(y2, (AX_S32)(nHeight - 1));
                                y2 = AX_MAX(y2, 0);

                                DrawLine(&YUVImage, x1, y1, x2, y2, LineColor, 1);
                            }
                        }
                    }
                }

                AX_CHAR arrWrFile[256] = {0};
                if (bJpgFile) {
                    AX_CHAR arrTmpWrFile[256] = {0};
                    AX_U32 nInputFileLen = strlen(InputFile);

                    sprintf(arrTmpWrFile, "%s", InputFile);
                    arrTmpWrFile[nInputFileLen - 4] = 0;

                    sprintf(arrWrFile, "%s_result_%d_%d.jpg", arrTmpWrFile, nFrameCnt + 1, nRepeat + 1);
                }
                else {
                    sprintf(arrWrFile, "%s_result_%d_%d.jpg", InputFile, nFrameCnt + 1, nRepeat + 1);
                }

                AX_CHAR *fileName = NULL;
                AX_U32 fileNameLen = strlen(arrWrFile);
                AX_CHAR fullFileName[256] = {0};
                AX_S32 i = fileNameLen - 1;

                if (fileNameLen > 0) {
                    for (; i >= 0; i --) {
                        if (arrWrFile[i] == '/') {
                            break;
                        }
                    }
                }

                if (i < 0) {
                    i = 0;
                }
                else if (i < fileNameLen - 1) {
                    i ++;
                }

                fileName = (AX_CHAR *)&arrWrFile[i];

                sprintf(fullFileName, "%s/%s", ReWritePath, fileName);

                EncodeOneFrameToJpeg((AX_CHAR *)fullFileName, nStride, nWidth, nHeight, YUVDataWrPhy, YUVDataWrVir, nFrameSize);
            }

            if (InputFileFrameCnt != 1) {
                FrameMgr(pstResult);
            }

            if (pstResult) {
                AX_SKEL_Release((AX_VOID *)pstResult);
            }

            OUTPUT_LOG_SAVE("*****SKEL Frame(%d) Process End (Elasped: %lld ms)", nFrameCnt + 1, nProcessElasped);

            if (nInterval > 0) {
                usleep(nInterval);
            }
        }

        ALOGN("SKEL Process Objects Statistics: Body[%d], Vehicle[%d], Cycle[%d], Face[%d], Plate[%d]",
                tObjectTrackNum.nBodyNum,
                tObjectTrackNum.nVehicleNum,
                tObjectTrackNum.nCycleNum,
                tObjectTrackNum.nFaceNum,
                tObjectTrackNum.nPlateNum);

        OUTPUT_LOG_SAVE("\nSKEL Process Objects Statistics: Body[%d], Vehicle[%d], Cycle[%d], Face[%d], Plate[%d]",
                        tObjectTrackNum.nBodyNum,
                        tObjectTrackNum.nVehicleNum,
                        tObjectTrackNum.nCycleNum,
                        tObjectTrackNum.nFaceNum,
                        tObjectTrackNum.nPlateNum);

        ALOGN("SKEL Process Push Statistics: Body[%d], Vehicle[%d], Cycle[%d], Face[%d], Plate[%d]",
                tObjectPushNum.nBodyNum,
                tObjectPushNum.nVehicleNum,
                tObjectPushNum.nCycleNum,
                tObjectPushNum.nFaceNum,
                tObjectPushNum.nPlateNum);

        OUTPUT_LOG_SAVE("SKEL Process Push Statistics: Body[%d], Vehicle[%d], Cycle[%d], Face[%d], Plate[%d]\n",
                        tObjectPushNum.nBodyNum,
                        tObjectPushNum.nVehicleNum,
                        tObjectPushNum.nCycleNum,
                        tObjectPushNum.nFaceNum,
                        tObjectPushNum.nPlateNum);

        if (InputFileHandle) {
            ReLoadFile(InputFileHandle);
        }
    }

    ALOGN("SKEL Process Elapsed Info: Repeats: %d, (min: %lld ms, avr: %lld ms, max: %lld ms)",
            nRepeatTimes,
            nResultElaspedMin,
            (nSkelFrameId > 1) ? (nResultElaspedTotal / (nSkelFrameId - 1)) : 0,
            nResultElaspedMax);

    OUTPUT_LOG_SAVE("SKEL Process Elapsed Info: Repeats: %d, (min: %lld ms, avr: %lld ms, max: %lld ms)",
                    nRepeatTimes,
                    nResultElaspedMin,
                    (nSkelFrameId > 1) ? (nResultElaspedTotal / (nSkelFrameId - 1)) : 0,
                    nResultElaspedMax);

EXIT3:
    if (pHandle) {
        AX_SKEL_Destroy(pHandle);
    }

EXIT2:
    AX_SKEL_DeInit();

EXIT1:
    AX_NPU_SDK_EX_Deinit();

EXIT0:
    if (bJpgFile) {
        if (YUVDataVir) {
            if (YUVDataPhy == 0) {
                free(YUVDataVir);
            } else {
                AX_SYS_MemFree(YUVDataPhy, YUVDataVir);
            }
        }
    }
    else {
        FrameMgrDestroy();
    }

    if (YUVDataVir) {
        if (YUVDataPhy == 0) {
            free(YUVDataVir);
        } else {
            AX_SYS_MemFree(YUVDataPhy, YUVDataVir);
        }
    }
    if (YUVDataWrVir) {
        if (YUVDataWrPhy == 0) {
            free(YUVDataWrVir);
        } else {
            AX_SYS_MemFree(YUVDataWrPhy, YUVDataWrVir);
        }
    }

    if (OneYUVDataVir) {
        if (OneYUVDataPhy == 0) {
            free(OneYUVDataVir);
        } else {
            AX_SYS_MemFree(OneYUVDataPhy, OneYUVDataVir);
        }
    }

    if (fpResultFile) {
        fclose(fpResultFile);
    }

    if (InputFileHandle) {
        UnLoadFile(InputFileHandle);
    }

    AX_VENC_Deinit();

    AX_VDEC_DeInit();

    AX_SYS_Deinit();

    return (0 != nRet) ? -1 : 0;
}
