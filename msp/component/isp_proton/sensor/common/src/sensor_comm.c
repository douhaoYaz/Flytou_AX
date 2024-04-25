#include <math.h>
#include<stdlib.h>
#include<stdio.h>
#include<isp_sensor_internal.h>
#include<isp_sensor_types.h>

#define EPS (1E-06)

static int Compare(const void *a, const void *b){
    float da = *(float *)a;
    float db = *(float *)b;
    return (da > db) ? 1 : -1;
}

void ax_sns_quick_sort_float(void* base, size_t num){
    qsort(base, num, sizeof(float), Compare);
    return;
}

int ax_sns_is_zero(float para){
     if (fabs(para) < EPS) {
        return 1;
     }
     return 0;
}

AX_U32 ax_float_convert_to_int(AX_F32 value, AX_U32 int_bit, AX_U32 frac_bit, AX_BOOL signed_value)
{
    AX_U32 result = 0;
    if ((int_bit + frac_bit + signed_value) > 32) {
        printf("%s, invalid parameters\n", __func__);
        return -1;
    }

    result = ABS(value) * (AX_U32)(1 << frac_bit);

    AX_U32 sign_bit = 0;
    AX_U32 data_bit = int_bit + frac_bit;
    /* if reg is signed */
    if (signed_value) {
        sign_bit = int_bit + frac_bit;
        if (value < 0) {
            result = ((~result + 1) & MASK(data_bit)) | (1 << sign_bit);
        } else {
            result = result & MASK(data_bit);
        }
    }

    return result;
}

AX_F32 ax_int_convert_to_float(AX_S64 p, AX_S32 int_bit, AX_S32 frac_bit, AX_BOOL signed_value)
{
    AX_F32 result = 0.0;
    if ((int_bit + frac_bit + signed_value) > 32) {
        printf("%s, invalid parameters\n", __func__);
        return -1.0;
    }
    AX_BOOL neg_flag = AX_FALSE;
    if (signed_value) {
        if (p < 0) {
            p = -p;
            neg_flag = AX_TRUE;
        }
    }

    result = ((AX_F64)p / ((AX_U64)1 << frac_bit));

    if (neg_flag) {
        result = -result;
    }

    return (AX_F32)result;
}