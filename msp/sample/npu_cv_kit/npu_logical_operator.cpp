#include <cstdio>
#include <assert.h>
#include <string>
#include <chrono>
#include <memory.h>
#include <math.h>
#include <float.h>
#include "ax_sys_api.h"
#include "ax_npu_imgproc.h"
#include "cmdline.h"
#include "img_helper.h"
#include <sys/stat.h>
#include <iostream>
#include <fstream>

#define min(a,b) ((a)>(b)?(b):(a))
#define max(a,b) ((a)<(b)?(b):(a))

int main(int argc, char* argv[]) {

    printf("NPU-CV-KIT Version:%s\n", AX_NPU_CV_Version());

    cmdline::parser args;
    args.add<std::string> ("mode", '\0', "NPU hard mode: disable, 1_1", false, "disable");
    args.add<std::string> ("mode-type", '\0', "Virtual NPU mode type: disable， 1_1_1， 1_1_2", false, "disable");
    args.add<AX_U32>("row", '\0', "rows of matrix A", true);
    args.add<AX_U32>("column", '\0', "columns of matrix A", true);
    args.add<std::string> ("A", '\0', "Input file of matrix A", false);
    args.add<std::string> ("B", '\0', "Input file of matrix B", false);
    args.add<std::string> ("input-data-type", '\0', "input data type", false, "uint8");
    args.add<std::string> ("output-data-type", '\0', "output data type", false, "uint8");
    args.add<std::string> ("operator", '\0', "oprator method(and,or,xor)", true, "and");
    args.add<std::string> ("output", '\0', "Output directory for result", false, "");
    args.add<AX_U32>("repeat", '\0', "MatArith execution count", false, 1);
    args.add<AX_U32>("check", '\0', "check", false, 0);

    // example:
    // LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH ./bin/npu_matarithax620a --mode=disable --mode-type=disable --M 100 --N 100 --output .

    args.parse_check(argc, argv);

    AX_NPU_SDK_EX_ATTR_T hard_mode = get_npu_hard_mode(args.get<std::string>("mode"));

    assert(AX_SYS_Init() == 0);
    assert(AX_NPU_SDK_EX_Init_with_attr(&hard_mode) == 0);

    int nRepeat = args.get<AX_U32>("repeat");
    int bCheck = args.get<AX_U32>("check");
    std::string out_dir = args.get<std::string>("output");

    int M = args.get<AX_U32>("row");
    int N = args.get<AX_U32>("column");
    std::string input_a = args.get<std::string>("A");
    std::string input_b = args.get<std::string>("B");
    std::string arith_method = args.get<std::string>("operator");
    std::string input_type = args.get<std::string>("input-data-type");
    std::string output_type = args.get<std::string>("output-data-type");

    AX_NPU_CV_DataType in_type = get_data_type(input_type);
    AX_NPU_CV_DataType out_type = get_data_type(output_type);

    AX_NPU_CV_Matrix2D mat_a = create_matrix_universal(input_a, M, N, in_type);
    AX_NPU_CV_Matrix2D mat_b = create_matrix_universal(input_b, M, N, in_type);
    assert(mat_a.nColumns == mat_b.nColumns);

    matrix_binary(&mat_a);
    matrix_binary(&mat_b);

    AX_NPU_CV_DataType result_data_type = out_type;
    AX_NPU_CV_Matrix2D result_matrix_2d = create_matrix_by_size(M, N, result_data_type);

    AX_NPU_SDK_EX_MODEL_TYPE_T virtual_npu_mode_type = get_npu_mode_type(args.get<std::string>("mode-type"));

    auto time_start = std::chrono::system_clock::now();
    for (int i = 0; i < nRepeat; ++i) {
        if(arith_method == "and")
            assert(0 == AX_NPU_CV_AND(virtual_npu_mode_type, &mat_a, &mat_b, &result_matrix_2d));
        else if(arith_method == "or")
            assert(0 == AX_NPU_CV_OR(virtual_npu_mode_type, &mat_a, &mat_b, &result_matrix_2d));
        else if(arith_method == "xor")
            assert(0 == AX_NPU_CV_XOR(virtual_npu_mode_type, &mat_a, &mat_b, &result_matrix_2d));
        else assert(0);
    }

    auto time_end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start) / nRepeat;
    printf("\n**************************************************\n");
    printf("Run task took %lld us (%d rounds for average)\n", duration.count(), nRepeat);
    printf("**************************************************\n");

    if (!out_dir.empty()) {
        struct stat info;
        if (stat(out_dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            fprintf(stderr, "cannot access directory '%s'\n", out_dir.c_str());
            return 1;
        }

        {
            char fn[256];
            sprintf(fn, "%s/result.%s.%dx%d.npu_mode_%d.%s",
                out_dir.c_str(),
                arith_method.c_str(),
                M, N,
                virtual_npu_mode_type,
                get_data_type_name(result_data_type).c_str());
            dump_matrix_to_file(fn, result_matrix_2d);
        }

    }

    // Calculate GT
    if(bCheck){
        //printf("src a=====================\n");
        //print_matrix(mat_a);
        //printf("src b=====================\n");
        //print_matrix(mat_b);

        AX_NPU_CV_Matrix2D gt_mat = create_matrix_by_size(M, N, result_data_type);
        AX_U8 * gt_data = (AX_U8*)gt_mat.pVir;
        memset(gt_data, 0, M * N * sizeof(AX_U8));

        AX_U8 *a_data = (AX_U8  *)mat_a.pVir;
        AX_U8  *b_data = (AX_U8 *)mat_b.pVir;
        for (int m = 0; m < M; ++m) {
            for (int n = 0; n < N; ++n) {
                AX_U32 res = 0;
                if(arith_method == "and")
                    res =(AX_U32)a_data[m*N+n] * (AX_U32)b_data[m*N+n];
                if(arith_method == "or")
                    res = (AX_U32)a_data[m*N+n] + (AX_U32)b_data[m*N+n];
                if(arith_method == "xor")
                    res = (AX_U32)((a_data[m*N+n] - b_data[m*N+n])!=0);
                gt_data[m*N + n]  = res? 255: 0;
            }
        }
        //printf("gt=======================\n");
        //print_matrix(gt_mat);

        //printf("out======================\n");
        //print_matrix(result_matrix_2d);

        bool match = true;
        match = matrix_cmp(gt_mat, result_matrix_2d);
        if(match){
            printf("The logical opr result match with GT!\n");
        }else{
            printf("The logical opr  result mismatch with GT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        }

        release_matrix_memory(gt_mat);
    }

    release_matrix_memory(mat_a);
    release_matrix_memory(mat_b);
    release_matrix_memory(result_matrix_2d);

    AX_NPU_SDK_EX_Deinit();
    AX_SYS_Deinit();

    return 0;
}