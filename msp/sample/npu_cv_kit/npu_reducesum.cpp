#include <cstdio>
#include <assert.h>
#include <string>
#include <chrono>
#include <memory.h>
#include <math.h>
#include <sys/stat.h>
#include "ax_sys_api.h"
#include "ax_npu_imgproc.h"
#include "cmdline.h"
#include <cfenv>
#include <cmath>
#include "img_helper.h"

int main(int argc, char* argv[]) {

    printf("NPU-CV-KIT Version:%s\n", AX_NPU_CV_Version());

    cmdline::parser args;
    args.add<std::string> ("mode", '\0', "NPU hard mode: disable, 1_1", false, "disable");
    args.add<std::string> ("mode-type", '\0', "Virtual NPU mode type: disable， 1_1_1， 1_1_2", false, "disable");
    args.add<std::string> ("A", '\0', "matrix file", false);
    args.add<AX_U32>("row", '\0', "rows of matrix", true);
    args.add<AX_U32>("column", '\0', "columns of matrix", true);
    args.add<std::string> ("input-data-type", '\0', "uint8,int8,float", false, "uint8");
    args.add<std::string> ("output-data-type", '\0', "uint16,float", false, "uint16");
    args.add<AX_U32>("repeat", '\0', "execution count", false, 1);
    args.add<bool>("check", '\0', "Check outputs", false, false);
    args.add<std::string> ("output", '\0', "output dir", false);

    args.parse_check(argc, argv);

    // LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH ./bin/npu_reducesum.ax620a  --mode=disable --mode-type=disable --data-type=float --row 128 --column 128 --repeat=100 --check=1

    AX_NPU_SDK_EX_ATTR_T hard_mode = get_npu_hard_mode(args.get<std::string>("mode"));

    assert(AX_SYS_Init() == 0);
    assert(AX_NPU_SDK_EX_Init_with_attr(&hard_mode) == 0);

    int nRepeat = args.get<AX_U32>("repeat");
    bool bCheck = args.get<bool>("check");
    std::string matrix_a = args.get<std::string>("A");
    int nRow = args.get<AX_U32>("row");
    int nColumn = args.get<AX_U32>("column");
    AX_NPU_CV_DataType data_type = get_data_type(args.get<std::string>("input-data-type"));
    std::string output = args.get<std::string>("output");

    if(data_type != AX_NPU_CV_DT_UINT8 && data_type != AX_NPU_CV_DT_INT8 && data_type != AX_NPU_CV_DT_FLOAT32){
        printf("unsupported data type %d for matrix\n", (int)data_type);
        assert(0);
    }

    AX_NPU_CV_Matrix2D a_matrix_2d = create_matrix_universal(matrix_a, nRow, nColumn, data_type);

    printf("Input row %d, column %d, repeat %d\n", nRow, nColumn, nRepeat);

    srand(time(0));

    AX_NPU_CV_DataType result_data_type = get_data_type(args.get<std::string>("output-data-type"));
    if(result_data_type != AX_NPU_CV_DT_UINT16 && result_data_type != AX_NPU_CV_DT_FLOAT32){
        printf("unsupported data type %d for matrix\n", (int)result_data_type);
        assert(0);
    }
    AX_NPU_CV_Matrix2D result_matrix_2d = create_matrix_by_size(1, nColumn, result_data_type);

    AX_NPU_SDK_EX_MODEL_TYPE_T virtual_npu_mode_type = get_npu_mode_type(args.get<std::string>("mode-type"));

    auto time_start = std::chrono::system_clock::now();
    for (int i = 0; i < nRepeat; ++i) {
        assert(0 == AX_NPU_CV_ReduceSum(virtual_npu_mode_type, &a_matrix_2d, &result_matrix_2d));
        if(nRepeat > 1 && i == 0){
            time_start = std::chrono::system_clock::now();
        }
    }
    auto time_end = std::chrono::system_clock::now();
    if (nRepeat > 1) nRepeat--;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start) / nRepeat;
    AX_NPU_SDK_EX_WORK_CYCLE_T cur = get_current_work_cycle();
    print_work_cycle(cur);
    printf("\n**************************************************\n");
    printf("Run task took %lld us (%d rounds for average)\n", duration.count(), nRepeat);
    printf("**************************************************\n");

     if(!output.empty()){
        struct stat info;
        if (stat(output.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            fprintf(stderr, "cannot access directory '%s'\n", output.c_str());
        }else{
            char fname[256];
            sprintf(fname, "%s/npu_reducesum.row_%d.column_%d.%s.npu_mode_%d.%s",
                output.c_str(),
                nRow, nColumn,
                get_data_type_name(data_type).c_str(),
                virtual_npu_mode_type,
                get_data_type_name(result_data_type).c_str());
            dump_matrix_to_file(fname, result_matrix_2d);
        }
    }

    if (bCheck) {
        //printf("src data ============\n");
        //print_matrix(a_matrix_2d);
        // Calculate GT
        AX_NPU_CV_Matrix2D gt_matrix = create_matrix_by_size(1, nColumn, result_data_type);

        matrix_transpose(a_matrix_2d);
        matrix_transpose(gt_matrix);
        matrx_stats(a_matrix_2d, gt_matrix, "sum");
        matrix_transpose(gt_matrix);

        //printf("gt data---------------------->\n");
        //print_matrix(gt_matrix);

        printf("**************************************************\n");
        printf("**************************************************\n");

        //printf("dump reducesum results:\n");
        //print_matrix(result_matrix_2d);

        // Check GT with result
        bool match_gt = true;
        match_gt = matrix_cmp(gt_matrix, result_matrix_2d);
        if (match_gt) {
            printf("The reducesum result match with GT!\n");
        } else {
            printf("The reducesum result mismatch with GT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        }

        release_matrix_memory(gt_matrix);
    }

    release_matrix_memory(a_matrix_2d);
    release_matrix_memory(result_matrix_2d);

    AX_NPU_SDK_EX_Deinit();
    AX_SYS_Deinit();

    return 0;
}
