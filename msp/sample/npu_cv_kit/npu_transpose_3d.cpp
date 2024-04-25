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
    args.add<AX_U32>("N", '\0', "count of matrixs", false, 1);
    args.add<AX_U32>("row", '\0', "rows of matrix", true);
    args.add<AX_U32>("column", '\0', "columns of matrix", true);
    args.add<std::string> ("data-type", '\0', "uint8,int8,float", false, "uint8");
    args.add<AX_U32>("repeat", '\0', "execution count", false, 1);
    args.add<bool>("check", '\0', "Check outputs", false, false);
    args.add<std::string> ("output", '\0', "output dir", false);

    args.parse_check(argc, argv);

    // LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH ./bin/npu_transpose.ax620a  --mode=disable --mode-type=disable --data-type=float --row 128 --column 128 --repeat=100 --check=1

    AX_NPU_SDK_EX_ATTR_T hard_mode = get_npu_hard_mode(args.get<std::string>("mode"));

    assert(AX_SYS_Init() == 0);
    assert(AX_NPU_SDK_EX_Init_with_attr(&hard_mode) == 0);

    int nRepeat = args.get<AX_U32>("repeat");
    bool bCheck = args.get<bool>("check");
    std::string matrix_a = args.get<std::string>("A");
    int N = args.get<AX_U32>("N");
    int nRow = args.get<AX_U32>("row");
    int nColumn = args.get<AX_U32>("column");
    AX_NPU_CV_DataType data_type = get_data_type(args.get<std::string>("data-type"));
    std::string output = args.get<std::string>("output");

    if(data_type != AX_NPU_CV_DT_UINT8 && data_type != AX_NPU_CV_DT_INT8 && data_type != AX_NPU_CV_DT_FLOAT32){
        printf("unsupported data type %d for matrix\n", (int)data_type);
        assert(0);
    }

    AX_NPU_CV_Matrix3D a_matrix_3d = create_matrix_universal_3d(matrix_a, N, nRow, nColumn, data_type);

    printf("Input N %d, row %d, column %d, repeat %d\n", N, nRow, nColumn, nRepeat);

    srand(time(0));

    AX_NPU_CV_Matrix3D result_matrix_3d = create_matrix_by_size_3d(N, nColumn, nRow, data_type);

    AX_NPU_SDK_EX_MODEL_TYPE_T virtual_npu_mode_type = get_npu_mode_type(args.get<std::string>("mode-type"));

    auto time_start = std::chrono::system_clock::now();
    for (int i = 0; i < nRepeat; ++i) {
        assert(0 == AX_NPU_CV_Transpose_3Dim(virtual_npu_mode_type, &a_matrix_3d, &result_matrix_3d));
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
            sprintf(fname, "%s/npu_transpose_3d.N_%d.row_%d.column_%d.npu_mode_%d.%s",
                output.c_str(),
                N, nRow, nColumn,
                virtual_npu_mode_type,
                get_data_type_name(data_type).c_str());
            dump_matrix_to_file_3d(fname, result_matrix_3d);
        }
    }

    if (bCheck) {
        //print_matrix_3d(a_matrix_3d);
        // Calculate GT
        matrix_transpose_3d(a_matrix_3d);
        AX_NPU_CV_Matrix3D gt_matrix = a_matrix_3d;
        //printf("gt data---------------------->\n");
        //print_matrix_3d(gt_matrix);

        printf("**************************************************\n");
        printf("**************************************************\n");

        //printf("dump transpose results:\n");
        //print_matrix_3d(result_matrix_3d);

        // Check GT with result
        bool match_gt = true;
        match_gt = matrix_cmp_3d(gt_matrix, result_matrix_3d);
        if (match_gt) {
            printf("The transpose result match with GT!\n");
        } else {
            printf("The transpose result mismatch with GT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        }
    }

    release_matrix_memory_3d(a_matrix_3d);
    release_matrix_memory_3d(result_matrix_3d);

    AX_NPU_SDK_EX_Deinit();
    AX_SYS_Deinit();

    return 0;
}
