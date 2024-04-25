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

#define max(a, b) ((a) > (b) ? (a):(b))
#define min(a, b) ((a) < (b) ? (a):(b))

int main(int argc, char* argv[]) {

    printf("NPU-CV-KIT Version:%s\n", AX_NPU_CV_Version());

    cmdline::parser args;
    args.add<std::string> ("mode", '\0', "NPU hard mode: disable, 1_1", false, "disable");
    args.add<std::string> ("mode-type", '\0', "Virtual NPU mode type: disable， 1_1_1， 1_1_2", false, "disable");
    args.add<std::string> ("A", '\0', "matrix file", false);
    args.add<AX_U32>("row", '\0', "rows of matrix", true);
    args.add<AX_U32>("column", '\0', "columns of matrix", true);
    args.add<std::string> ("data-type", '\0', "uint8", false, "uint8");
    args.add<AX_U32>("repeat", '\0', "execution count", false, 1);
    args.add<bool>("check", '\0', "Check outputs", false, false);
    args.add<std::string> ("output", '\0', "output dir", false);

    args.parse_check(argc, argv);

    // LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH ./bin/npu_dilate.ax620a  --mode=disable --mode-type=disable --data-type=uint8 --row 128 --column 128 --repeat=100 --check=1

    AX_NPU_SDK_EX_ATTR_T hard_mode = get_npu_hard_mode(args.get<std::string>("mode"));

    assert(AX_SYS_Init() == 0);
    assert(AX_NPU_SDK_EX_Init_with_attr(&hard_mode) == 0);

    int nRepeat = args.get<AX_U32>("repeat");
    bool bCheck = args.get<bool>("check");
    std::string matrix_a = args.get<std::string>("A");
    int nRow = args.get<AX_U32>("row");
    int nColumn = args.get<AX_U32>("column");
    AX_NPU_CV_DataType data_type = get_data_type(args.get<std::string>("data-type"));
    std::string output = args.get<std::string>("output");

    if(data_type != AX_NPU_CV_DT_UINT8){
        printf("unsupported data type %d for matrix\n", (int)data_type);
        assert(0);
    }

    AX_NPU_CV_Matrix2D a_matrix_2d = create_matrix_universal(matrix_a, nRow, nColumn, data_type);

    printf("Input row %d, column %d, repeat %d\n", nRow, nColumn, nRepeat);

    srand(time(0));

    AX_NPU_CV_Matrix2D result_matrix_2d = create_matrix_by_size(nRow, nColumn, data_type);

    AX_NPU_SDK_EX_MODEL_TYPE_T virtual_npu_mode_type = get_npu_mode_type(args.get<std::string>("mode-type"));

    AX_NPU_CV_Erode_Context context;
    memset(&context, 0, sizeof(context));

    AX_NPU_CV_ErodeParam param;
    std::vector<AX_U8> mat33 = {255,0,255,0,0,0,255,0,255};
    memcpy(param.nKernelValue, mat33.data(), sizeof(param));

    auto time_start = std::chrono::system_clock::now();
    for (int i = 0; i < nRepeat; ++i) {
        assert(0 == AX_NPU_CV_Erode(&context, virtual_npu_mode_type, &a_matrix_2d, &result_matrix_2d, &param));
        if(nRepeat > 1 && i == 0){
            time_start = std::chrono::system_clock::now();
        }
    }
    auto time_end = std::chrono::system_clock::now();
    if (nRepeat > 1) nRepeat--;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start) / nRepeat;
    AX_NPU_SDK_EX_WORK_CYCLE_T cur = get_current_work_cycle();
    print_work_cycle(cur);
    AX_NPU_CV_DestroyErodeContext(context);
    printf("\n**************************************************\n");
    printf("Run task took %lld us (%d rounds for average)\n", duration.count(), nRepeat);
    printf("**************************************************\n");

     if(!output.empty()){
        struct stat info;
        if (stat(output.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
            fprintf(stderr, "cannot access directory '%s'\n", output.c_str());
        }else{
            char fname[256];
            sprintf(fname, "%s/npu_erode.row_%d.column_%d.npu_mode_%d.%s",
                output.c_str(),
                nRow, nColumn,
                virtual_npu_mode_type,
                get_data_type_name(data_type).c_str());
            dump_matrix_to_file(fname, result_matrix_2d);
        }
    }

    if (bCheck) {
        //print_matrix(a_matrix_2d);
        // Calculate GT
        int kernel_size = 3;
        AX_NPU_CV_Matrix2D gt_matrix = create_matrix_by_size(nRow, nColumn, data_type);
        AX_U8* gt_data = (AX_U8*)gt_matrix.pVir;
        memset(gt_data, 0, nRow*nColumn*get_data_bit_width(gt_matrix.eDataType)/8);

        AX_U8 *a_data = (AX_U8 *)a_matrix_2d.pVir;
        //printf("gt data---------------------->\n");
        for (int r = 0; r < nRow; ++r) {
            for (int c = 0; c < nColumn; ++c) {
                int sum = 1;
                for(int j = -kernel_size/2; j <= kernel_size/2; j++){
                    for(int i = -kernel_size/2; i <= kernel_size/2; i++){
                        if(r+j >= 0 && r+j < nRow && c+i >= 0 && c+i < nColumn){
                            AX_U8 data = a_data[(r+j)*nColumn + c+i];
                            sum *= (mat33[j*3 + i + kernel_size/2*4] + data);
                            if(sum > 0) sum = 1;
                        }
                    }
                }
                if(sum > 0){
                    gt_data[r*nColumn + c] = 255;
                }

            }
        }

        //print_matrix(gt_matrix);

        printf("**************************************************\n");
        printf("**************************************************\n");

        //printf("dump erode results:\n");
        //print_matrix(result_matrix_2d);

        // Check GT with result
        bool match_gt = true;
        match_gt = matrix_cmp(gt_matrix, result_matrix_2d);
        if (match_gt) {
            printf("The erode result match with GT!\n");
        } else {
            printf("The erode result mismatch with GT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        }

        release_matrix_memory(gt_matrix);
    }

    release_matrix_memory(a_matrix_2d);
    release_matrix_memory(result_matrix_2d);

    AX_NPU_SDK_EX_Deinit();
    AX_SYS_Deinit();

    return 0;
}
