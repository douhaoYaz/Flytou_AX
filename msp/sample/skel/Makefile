CUR_PATH			:= $(shell pwd)
HOME_PATH			:= $(CUR_PATH)/../../..
SRC_PATH			:= $(CUR_PATH)
BASE_PATH			:= $(HOME_PATH)/msp/component
OUT_PATH			:= $(HOME_PATH)/msp/out
LIB_PATH			:= $(OUT_PATH)/lib
THIRD_PARTY_PATH  	:= $(HOME_PATH)/third-party
ROOTFS_TARGET_PATH 	?= $(OUT_PATH)
include $(HOME_PATH)/build/config.mak

MOD_NAME = sample_skel
OUTPUT 				:= .obj

# source
SRCS                := $(wildcard $(SRC_PATH)/*.c)
SRCCPPS             := $(wildcard $(SRC_PATH)/*.cpp)

CINCLUDE            := -I$(SRC_PATH) \
                       -I$(OUT_PATH)/include

OBJS                := $(SRCS:%.c=$(OUTPUT)/%.o) \
                       $(SRCCPPS:%.cpp=$(OUTPUT)/%.o)

DEPS                := $(OBJS:%.o=%.d)

# exec
TARGET				:= $(MOD_NAME)

# build flags
CFLAGS				:= -Wall -Wl,-rpath-link=$(LIB_PATH)
CFLAGS              += -DSAMPLE_SKEL_BUILD_VERSION=\"$(SDK_VERSION)\"
CFLAGS              += -DSAMPLE_SKEL_NAME=\"$(TARGET)\"
CFLAGS              += -D_FILE_OFFSET_BITS=64
debug = no
ifeq ($(debug),yes)
CFLAGS				+= -O0 -ggdb3
else
CFLAGS				+= -O2
endif

CPPFLAGS            := --std=c++11 -Wall -Wl,-rpath-link=$(LIB_PATH)

# dependency
CLIB				:= -L$(LIB_PATH)
CLIB				+= -lax_sys -lax_npu_cv_kit -lax_run_joint -lax_interpreter_external -lax_interpreter
CLIB				+= -pthread -lm -ldl -lstdc++ -lstdc++fs
CLIB				+= -lax_skel
CLIB				+= -lax_vdec
CLIB				+= -lax_venc
CLIB				+= -lstdc++

# install
INSTALL_TARGET		:= $(TARGET)
INSTALL_DIR		:= $(ROOTFS_TARGET_PATH)/bin/
INSTALL_DATA    	:= $(CUR_PATH)/data/skel

# link
LINK = $(CC)

include $(HOME_PATH)/build/rules.mak
