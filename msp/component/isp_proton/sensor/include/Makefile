CUR_PATH		:= $(shell pwd)
HOME_PATH		:= $(CUR_PATH)/../../../../..
OUT_PATH        := $(HOME_PATH)/msp/out
SRC_PATH		:= $(CUR_PATH)
ROOTFS_TARGET_PATH ?= $(HOME_PATH)/msp/out

include $(HOME_PATH)/build/config.mak

# output
MOD_NAME 		:=
OUTPUT 			:= .obj

# source
SRCS 			:=
CINCLUDE		:=	-I$(OUT_PATH)/include

OBJS 			:= $(SRCS:%.c=$(OUTPUT)/%.o)
DEPS			:= $(OBJS:%.o=%.d)

# build flags
CFLAGS			:= -shared -fPIC
ifeq ($(debug),yes)
CFLAGS			+= -Wall -O0 -g
else
CFLAGS			+= -Wall -O2
endif

# dependency

# install
INSTALL_INC		+= $(SRC_PATH)/ax_sensor_struct.h
# link
LINK = $(CC)

include $(HOME_PATH)/build/rules.mak
