CUR_PATH		   := $(shell pwd)
HOME_PATH		   := $(CUR_PATH)/../..
ROOTFS_TARGET_PATH := $(HOME_PATH)/msp/out

include $(HOME_PATH)/build/color.mk
include $(HOME_PATH)/build/config.mak

all:
	@echo -e $(GREEN)"\nBuild ai-sdk success!!\n"  $(DONE)


install:
	@mkdir -p $(ROOTFS_TARGET_PATH)/lib

	@cp -rf $(CUR_PATH)/sdk/lib/*.so $(ROOTFS_TARGET_PATH)/lib
	@cp -rf $(CUR_PATH)/sdk/models $(ROOTFS_TARGET_PATH)/etc
	@echo -e $(GREEN)"\nInstall ai-sdk success!!\n"  $(DONE)

clean:
	@rm -f $(ROOTFS_TARGET_PATH)/lib/libmcvsdk_*.so
	@rm -f $(ROOTFS_TARGET_PATH)/lib/libmegsearch.so
	@rm -rf $(ROOTFS_TARGET_PATH)/etc/models
	@echo -e $(GREEN)"\nClean ai-sdk success!!\n"  $(DONE)







