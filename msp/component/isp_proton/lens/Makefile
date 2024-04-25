#interface must be the last one in SUBDIRS
SUBDIRS = include  dciris 

################################################################################
#	prepare param
################################################################################
LOCAL_PATH        := $(shell pwd)
HOME_PATH         := $(LOCAL_PATH)/../../../..
OUT_PATH          := $(HOME_PATH)/msp/out
ROOTFS_TARGET_PATH	?= $(OUT_PATH)

include $(HOME_PATH)/build/color.mk
include $(HOME_PATH)/build/config.mak
################################################################################
#	set task
################################################################################
SUBDIRS_CLEAN   = $(addsuffix .clean, $(SUBDIRS))
SUBDIRS_INSTALL = $(addsuffix .install, $(SUBDIRS))

.PHONY: $(SUBDIRS) $(SUBDIRS_INSTALL) $(SUBDIRS_CLEAN)

all: $(SUBDIRS)
	@echo -e $(GREEN)"\nBuild All Sensor success!!\n"  $(DONE)

install: $(SUBDIRS_INSTALL)
	@echo -e $(GREEN)"\nInstall Sensor success!!\n"  $(DONE)

clean:	$(SUBDIRS_CLEAN)
	@echo -e $(GREEN)"\nClean Sensor success!!\n"  $(DONE)

$(SUBDIRS):
	@$(ECHO)
	@$(ECHO) -e $(CYAN)"In subdir $@ ..." $(DONE)
	@make -C $(basename $@ )

$(SUBDIRS_INSTALL):
	@$(ECHO)
	@$(ECHO) -e $(CYAN)"In subdir $(basename $@ )..." $(DONE)
	@ make -C $(basename $@ ) install

$(SUBDIRS_CLEAN):
	@$(ECHO) -e $(CYAN)"In subdir $(basename $@ )..." $(DONE)
	@make -C $(basename $@ ) clean


# Export val
export ROOTFS_TARGET_PATH
