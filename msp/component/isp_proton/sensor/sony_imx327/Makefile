CUR_PATH				:= $(shell pwd)
HOME_PATH				:= $(CUR_PATH)/../../../../..
BASE_PATH		        := $(HOME_PATH)/msp/component
OUT_PATH                := $(HOME_PATH)/msp/out
ROOTFS_TARGET_PATH 		?= $(OUT_PATH)

include $(HOME_PATH)/build/color.mk
include $(HOME_PATH)/build/config.mak

################################################################################
# build
################################################################################

.PHONY : all clean install lib
all: lib
	@echo -e $(GREEN)"\nBuild $@ success!!\n"  $(DONE)

clean:
	$(VERB) $(MAKE) -f Makefile.dynamic clean
	$(VERB) $(MAKE) -f Makefile.static clean
	@echo -e $(GREEN)"\nClean $@ success!!\n"  $(DONE)

install:
	@echo -e $(GREEN)"install $@ ..."  $(DONE)
	$(VERB) $(MAKE) -f Makefile.dynamic install
	$(VERB) $(MAKE) -f Makefile.static install

lib:
	@echo -e $(GREEN)"build dynamic $@ ..."  $(DONE)
	$(VERB) $(MAKE) -f Makefile.dynamic

	@echo -e $(GREEN)"build static $@ ..."  $(DONE)
	$(VERB) $(MAKE) -f Makefile.static

export ROOTFS_TARGET_PATH
