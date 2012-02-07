# Makefile for Harris Corner Detector
# can compile HCD for the host system using the standard g++ compiler
# or cross-compile HCD for another platform (ARM)
# also compiles the sources for the DSP
#
# possible targets:
#	arm-slow			builds the pure ARM version without optimizations (very slow)
#	arm-intimg			builds the pure ARM version with integral image
#	arm-harris			builds the pure ARM version with Harris-based NCC, but without optimizations
#	arm-harris-intimg	builds the pure ARM version with Harris-based NCC and with integral image
#	dsp-slow			builds the ARM+DSP version without optimizations (slow)
#	dsp-intimg			builds the ARM+DSP version with integral image
#	dsp-harris			builds the ARM+DSP version with Harris-based NCC, but without optimizations
#	dsp-harris-intimg	builds the ARM+DSP version with Harris-based NCC and with integral image
#	dsp-generic			builds the DSP only library
#


OPTIMIZATION_LEVEL = 2
DSP_API = 2

BEAGLEBOARD_ADDRESS = beagleboard
#BEAGLEBOARD_ADDRESS = 10.1.0.66

# pretty print
#V = 1

ifndef V
QUIET_CC    = @echo '   CC         '$@;
QUIET_LINK  = @echo '   LINK       '$@;
QUIET_CLEAN = @echo '   CLEAN      '$@;
QUIET_TICC  = @echo '   DSPCC      '$@;
QUIET_TILINK= @echo '   DSPLINK    '$@;
QUIET_DLL   = @echo '   DLLCREATE  '$@;
endif


# -------------------------------------------------------------------------------
# ARM compilation stuff

# ubuntu version
#CROSS_PREFIX = arm-linux-gnueabi-

# codesourcery version
CROSS_PREFIX = /opt/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-linux-gnueabi-

CC = g++
LD_FLAGS = `GraphicsMagick++-config --libs`
CC_FLAGS = -I/usr/include/GraphicsMagick -O$(OPTIMIZATION_LEVEL) -g3 -Wall `GraphicsMagick++-config --cppflags` -c -fmessage-length=0 -D_GNU_SOURCE -DDSP_API=$(DSP_API)

LD_FLAGS_ARM = -Lopt/lib -lGraphicsMagick++ -lGraphicsMagick -llcms -ltiff -lfreetype -ljasper -ljpeg -lpng -lwmflite -lXext -lSM -lICE -lX11 -lbz2 -lxml2 -lz -lm -lgomp -lpthread -lltdl -luuid -lxcb -lXau -lXdmcp
CC_FLAGS_ARM = -Iopt/include/GraphicsMagick -O$(OPTIMIZATION_LEVEL) -g3 -Wall -c -fmessage-length=0 -D_GNU_SOURCE -DDSP_API=$(DSP_API)


# -------------------------------------------------------------------------------
# DSP compilation stuff

DSP_TOOLS := /opt/TI/C6000CGT7.2.2
DSP_DOFFBUILD := /opt/doffbuild
CL6X := $(DSP_TOOLS)/bin/cl6x
LNK6X := $(DSP_TOOLS)/bin/lnk6x
DLLCREATE := $(DSP_DOFFBUILD)/bin/DLLcreate

DSP_CC_FLAGS := --std_lib_func_defined --silicon_version=6400+ -Iopt/dsplib/inc/ -o2 -mh32 -s --consultant -D_GNU_SOURCE -DDSP_API=$(DSP_API)
#DSP_CC_FLAGS := -Iopt/dsplib/inc/ -DDSP_API=$(DSP_API)
DSP_LD_FLAGS := --silicon_version=6400+ -lopt/dsplib/lib/dsplib.a64P  # must be the LAST argument for lnk6x!
#DSP_LD_FLAGS := -lopt/dsplib/lib/dsplib.a64P
DSP_INCLUDES := -I$(DSP_TOOLS)/include/

DSP_OUT_DIR := ./bin/dsp
DSP_EXECUTABLE := harris.dll64P


# build rules for DSP stuff
./bin/dsp/harris_bridge.o64P: ./src/arm_dsp/dsp/harris_bridge.s
	$(QUIET_TICC)$(CL6X) $(DSP_CC_FLAGS) $(DSP_INCLUDES) -mv=64p -eo.o64P --obj_directory $(DSP_OUT_DIR) --asm_directory $(DSP_OUT_DIR) -c $<

./bin/dsp/harris_dsp.o64P: ./src/arm_dsp/dsp/harris_dsp.c
	$(QUIET_TICC)$(CL6X) $(DSP_CC_FLAGS) $(DSP_INCLUDES) -mv=64p -eo.o64P --obj_directory $(DSP_OUT_DIR) --asm_directory $(DSP_OUT_DIR) -c $<

./bin/dsp/harris.x64P: ./bin/dsp/harris_dsp.o64P ./bin/dsp/harris_bridge.o64P
	$(QUIET_TILINK)$(LNK6X) -r -cr --localize='$$bss' -o $@ $+ $(DSP_LD_FLAGS)

./bin/dsp/harris.dll64P: ./bin/dsp/harris.x64P
	$(QUIET_DLL)$(DLLCREATE) $< -o=$@

# DSP target
dsp-generic: $(DSP_OUT_DIR)/$(DSP_EXECUTABLE)


# -------------------------------------------------------------------------------
# other targets of Makefile


# source files, for ARM-only version
CPP_SRCS_ARM = \
./src/util/Logger.cpp \
./src/util/ConsoleLogger.cpp \
./src/util/FileLogger.cpp \
./src/util/TimeMeasureBase.cpp \
./src/util/HarrisCornerPoint.cpp \
./src/pure_arm/ImageBitstream.cpp \
./src/util/FeatureDescriptor.cpp \
./src/pure_arm/FeatureDetector.cpp \
./src/pure_arm/FeatureDetector_IntImg.cpp \
./src/pure_arm/FeatureDetector_Harris_Std.cpp \
./src/pure_arm/FeatureDetector_Harris_IntImg.cpp \
./src/util/FeatureGenerator.cpp \
./src/pure_arm/GaussFilter.cpp \
./src/pure_arm/NonMaxSuppressor.cpp \
./src/pure_arm/HarrisCornerDetector.cpp \
./src/arm_dsp/arm/dspbridge/dsp_bridge.cpp \
./src/arm_dsp/arm/dspbridge/Dsp.cpp \
./src/arm_dsp/arm/HarrisCornerDetector_DSP.cpp \
./src/main.cpp 

OBJS_ARM = \
./bin/Logger.o \
./bin/ConsoleLogger.o \
./bin/FileLogger.o \
./bin/TimeMeasureBase.o \
./bin/HarrisCornerPoint.o \
./bin/ImageBitstream.o \
./bin/FeatureDescriptor.o \
./bin/FeatureDetector.o \
./bin/FeatureDetector_IntImg.o \
./bin/FeatureDetector_Harris_Std.o \
./bin/FeatureDetector_Harris_IntImg.o \
./bin/FeatureGenerator.o \
./bin/GaussFilter.o \
./bin/NonMaxSuppressor.o \
./bin/HarrisCornerDetector.o \
./bin/dsp_bridge.o \
./bin/Dsp.o \
./bin/HarrisCornerDetector_DSP.o \
./bin/main.o 


BIN = ./bin/HarrisDetector


DEPLOYDIR = ./bin/deploy


# quick targets
all: HarrisCornerDetectorHost

# quick targets for ARM platform with DSP support
arm-dsp-harris: CC_FLAGS_ARM += -DFEATDET_USE_NCCHARRISSTD -DHARRIS_USE_DSP #-DFEATDET_USE_DSP
arm-dsp-harris: hcd-arm-dsp

arm-dsp-harris-intimg: CC_FLAGS_ARM += -DFEATDET_USE_NCCHARRISINTIMG -DHARRIS_USE_DSP #-DFEATDET_USE_DSP
arm-dsp-harris-intimg: hcd-arm-dsp

# quick targets for ARM platform without DSP
arm-slow: CC_FLAGS_ARM += -DFEATDET_USE_NCCSTD
arm-slow: hcd-arm-nondsp

arm-intimg: CC_FLAGS_ARM += -DFEATDET_USE_NCCINTIMG
arm-intimg: hcd-arm-nondsp

arm-harris: CC_FLAGS_ARM += -DFEATDET_USE_NCCHARRISSTD
arm-harris: hcd-arm-nondsp

arm-harris-intimg: CC_FLAGS_ARM += -DFEATDET_USE_NCCHARRISINTIMG
arm-harris-intimg: hcd-arm-nondsp

# quick targets for host platform (x86) (mainly for testing)
host-slow: CC_FLAGS += -DFEATDET_USE_NCCSTD
host-slow: HarrisCornerDetectorHost

host-intimg: CC_FLAGS += -DFEATDET_USE_NCCINTIMG
host-intimg: HarrisCornerDetectorHost

host-harris: CC_FLAGS += -DFEATDET_USE_NCCHARRISSTD
host-harris: HarrisCornerDetectorHost

host-harris-intimg: CC_FLAGS += -DFEATDET_USE_NCCHARRISINTIMG
host-harris-intimg: HarrisCornerDetectorHost


# platform targets
HarrisCornerDetectorHost: buildHarrisCornerDetectorHost

hcd-arm-nondsp: buildHarrisCornerDetectorARM deploybeagleboard

hcd-arm-dsp: buildHarrisCornerDetectorARM dsp-generic deploybeagleboard


# compile for host machine
buildHarrisCornerDetectorHost: CCC = $(CC)
buildHarrisCornerDetectorHost: $(OBJS_ARM)
	$(QUIET_LINK)$(CCC) $(LD_FLAGS) -o"$(BIN)" $(OBJS_ARM)


# compile using cross compiler (for ARM)
buildHarrisCornerDetectorARM: CCC = $(CROSS_PREFIX)$(CC)
buildHarrisCornerDetectorARM: LD_FLAGS = $(LD_FLAGS_ARM)
buildHarrisCornerDetectorARM: CC_FLAGS = $(CC_FLAGS_ARM)
buildHarrisCornerDetectorARM: $(OBJS_ARM)
	$(QUIET_LINK)$(CCC) $(LD_FLAGS) -o"$(BIN)" $(OBJS_ARM)


# build targets for ARM only version
./bin/main.o: ./src/main.cpp $(CPP_SRCS_ARM)
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

# utils
./bin/ConsoleLogger.o: ./src/util/ConsoleLogger.cpp ./src/util/ConsoleLogger.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FileLogger.o: ./src/util/FileLogger.cpp ./src/util/FileLogger.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/Logger.o: ./src/util/Logger.cpp ./src/util/Logger.h ./src/util/ConsoleLogger.cpp ./src/util/ConsoleLogger.h ./src/util/FileLogger.cpp ./src/util/FileLogger.h ./src/util/Logger.h ./src/util/Logger.cpp
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/TimeMeasureBase.o: ./src/util/TimeMeasureBase.cpp ./src/util/TimeMeasureBase.h ./src/util/SystemTimeMeasure.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

# generic
./bin/ImageBitstream.o: ./src/pure_arm/ImageBitstream.cpp ./src/pure_arm/ImageBitstream.h ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/HarrisCornerPoint.o: ./src/util/HarrisCornerPoint.cpp ./src/util/HarrisCornerPoint.h ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

# ARM version
./bin/HarrisCornerDetector.o: ./src/pure_arm/HarrisCornerDetector.cpp ./src/pure_arm/HarrisCornerDetector.h ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/GaussFilter.o: ./src/pure_arm/GaussFilter.cpp ./src/pure_arm/GaussFilter.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/NonMaxSuppressor.o: ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDescriptor.o: ./src/util/FeatureDescriptor.cpp ./src/util/FeatureDescriptor.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureGenerator.o: ./src/util/FeatureGenerator.cpp ./src/util/FeatureGenerator.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDetector.o: ./src/pure_arm/FeatureDetector.cpp ./src/pure_arm/FeatureDetector.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDetector_IntImg.o: ./src/pure_arm/FeatureDetector_IntImg.cpp ./src/pure_arm/FeatureDetector_IntImg.h ./src/pure_arm/FeatureDetector.cpp ./src/pure_arm/FeatureDetector.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDetector_Harris_Std.o: ./src/pure_arm/FeatureDetector_Harris_Std.cpp ./src/pure_arm/FeatureDetector_Harris_Std.h ./src/pure_arm/FeatureDetector.cpp ./src/pure_arm/FeatureDetector.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp ./src/util/FeatureGenerator.h ./src/util/FeatureGenerator.cpp ./src/pure_arm/HarrisCornerDetector.cpp ./src/pure_arm/HarrisCornerDetector.h ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDetector_Harris_IntImg.o: ./src/pure_arm/FeatureDetector_Harris_IntImg.cpp ./src/pure_arm/FeatureDetector_Harris_IntImg.h ./src/pure_arm/FeatureDetector_IntImg.cpp ./src/pure_arm/FeatureDetector_IntImg.h ./src/pure_arm/FeatureDetector.cpp ./src/pure_arm/FeatureDetector.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp ./src/util/FeatureGenerator.h ./src/util/FeatureGenerator.cpp ./src/pure_arm/HarrisCornerDetector.cpp ./src/pure_arm/HarrisCornerDetector.h ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

# DSP utils
./bin/dsp_bridge.o: ./src/arm_dsp/arm/dspbridge/dsp_bridge.cpp ./src/arm_dsp/arm/dspbridge/dsp_bridge.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/Dsp.o: ./src/arm_dsp/arm/dspbridge/Dsp.cpp ./src/arm_dsp/arm/dspbridge/Dsp.h ./src/arm_dsp/arm/dspbridge/dsp_bridge.h ./src/arm_dsp/arm/dspbridge/dsp_bridge.cpp ./src/arm_dsp/arm/dspbridge/dmm_buffer.h ./src/arm_dsp/arm/dspbridge/node.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<

# DSP version
./bin/HarrisCornerDetector_DSP.o: ./src/arm_dsp/arm/HarrisCornerDetector_DSP.cpp ./src/arm_dsp/arm/HarrisCornerDetector_DSP.h ./src/pure_arm/HarrisCornerDetector.cpp ./src/pure_arm/HarrisCornerDetector.h ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/arm_dsp/common/harris_common.h ./src/pure_arm/FixedArithmetic.h
	$(QUIET_CC)$(CCC) $(CC_FLAGS) -o"$@" $<


.PHONY: deploybeagleboard

deploybeagleboard:
	@echo deploying files to beagleboard...
	mkdir -p $(DEPLOYDIR)
	cp $(BIN) $(DEPLOYDIR)
	cp $(DSP_OUT_DIR)/$(DSP_EXECUTABLE) $(DEPLOYDIR)
	rsync -avze --progress --inplace --rsh='ssh' $(DEPLOYDIR)/* ubuntu@$(BEAGLEBOARD_ADDRESS):/home/ubuntu/harris


.PHONY: clean

clean:
	$(QUIET_CLEAN)rm -rf $(BIN) $(OBJS_ARM)
	$(QUIET_CLEAN)rm -rf ./bin/dsp/*

.PHONY: dsp-clean

dsp-clean:
	$(QUIET_CLEAN)rm -rf ./bin/dsp/*

