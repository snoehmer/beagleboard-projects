# Makefile for Harris Corner Detector
# can compile HCD for the host system using the standard g++ compiler
# or cross-compile HCD for another platform (ARM)
# also compiles the sources for the DSP

OPTIMIZATION_LEVEL = 3

# ubuntu version
#CROSS_PREFIX = arm-linux-gnueabi-

# codesourcery version
CROSS_PREFIX = /opt/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-linux-gnueabi-

CC = g++
LD_FLAGS = `GraphicsMagick++-config --libs`
CC_FLAGS = -I/usr/include/GraphicsMagick -O$(OPTIMIZATION_LEVEL) -g3 -Wall `GraphicsMagick++-config --cppflags` -c -fmessage-length=0

LD_FLAGS_ARM = -Lopt/lib -lGraphicsMagick++ -lGraphicsMagick -llcms -ltiff -lfreetype -ljasper -ljpeg -lpng -lwmflite -lXext -lSM -lICE -lX11 -lbz2 -lxml2 -lz -lm -lgomp -lpthread -lltdl -luuid -lxcb -lXau -lXdmcp
CC_FLAGS_ARM = -Iopt/include/GraphicsMagick -O$(OPTIMIZATION_LEVEL) -g3 -Wall -c -fmessage-length=0

# source files, TODO
CPP_SRCS = \
./src/util/Logger.cpp \
./src/util/ConsoleLogger.cpp \
./src/util/FileLogger.cpp \
./src/util/TimeMeasureBase.cpp \
./src/util/HarrisCornerPoint.cpp \
./src/pure_arm/ImageBitstream.cpp \
./src/util/FeatureDescriptor.cpp \
./src/pure_arm/FeatureDetector.cpp \
./src/util/FeatureGenerator.cpp \
./src/pure_arm/GaussFilter.cpp \
./src/pure_arm/NonMaxSuppressor.cpp \
./src/pure_arm/HarrisCornerDetector.cpp \
./src/main.cpp 


OBJS = \
./bin/Logger.o \
./bin/ConsoleLogger.o \
./bin/FileLogger.o \
./bin/TimeMeasureBase.o \
./bin/HarrisCornerPoint.o \
./bin/ImageBitstream.o \
./bin/FeatureDescriptor.o \
./bin/FeatureDetector.o \
./bin/FeatureGenerator.o \
./bin/GaussFilter.o \
./bin/NonMaxSuppressor.o \
./bin/HarrisCornerDetector.o \
./bin/main.o 

BIN = ./bin/HarrisDetector

all: HarrisCornerDetectorHost

# compile for host machine
HarrisCornerDetectorHost: CCC = $(CC)
HarrisCornerDetectorHost: $(OBJS)
	$(CCC) $(LD_FLAGS) -o"$(BIN)" $(OBJS)

# compile using cross compiler (for ARM)
HarrisCornerDetectorARM: CCC = $(CROSS_PREFIX)$(CC)
HarrisCornerDetectorARM: LD_FLAGS = $(LD_FLAGS_ARM)
HarrisCornerDetectorARM: CC_FLAGS = $(CC_FLAGS_ARM)
HarrisCornerDetectorARM: $(OBJS)
	$(CCC) $(LD_FLAGS) -o"$(BIN)" $(OBJS)


# build targets for ARM only version
./bin/main.o: ./src/main.cpp $(CPP_SRCS)
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/ConsoleLogger.o: ./src/util/ConsoleLogger.cpp ./src/util/ConsoleLogger.h
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FileLogger.o: ./src/util/FileLogger.cpp ./src/util/FileLogger.h
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/Logger.o: ./src/util/Logger.cpp ./src/util/Logger.h ./src/util/ConsoleLogger.cpp ./src/util/ConsoleLogger.h ./src/util/FileLogger.cpp ./src/util/FileLogger.h ./src/util/Logger.h ./src/util/Logger.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/TimeMeasureBase.o: ./src/util/TimeMeasureBase.cpp ./src/util/TimeMeasureBase.h ./src/util/SystemTimeMeasure.h
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/ImageBitstream.o: ./src/pure_arm/ImageBitstream.cpp ./src/pure_arm/ImageBitstream.h
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/HarrisCornerPoint.o: ./src/util/HarrisCornerPoint.cpp ./src/util/HarrisCornerPoint.h
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/HarrisCornerDetector.o: ./src/pure_arm/HarrisCornerDetector.cpp ./src/pure_arm/HarrisCornerDetector.h ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/GaussFilter.o: ./src/pure_arm/GaussFilter.cpp ./src/pure_arm/GaussFilter.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/NonMaxSuppressor.o: ./src/pure_arm/NonMaxSuppressor.cpp ./src/pure_arm/NonMaxSuppressor.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDescriptor.o: ./src/util/FeatureDescriptor.cpp ./src/util/FeatureDescriptor.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureGenerator.o: ./src/util/FeatureGenerator.cpp ./src/util/FeatureGenerator.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/HarrisCornerPoint.h ./src/util/HarrisCornerPoint.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<

./bin/FeatureDetector.o: ./src/pure_arm/FeatureDetector.cpp ./src/pure_arm/FeatureDetector.h ./src/pure_arm/ImageBitstream.h ./src/pure_arm/ImageBitstream.cpp ./src/util/FeatureDescriptor.h ./src/util/FeatureDescriptor.cpp
	$(CCC) $(CC_FLAGS) -o"$@" $<


.PHONY: clean

clean:
	rm -rf $(BIN) $(OBJS)

