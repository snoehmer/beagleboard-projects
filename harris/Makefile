# Makefile for Harris Corner Detector on x86

CC = g++
LD_FLAGS = `GraphicsMagick++-config --libs`
CC_FLAGS = -I/usr/include/GraphicsMagick -O0 -g3 -Wall `GraphicsMagick++-config --cppflags` -c -fmessage-length=0

# source files, TODO
CPP_SRCS = \
./src/util/HarrisCornerPoint.cpp \
./src/pure_arm/ImageBitstream.cpp \
./src/pure_arm/GaussFilter.cpp \
./src/pure_arm/HarrisCornerDetector.cpp \
./src/main.cpp 

OBJS = \
./bin/HarrisCornerPoint.o \
./bin/ImageBitstream.o \
./bin/GaussFilter.o \
./bin/HarrisCornerDetector.o \
./bin/main.o 

BIN = ./bin/HarrisDetector

all: HarrisCornerDetectorx86

HarrisCornerDetectorx86: $(OBJS)
	$(CC) $(LD_FLAGS) -o"$(BIN)" $(OBJS)

./bin/main.o: ./src/main.cpp
	$(CC) $(CC_FLAGS) -o"$@" $<

./bin/ImageBitstream.o: ./src/pure_arm/ImageBitstream.cpp 
	$(CC) $(CC_FLAGS) -o"$@" $<

./bin/HarrisCornerPoint.o: ./src/util/HarrisCornerPoint.cpp
	$(CC) $(CC_FLAGS) -o"$@" $<

./bin/HarrisCornerDetector.o: ./src/pure_arm/HarrisCornerDetector.cpp
	$(CC) $(CC_FLAGS) -o"$@" $<

./bin/GaussFilter.o: ./src/pure_arm/GaussFilter.cpp
	$(CC) $(CC_FLAGS) -o"$@" $<

.PHONY: clean
clean:
	rm -rf $(BIN) $(OBJS)
