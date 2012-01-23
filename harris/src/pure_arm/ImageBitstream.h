/*
 * ImageBitstream.h
 *
 *  Created on: 19.08.2011
 *      Author: sn
 */

#ifndef IMAGEBITSTREAM_H_
#define IMAGEBITSTREAM_H_

#include <cstring>
#include <Magick++.h>
#include "FixedArithmetic.h"

using namespace std;
using namespace Magick;

/**
 * @class ImageBitstream
 * is an image converted to grayscale and represented as uchar array
 * 1 pixel equals 1 array element
 */
class ImageBitstream
{
public:

	ImageBitstream();
	ImageBitstream(const ImageBitstream &original);  // copy constructor
	ImageBitstream(Magick::Image img);
	ImageBitstream(string filename);
	ImageBitstream(int width, int height);
	ImageBitstream(float *original, int width, int height, bool scale = true);
	virtual ~ImageBitstream();

	void setImage(Magick::Image img);
	void setImage(string filename);
	Magick::Image getImage();
	void saveImage(string filename);
	bool isLoaded();

	int getWidth();
	int getHeight();
	unsigned char* getBitstream();
	unsigned char* copyBitstream();
	unsigned char& pixel(int row, int col);

	ImageBitstream extend(int borderSize);
	ImageBitstream convolve(float *kernel, int kernelSize);
	ImageBitstream stretchContrast();

	ImageBitstream& operator=(const ImageBitstream& original);

	static float* extend(float *input, int width, int height, int borderSize);
	static Fixed* extend(Fixed *input, int width, int height, int borderSize);
	static int* extend(int *input, int width, int height, int borderSize);
	static unsigned char* extend(unsigned char *input, int width, int height, int borderSize);
	static short* extend(short *input, int width, int height, int borderSize);

private:

	unsigned char *bitstream_;
	int width_;
	int height_;

	unsigned char* extendImage(int borderSize);
};

#endif /* IMAGEBITSTREAM_H_ */
