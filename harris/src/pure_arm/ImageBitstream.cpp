/*
 * ImageBitstream.cpp
 *
 *  Created on: 19.08.2011
 *      Author: sn
 */

#include "ImageBitstream.h"
#include <cmath>
#include <Magick++.h>

using namespace std;
using namespace Magick;

ImageBitstream::ImageBitstream(int width, int height)
{
	width_ = width;
	height_ = height;

	bitstream_ = new unsigned char[width_ * height_];
}

ImageBitstream::ImageBitstream(float *original, int width, int height, bool scale)
{
	width_ = width;
	height_ = height;

	bitstream_ = new unsigned char[width_ * height_];

	int row, col;

	if(scale)  // float image should be scaled to fit the uchar range 0...255
	{
		float min, max;
		min = max = original[0];

		for(row = 0; row < height_; row++)
		{
			for(col = 0; col < width_; col++)
			{
				if(original[row * width_ + col] < min) min = original[row * width_ + col];
				if(original[row * width_ + col] > max) max = original[row * width_ + col];
			}
		}

		for(row = 0; row < height_; row++)
			for(col = 0; col < width_; col++)
				bitstream_[row * width_ + col] = (unsigned char) ((original[row * width_ + col] - min) * 255.0f / (max - min));
	}
	else  // float image is in range 0.0f...1.0f
	{
		for(row = 0; row < height_; row++)
			for(col = 0; col < width_; col++)
				bitstream_[row * width_ + col] = (unsigned char) (255.0f * original[row * width_ + col]);
	}
}

ImageBitstream ImageBitstream::convolve(float *kernel, int kernelSize)
{
	unsigned char *extendedImg;
	ImageBitstream newImg;

	if(kernel == 0 || kernelSize % 2 == 0)
		return newImg;

	int imgrow;  // current row and col in the image where the filter is calculated
	int imgcol;
	int krow;  // current row and col in the kernel for the convolution sum
	int kcol;
	int row;  // row and col of image pixel for current kernel position
	int col;

	int offset = (kernelSize - 1) / 2;

	int extWidth = width_ + 2 * offset;
	int extHeight = height_ + 2 * offset;

	float sum;

	extendedImg = extendImage(offset);

	newImg.width_ = width_;
	newImg.height_ = height_;
	newImg.bitstream_ = new unsigned char[width_ * height_];

	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sum = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < kernelSize; krow++)
			{
				for(kcol = 0; kcol < kernelSize; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sum += extendedImg[row * extWidth + col] * kernel[krow * kernelSize + kcol];
				}
			}

			newImg.bitstream_[(imgrow - offset) * width_ + (imgcol - offset)] = (unsigned char) round(sum);
		}
	}

	delete[] extendedImg;

	return newImg;
}


bool ImageBitstream::isLoaded()
{
	return (bitstream_ != 0);
}


int ImageBitstream::getHeight()
{
	return height_;
}



unsigned char & ImageBitstream::pixel(int row, int col)
{
	//if(row >= height_ || col >= width_) indexOutOfBounds!

	return bitstream_[row * width_ + col];
}



void ImageBitstream::saveImage(string filename)
{
	if(bitstream_)
	{
		Image img;

		img.read(width_, height_, "I", CharPixel, bitstream_);
		img.write(filename);
	}
}



void ImageBitstream::setImage(Magick::Image img)
{
	width_ = img.size().width();
	height_ = img.size().height();

	if(bitstream_)
		delete[] bitstream_;

	bitstream_ = new unsigned char[width_ * height_];

	img.write(0, 0, width_, height_, "I", CharPixel, bitstream_);
}



unsigned char *ImageBitstream::extendImage(int borderSize)
{
	int row;
	int col;
	unsigned char *extendedImg;

	int offset = borderSize;

	int extWidth = width_ + 2 * offset;
	int extHeight = height_ + 2 * offset;

	extendedImg = new unsigned char[extWidth * extHeight];

	// step 0: copy image
	for(row = 0; row < height_; row++)
			for(col = 0; col < width_; col++)
					extendedImg[(row + offset) * extWidth + (col + offset)] = bitstream_[row * width_ + col];

	// step 1a: copy upper border
	for(row = 0; row < offset; row++)
			for(col = 0; col < width_; col++)
					extendedImg[row * extWidth + (col + offset)] = bitstream_[0 * width_ + col];

	// step 1b: copy lower border
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = 0; col < width_; col++)
					extendedImg[row * extWidth + (col + offset)] = bitstream_[(height_ - 1) * width_ + col];

	// step 1c: copy left border
	for(col = 0; col < offset; col++)
			for(row = 0; row < height_; row++)
					extendedImg[(row + offset) * extWidth + col] = bitstream_[row * width_ + 0];

	// step 1d: copy right border
	for(col = offset + width_; col < width_ + 2*offset; col++)
			for(row = 0; row < height_; row++)
					extendedImg[(row + offset) * extWidth + col] = bitstream_[row * width_ + (width_ - 1)];

	// step 2a: copy upper left corner
	for(row = 0; row < offset; row++)
			for(col = 0; col < offset; col++)
					extendedImg[row * extWidth + col] = bitstream_[0 * width_ + 0];

	// step 2b: copy upper right corner
	for(row = 0; row < offset; row++)
			for(col = offset + width_; col < width_ + 2*offset; col++)
					extendedImg[row * extWidth + col] = bitstream_[0 * width_ + (width_ - 1)];

	// step 2c: copy lower left corner
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = 0; col < offset; col++)
					extendedImg[row * extWidth + col] = bitstream_[(height_ - 1) * width_ + 0];

	// step 2d: copy lower right corner
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = offset + width_; col < width_ + 2*offset; col++)
					extendedImg[row * extWidth + col] = bitstream_[(height_ - 1) * width_ + (width_ - 1)];

	return extendedImg;
}


void ImageBitstream::setImage(string filename)
{
	Image img;

	img.read(filename);

	setImage(img);
}


ImageBitstream & ImageBitstream::operator =(const ImageBitstream & original)
{
	width_ = original.width_;
	height_ = original.height_;

	if(bitstream_ != 0)
		delete[] bitstream_;

	bitstream_ = new unsigned char[width_ * height_];
	memcpy(bitstream_, original.bitstream_, width_ * height_ * sizeof(unsigned char));

	return *this;
}


ImageBitstream ImageBitstream::stretchContrast()
{
	ImageBitstream newImg;
	unsigned char min = 255, max = 0;
	int pixel;

	// initialize new image bitstream
	newImg.width_ = width_;
	newImg.height_ = height_;
	newImg.bitstream_ = new unsigned char[width_ * height_];

	// get contrast range
	for(pixel = 0; pixel < width_ * height_; pixel++)
	{
		if(bitstream_[pixel] < min) min = bitstream_[pixel];
		if(bitstream_[pixel] > max) max = bitstream_[pixel];
	}

	// stretch contrast
	for(pixel = 0; pixel < width_ * height_; pixel++)
	{
		newImg.bitstream_[pixel] = (unsigned char) (((int) (bitstream_[pixel] - min)) * (int) 255) / ((int) max - min);
	}

	return newImg;
}



ImageBitstream::ImageBitstream(string filename)
{
	bitstream_ = 0;
	setImage(filename);
}



unsigned char *ImageBitstream::copyBitstream()
{
	unsigned char *copy;

	copy = new unsigned char[width_ * height_];
	memcpy(copy, bitstream_, width_ * height_ * sizeof(unsigned char));

	return copy;
}



ImageBitstream::ImageBitstream(const ImageBitstream & original)
{
	width_ = original.width_;
	height_ = original.height_;

	bitstream_ = new unsigned char[width_ * height_];
	memcpy(bitstream_, original.bitstream_, width_ * height_ * sizeof(unsigned char));
}



int ImageBitstream::getWidth()
{
	return width_;
}



Magick::Image ImageBitstream::getImage()
{
	Image img;

	if(bitstream_)
	{
		img.read(width_, height_, "I", CharPixel, bitstream_);
	}

	return img;
}



ImageBitstream::~ImageBitstream()
{
	if(bitstream_)
		delete[] bitstream_;
}



ImageBitstream::ImageBitstream(Magick::Image img)
{
	bitstream_ = 0;
	setImage(img);
}



unsigned char *ImageBitstream::getBitstream()
{
	return bitstream_;
}



ImageBitstream::ImageBitstream()
{
	bitstream_ = 0;
}


ImageBitstream ImageBitstream::extend(int borderSize)
{
	ImageBitstream extendedImg;

	extendedImg.width_ = width_ + 2 * borderSize;
	extendedImg.height_ = height_ + 2 * borderSize;

	extendedImg.bitstream_ = extendImage(borderSize);

	return extendedImg;
}
