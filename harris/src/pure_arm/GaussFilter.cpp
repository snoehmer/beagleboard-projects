/*
 * GaussFilter.cpp
 *
 *  Created on: 16.08.2011
 *      Author: sn
 */

#include "GaussFilter.h"
#include <cmath>

GaussFilter::GaussFilter(int kernelSize, float sigma)
{
	if(kernelSize % 2 != 0)
		kernelSize_ = kernelSize;
	else
		kernelSize_ = kernelSize - 1;

	sigma_ = sigma;

	kernel_ = 0;
}


GaussFilter::GaussFilter(ImageBitstream original, int kernelSize, float sigma)
{
	if(kernelSize % 2 != 0)
		kernelSize_ = kernelSize;
	else
		kernelSize_ = kernelSize - 1;

	sigma_ = sigma;

	inputImage(original);

	kernel_ = 0;
}


GaussFilter::~GaussFilter()
{
	if(kernel_)
		delete[] kernel_;
}


void GaussFilter::generateKernel()
{
	int row;
	int col;
	int center = (kernelSize_ - 1) / 2;
	int xc2; // = (x - center)^2
	int yc2;
	float sigma2 = sigma_ * sigma_;

	float sum = 0; // needed for normalization

	kernel_ = new float[kernelSize_ * kernelSize_];

	// step 1: calculate Gauss function for kernel
	for(row = 0; row < kernelSize_; row++)
	{
		for(col = 0; col < kernelSize_; col++)
		{
			xc2 = (col - center) * (col - center);
			yc2 = (row - center) * (row - center);

			kernel_[row * kernelSize_ + col] = 1.0f / (2 * M_PI * sigma2)
					* exp(((float) -(xc2 + yc2)) / (2 * sigma2));

			sum += kernel_[row * kernelSize_ + col];
		}
	}

	// step 2: normalize kernel
	for(row = 0; row < kernelSize_; row++)
	{
		for(col = 0; col < kernelSize_; col++)
		{
			kernel_[row * kernelSize_ + col] = kernel_[row * kernelSize_ + col] / sum;
		}
	}
}


void GaussFilter::inputImage(ImageBitstream input)
{
	input_ = input;
}


ImageBitstream GaussFilter::calculate()
{
	ImageBitstream output;

	if(!kernel_)
		generateKernel();

	if(!input_.isLoaded())
		return output;

	// perform convolution
	return input_.convolve(kernel_, kernelSize_);
}


ImageBitstream GaussFilter::filterImage(ImageBitstream input)
{
	if(!kernel_) // kernel not initialized!
		generateKernel();

	inputImage(input);
	return calculate();
}
