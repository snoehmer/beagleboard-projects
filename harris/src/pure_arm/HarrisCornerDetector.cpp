/*
 * HarrisCornerDetector.cpp
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

#include "HarrisCornerDetector.h"
#include "GaussFilter.h"
#include "../util/HarrisCornerPoint.h"
#include <cmath>

using namespace std;

HarrisCornerDetector::HarrisCornerDetector(float threshold, float dSigma, int kernelSize, float gSigma, float k)
{
	devSigma_ = dSigma;
	kernelSize_ = kernelSize;
	gaussSigma_ = gSigma;
	harrisK_ = k;
	threshold_ = threshold;

	devKernelX_ = 0;
	devKernelY_ = 0;
	gaussKernel_ = 0;
}

HarrisCornerDetector::~HarrisCornerDetector()
{
	if(devKernelX_)
		delete[] devKernelX_;

	if(devKernelY_)
		delete[] devKernelY_;

	if(gaussKernel_)
		delete[] gaussKernel_;
}

void HarrisCornerDetector::init()
{
    int row;
    int col;
    int center = (kernelSize_ - 1) / 2;
    int xc2; // = (x - center)^2
    int yc2;
    float sigma2 = devSigma_ * devSigma_;
    float sigma2g = gaussSigma_ * gaussSigma_;

    float sumX = 0; // needed for normalization
    float sumY = 0;
    float sumGauss = 0;

    devKernelX_ = new float[kernelSize_ * kernelSize_];
    devKernelY_ = new float[kernelSize_ * kernelSize_];
    gaussKernel_ = new float[kernelSize_ * kernelSize_];


    // step 1: calculate derived Gauss function for kernels and the Gauss filter kernel
    for(row = 0; row < kernelSize_; row++)
    {
        for(col = 0; col < kernelSize_; col++)
        {
            xc2 = (col - center) * (col - center);
            yc2 = (row - center) * (row - center);

            devKernelX_[row * kernelSize_ + col] = -((float) col - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));

            devKernelY_[row * kernelSize_ + col] = -((float) row - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));

            gaussKernel_[row * kernelSize_ + col] = exp(((float) -(xc2 + yc2)) / (2 * sigma2g));

            sumX += abs(devKernelX_[row * kernelSize_ + col]);
            sumY += abs(devKernelY_[row * kernelSize_ + col]);
            sumGauss += gaussKernel_[row * kernelSize_ + col];
        }
    }

    // step 2: normalize kernels
    for(row = 0; row < kernelSize_; row++)
    {
        for(col = 0; col < kernelSize_; col++)
        {
            devKernelX_[row * kernelSize_ + col] = devKernelX_[row * kernelSize_ + col] / sumX;
            devKernelY_[row * kernelSize_ + col] = devKernelY_[row * kernelSize_ + col] / sumY;
            gaussKernel_[row * kernelSize_ + col] = gaussKernel_[row * kernelSize_ + col] / sumGauss;
        }
    }
}


void HarrisCornerDetector::inputImage(ImageBitstream img)
{
    input_ = img;
    width_ = img.getWidth();
    height_ = img.getHeight();
}

ImageBitstream HarrisCornerDetector::detectCorners(ImageBitstream img, vector<HarrisCornerPoint> &cornerList, float **hcr)
{
	if(!devKernelX_ || !devKernelY_ || !gaussKernel_)
		init();

	inputImage(img);

	return performHarris(hcr, cornerList);
}

ImageBitstream HarrisCornerDetector::performHarris(float **hcr, vector<HarrisCornerPoint> &cornerPoints)
{
	int imgrow;  // current row and col in the image where the filter is calculated
	int imgcol;
	int krow;  // current row and col in the kernel for the convolution sum
	int kcol;
	int row;  // row and col of image pixel for current kernel position
	int col;

	int offset = (kernelSize_ - 1) / 2;

	int extWidth = width_ + 2 * offset;
	int extHeight = height_ + 2 * offset;

	float *diffXX = new float[width_ * height_];
	float *diffYY = new float[width_ * height_];
	float *diffXY = new float[width_ * height_];

	float sumX;
	float sumY;
	float sumXY;


	// init
	ImageBitstream extendedImg = input_.extend((kernelSize_ - 1)/ 2);
	cornerPoints.clear();

	// step 1: convolve the image with the derives of Gaussians
	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < kernelSize_; krow++)
			{
				for(kcol = 0; kcol < kernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extendedImg.pixel(row, col) * devKernelX_[krow * kernelSize_ + kcol];
					sumY += extendedImg.pixel(row, col) * devKernelY_[krow * kernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY * sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumY;
		}
	}


	// step 2a: extend the derived images for Gaussian filtering
	float *extDiffXX = extendImage(diffXX, offset);
	float *extDiffYY = extendImage(diffYY, offset);
	float *extDiffXY = extendImage(diffXY, offset);


	// step 2b: apply Gaussian filters to convolved image
	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;
			sumXY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < kernelSize_; krow++)
			{
				for(kcol = 0; kcol < kernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extDiffXX[row * extWidth + col] * gaussKernel_[krow * kernelSize_ + kcol];
					sumY += extDiffYY[row * extWidth + col] * gaussKernel_[krow * kernelSize_ + kcol];
					sumXY += extDiffXY[row * extWidth + col] * gaussKernel_[krow * kernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumXY;
		}
	}

	delete[] extDiffXX;
	delete[] extDiffYY;
	delete[] extDiffXY;


	// step 3: calculate Harris corner response and perform thresholding
	float *hcrIntern = new float[width_ * height_];
	float Ixx;
	float Iyy;
	float Ixy;
	float hcrScore;

	for(row = 0; row < height_; row++)
	{
		for(col = 0; col < width_; col++)
		{
			Ixx = diffXX[row * width_ + col];
			Iyy = diffYY[row * width_ + col];
			Ixy = diffXY[row * width_ + col];

			hcrScore = Ixx * Iyy - Ixy * Ixy - harrisK_ * (Ixx + Iyy) * (Ixx + Iyy);

			if(hcrScore > threshold_)
			{
				hcrIntern[row * width_ + col] = hcrScore;

				cornerPoints.push_back(HarrisCornerPoint(row, col, hcrScore));
			}
			else
			{
				hcrIntern[row * width_ + col] = 0;
			}
		}
	}


	// step 4: do non-maximum suppression
	//TODO

	// generate grayscale corner strength image
	ImageBitstream cornerStrength(hcrIntern, width_, height_);

	// return HCR if user wants to, delete it otherwise
	if(*hcr)
		*hcr = hcrIntern;
	else
		delete[] hcrIntern;

	return cornerStrength;
}

float* HarrisCornerDetector::extendImage(float *input, int borderSize)
{
	int row;
	int col;
	float *extendedImg;

	int offset = borderSize;

	int extWidth = width_ + 2 * offset;
	int extHeight = height_ + 2 * offset;

	extendedImg = new float[extWidth * extHeight];

	// step 0: copy image
	for(row = 0; row < height_; row++)
			for(col = 0; col < width_; col++)
					extendedImg[(row + offset) * extWidth + (col + offset)] = input[row * width_ + col];

	// step 1a: copy upper border
	for(row = 0; row < offset; row++)
			for(col = 0; col < width_; col++)
					extendedImg[row * extWidth + (col + offset)] = input[0 * width_ + col];

	// step 1b: copy lower border
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = 0; col < width_; col++)
					extendedImg[row * extWidth + (col + offset)] = input[(height_ - 1) * width_ + col];

	// step 1c: copy left border
	for(col = 0; col < offset; col++)
			for(row = 0; row < height_; row++)
					extendedImg[(row + offset) * extWidth + col] = input[row * width_ + 0];

	// step 1d: copy right border
	for(col = offset + width_; col < width_ + 2*offset; col++)
			for(row = 0; row < height_; row++)
					extendedImg[(row + offset) * extWidth + col] = input[row * width_ + (width_ - 1)];

	// step 2a: copy upper left corner
	for(row = 0; row < offset; row++)
			for(col = 0; col < offset; col++)
					extendedImg[row * extWidth + col] = input[0 * width_ + 0];

	// step 2b: copy upper right corner
	for(row = 0; row < offset; row++)
			for(col = offset + width_; col < width_ + 2*offset; col++)
					extendedImg[row * extWidth + col] = input[0 * width_ + (width_ - 1)];

	// step 2c: copy lower left corner
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = 0; col < offset; col++)
					extendedImg[row * extWidth + col] = input[(height_ - 1) * width_ + 0];

	// step 2d: copy lower right corner
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = offset + width_; col < width_ + 2*offset; col++)
					extendedImg[row * extWidth + col] = input[(height_ - 1) * width_ + (width_ - 1)];

	return extendedImg;
}
