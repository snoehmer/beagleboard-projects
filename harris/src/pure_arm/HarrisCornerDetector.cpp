/*
 * HarrisCornerDetector.cpp
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

#define DEBUG_OUTPUT_PICS

#include "HarrisCornerDetector.h"
#include "../util/HarrisCornerPoint.h"
#include "NonMaxSuppressor.h"
#include "../util/TimeMeasureBase.h"
#include "../util/Logger.h"
#include <cmath>
#include <Magick++.h>
#include "FixedArithmetic.h"

using namespace std;

HarrisCornerDetector::HarrisCornerDetector(float threshold, float k, float dSigma, int dKernelSize, float gSigma, int gKernelSize)
{
  Logger::debug(Logger::HARRIS, "initializing Harris Corner Detector with threshold=%f, k=%f, dSigma=%f, dKSize=%d, gSigma=%f, gKSize=%d", threshold, k, dSigma, dKernelSize, gSigma, gKernelSize);

	devSigma_ = dSigma;
	devKernelSize_ = dKernelSize;
	gaussSigma_ = gSigma;
	gaussKernelSize_ = gKernelSize;
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
    int center;
    int xc2; // = (x - center)^2
    int yc2;
    float sigma2 = devSigma_ * devSigma_;
    float sigma2g = gaussSigma_ * gaussSigma_;

    Fixed sumX(0, HARRIS_Q); // needed for normalization
    Fixed sumY(0, HARRIS_Q);
    Fixed sumGauss(0, HARRIS_Q);

    Logger::debug(Logger::HARRIS, "calculating kernels");

    devKernelX_ = new Fixed[devKernelSize_ * devKernelSize_];
    devKernelY_ = new Fixed[devKernelSize_ * devKernelSize_];
    gaussKernel_ = new Fixed[gaussKernelSize_ * gaussKernelSize_];

    startTimer("_harris_kernels_arm");

    // step 1: calculate derived Gauss function for dev kernels
    center = (devKernelSize_ - 1) / 2;

    for(row = 0; row < devKernelSize_; row++)
    {
        for(col = 0; col < devKernelSize_; col++)
        {
            xc2 = (col - center) * (col - center);
            yc2 = (row - center) * (row - center);

            devKernelX_[row * devKernelSize_ + col] = Fixed(-((float) col - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2)), HARRIS_Q);  // initialization to 30 fractional bits
            devKernelY_[row * devKernelSize_ + col] = Fixed(-((float) row - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2)), HARRIS_Q);

            //devKernelX_[row * devKernelSize_ + col] = -((float) col - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));
            //devKernelY_[row * devKernelSize_ + col] = -((float) row - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));

            sumX += abs(devKernelX_[row * devKernelSize_ + col]);
            sumY += abs(devKernelY_[row * devKernelSize_ + col]);
        }
    }

    // step 2: normalize kernels
    for(row = 0; row < devKernelSize_; row++)
    {
        for(col = 0; col < devKernelSize_; col++)
        {
            devKernelX_[row * devKernelSize_ + col] = devKernelX_[row * devKernelSize_ + col] / sumX;
            devKernelY_[row * devKernelSize_ + col] = devKernelY_[row * devKernelSize_ + col] / sumY;
        }
    }


    // step 3: calculate Gauss function for gauss kernels
    center = (gaussKernelSize_ - 1) / 2;

    for(row = 0; row < gaussKernelSize_; row++)
    {
        for(col = 0; col < gaussKernelSize_; col++)
        {
            xc2 = (col - center) * (col - center);
            yc2 = (row - center) * (row - center);

            gaussKernel_[row * gaussKernelSize_ + col] = Fixed(exp(((float) -(xc2 + yc2)) / (2 * sigma2g)), HARRIS_Q);

            //gaussKernel_[row * gaussKernelSize_ + col] = exp(((float) -(xc2 + yc2)) / (2 * sigma2g));

            sumGauss += gaussKernel_[row * gaussKernelSize_ + col];
        }
    }

    // step 2: normalize kernels
    for(row = 0; row < gaussKernelSize_; row++)
    {
        for(col = 0; col < gaussKernelSize_; col++)
        {
            gaussKernel_[row * gaussKernelSize_ + col] = gaussKernel_[row * gaussKernelSize_ + col] / sumGauss;
        }
    }

    stopTimer("_harris_kernels_arm");
}


void HarrisCornerDetector::inputImage(ImageBitstream img)
{
    input_ = img;
    width_ = img.getWidth();
    height_ = img.getHeight();
}

vector<HarrisCornerPoint> HarrisCornerDetector::detectCorners(ImageBitstream img, Fixed **hcr)
{
	if(!devKernelX_ || !devKernelY_ || !gaussKernel_)
		init();

	inputImage(img);

	return performHarris(hcr);
}

vector<HarrisCornerPoint> HarrisCornerDetector::performHarris(Fixed **hcr)
{
	int imgrow;  // current row and col in the image where the filter is calculated
	int imgcol;
	int krow;  // current row and col in the kernel for the convolution sum
	int kcol;
	int row;  // row and col of image pixel for current kernel position
	int col;

	int offset;
	int extWidth;
	int extHeight;

	Image tempImg;


	// step 1: convolve the image with the derives of Gaussians
	offset = (devKernelSize_ - 1) / 2;
	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	Logger::debug(Logger::HARRIS, "step 1: convolving with derivates of Gaussians");

	ImageBitstream extendedImg = input_.extend(offset);

	Fixed *diffXX = new Fixed[width_ * height_];
	Fixed *diffYY = new Fixed[width_ * height_];
	Fixed *diffXY = new Fixed[width_ * height_];

	Fixed sumX(0, HARRIS_Q);
	Fixed sumY(0, HARRIS_Q);
	Fixed sumXY(0, HARRIS_Q);

	startTimer("_harris_conv_der_arm");

	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < devKernelSize_; krow++)
			{
				for(kcol = 0; kcol < devKernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += scale_uchar(extendedImg.pixel(row, col), HARRIS_Q) * devKernelX_[krow * devKernelSize_ + kcol];
					sumY += scale_uchar(extendedImg.pixel(row, col), HARRIS_Q) * devKernelY_[krow * devKernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY * sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumY;
		}
	}

	stopTimer("_harris_conv_der_arm");

#ifdef DEBUG_OUTPUT_PICS
	float *diffXXout = new float[width_ * height_];
	float *diffYYout = new float[width_ * height_];
  float *diffXYout = new float[width_ * height_];

  for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      diffXXout[row * width_ + col] = diffXX[row * width_ + col].toFloat() * 2;
      diffYYout[row * width_ + col] = diffYY[row * width_ + col].toFloat() * 2;
      diffXYout[row * width_ + col] = diffXY[row * width_ + col].toFloat() * 2;
    }
  }

	tempImg.read(width_, height_, "I", FloatPixel, diffXXout);
	tempImg.write("./output/diffXX.png");
	tempImg.read(width_, height_, "I", FloatPixel, diffYYout);
	tempImg.write("./output/diffYY.png");
	tempImg.read(width_, height_, "I", FloatPixel, diffXYout);
	tempImg.write("./output/diffXY.png");

	delete[] diffXXout;
	delete[] diffYYout;
  delete[] diffXYout;
#endif


	// step 2: apply Gaussian filters to convolved image
	offset = (gaussKernelSize_ - 1) / 2;
	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	Logger::debug(Logger::HARRIS, "step 2: applying Gauss filters to convolved image");

	Fixed *extDiffXX = ImageBitstream::extend(diffXX, width_, height_, offset);
	Fixed *extDiffYY = ImageBitstream::extend(diffYY, width_, height_, offset);
	Fixed *extDiffXY = ImageBitstream::extend(diffXY, width_, height_, offset);

	startTimer("_harris_conv_gauss_arm");

	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;
			sumXY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < gaussKernelSize_; krow++)
			{
				for(kcol = 0; kcol < gaussKernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extDiffXX[row * extWidth + col] * gaussKernel_[krow * gaussKernelSize_ + kcol];
					sumY += extDiffYY[row * extWidth + col] * gaussKernel_[krow * gaussKernelSize_ + kcol];
					sumXY += extDiffXY[row * extWidth + col] * gaussKernel_[krow * gaussKernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumXY;
		}
	}

	stopTimer("_harris_conv_gauss_arm");

	delete[] extDiffXX;
	delete[] extDiffYY;
	delete[] extDiffXY;

#ifdef DEBUG_OUTPUT_PICS
	diffXXout = new float[width_ * height_];
  diffYYout = new float[width_ * height_];
  diffXYout = new float[width_ * height_];

  for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      diffXXout[row * width_ + col] = diffXX[row * width_ + col].toFloat() * 3;
      diffYYout[row * width_ + col] = diffYY[row * width_ + col].toFloat() * 3;
      diffXYout[row * width_ + col] = diffXY[row * width_ + col].toFloat() * 3;
    }
  }

  tempImg.read(width_, height_, "I", FloatPixel, diffXXout);
  tempImg.write("./output/diffXX-gauss.png");
  tempImg.read(width_, height_, "I", FloatPixel, diffYYout);
  tempImg.write("./output/diffYY-gauss.png");
  tempImg.read(width_, height_, "I", FloatPixel, diffXYout);
  tempImg.write("./output/diffXY-gauss.png");

  delete[] diffXXout;
  delete[] diffYYout;
  delete[] diffXYout;
#endif


	// step 3: calculate Harris corner response
	Logger::debug(Logger::HARRIS, "step 3: calculating Harris response");

	Fixed *hcrIntern = new Fixed[width_ * height_];
	Fixed Ixx;
	Fixed Iyy;
	Fixed Ixy;

	startTimer("_harris_hcr_arm");

	for(row = 0; row < height_; row++)
	{
		for(col = 0; col < width_; col++)
		{
			Ixx = diffXX[row * width_ + col];
			Iyy = diffYY[row * width_ + col];
			Ixy = diffXY[row * width_ + col];

			hcrIntern[row * width_ + col] = Ixx * Iyy - Ixy * Ixy - harrisK_ * (Ixx + Iyy) * (Ixx + Iyy);
		}
	}

	stopTimer("_harris_hcr_arm");

	delete[] diffXX;
	delete[] diffYY;
	delete[] diffXY;

#ifdef DEBUG_OUTPUT_PICS
	diffXXout = new float[width_ * height_];

	for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      diffXXout[row * width_ + col] = hcrIntern[row * width_ + col].toFloat() * 200;
    }
  }

	tempImg.read(width_, height_, "I", FloatPixel, diffXXout);
	tempImg.write("./output/hcrIntern.png");

	delete[] diffXXout;
#endif


	// step 4: perform non-maximum-suppression
	Logger::debug(Logger::HARRIS, "step 4: performing Non-Maximum-suppression");

	NonMaxSuppressor nonMax;
	Fixed *hcrNonMax;

	hcrNonMax = nonMax.performNonMax(hcrIntern, width_, height_);

	delete[] hcrIntern;

#ifdef DEBUG_OUTPUT_PICS
	diffXXout = new float[width_ * height_];

  for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      diffXXout[row * width_ + col] = hcrNonMax[row * width_ + col].toFloat() * 50;
    }
  }

  tempImg.read(width_, height_, "I", FloatPixel, diffXXout);
  tempImg.write("./output/hcrNonMax.png");

  delete[] diffXXout;
#endif


	// step 5: normalize the image to a range 0...1 and threshold
	Logger::debug(Logger::HARRIS, "step 5: normalizing and thresholding image");

	vector<HarrisCornerPoint> cornerPoints = normalizeAndThreshold(hcrNonMax, width_ * height_, 1.0f, threshold_);

#ifdef DEBUG_OUTPUT_PICS
	diffXXout = new float[width_ * height_];

  for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      diffXXout[row * width_ + col] = hcrNonMax[row * width_ + col].toFloat();
    }
  }

  tempImg.read(width_, height_, "I", FloatPixel, diffXXout);
  tempImg.write("./output/hcrNonMax-tresh.png");

  delete[] diffXXout;
#endif


	// return HCR if user wants to, delete it otherwise
	if(hcr)
		*hcr = hcrNonMax;
	else
		delete[] hcrNonMax;

	return cornerPoints;
}


void HarrisCornerDetector::normalize(float *data, int n, float newMax)
{
	int i;
	float min, max;

	min = max = data[0];

	for(i = 0; i < n; i++)
	{
		if(data[i] > max) max = data[i];
		if(data[i] < min) min = data[i];
	}

	for(i = 0; i < n; i++)
	{
		data[i] = (data[i] - min) * newMax / (max - min);
	}
}

vector<HarrisCornerPoint> HarrisCornerDetector::threshold(float *data, int n, float threshold)
{
	int i;
	vector<HarrisCornerPoint> cornerPoints;

	for(i = 0; i < n; i++)
	{
		if(data[i] < threshold)
			data[i] = 0;
		else
			cornerPoints.push_back(HarrisCornerPoint(i / width_, i % width_, data[i]));
	}

	return cornerPoints;
}

vector<HarrisCornerPoint> HarrisCornerDetector::normalizeAndThreshold(float *data, int n, float newMax, float threshold)
{
	int i;
	float min, max;
	vector<HarrisCornerPoint> cornerPoints;

	min = max = data[0];

	startTimer("_harris_normtresh_arm");

	for(i = 0; i < n; i++)
	{
		if(data[i] > max) max = data[i];
		if(data[i] < min) min = data[i];
	}

	for(i = 0; i < n; i++)
	{
		data[i] = (data[i] - min) * newMax / (max - min);

		if(data[i] < threshold)
			data[i] = 0;
		else
			cornerPoints.push_back(HarrisCornerPoint(i / width_, i % width_, data[i]));
	}

	stopTimer("_harris_normtresh_arm");

	return cornerPoints;
}

vector<HarrisCornerPoint> HarrisCornerDetector::normalizeAndThreshold(Fixed *data, int n, Fixed newMax, Fixed threshold)
{
  int i;
  Fixed min, max;
  vector<HarrisCornerPoint> cornerPoints;

  min = max = data[0];

  startTimer("_harris_normtresh_arm");

  for(i = 0; i < n; i++)
  {
    if(data[i] > max) max = data[i];
    if(data[i] < min) min = data[i];
  }

  for(i = 0; i < n; i++)
  {
    data[i] = (data[i] - min) * newMax / (max - min);

    if(data[i] < threshold)
      data[i] = 0;
    else
      cornerPoints.push_back(HarrisCornerPoint(i / width_, i % width_, data[i].toFloat()));  //TODO: fixed-point?
  }

  stopTimer("_harris_normtresh_arm");

  return cornerPoints;
}
