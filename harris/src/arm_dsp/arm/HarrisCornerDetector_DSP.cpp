/*
 * HarrisCornerDetector.cpp
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

//#define DEBUG_OUTPUT_PICS

#include "HarrisCornerDetector_DSP.h"
#include "../../util/HarrisCornerPoint.h"
#include "../../pure_arm/NonMaxSuppressor.h"
#include "../../util/TimeMeasureBase.h"
#include "../../util/Logger.h"
#include <cmath>
#include <Magick++.h>
#include "../../pure_arm/FixedArithmetic.h"
#include "dspbridge/Dsp.h"
#include "../common/harris_common.h"

using namespace std;

HarrisCornerDetectorDSP::HarrisCornerDetectorDSP(float threshold, float k, float dSigma, int dKernelSize, float gSigma, int gKernelSize)
  : HarrisCornerDetector(threshold, k, dSigma, dKernelSize, gSigma, gKernelSize)
{
  Logger::debug(Logger::HARRIS, "initializing DSP-supported Harris Corner Detector with threshold=%f, k=%f, dSigma=%f, dKSize=%d, gSigma=%f, gKSize=%d", threshold, k, dSigma, dKernelSize, gSigma, gKernelSize);

	dspInstance_ = &(Dsp::Instance());
	dspNode_ = &(dspInstance_->GetNode());
}

HarrisCornerDetectorDSP::~HarrisCornerDetectorDSP()
{
	if(devKernel_gauss_)
	  dsp_free(devKernel_gauss_);

	if(devKernel_gauss_der_)
    dsp_free(devKernel_gauss_der_);

	if(kernel_gauss_)
	  dsp_free(kernel_gauss_);
}

void HarrisCornerDetectorDSP::init()
{
    int x;
    int center;
    int xc2; // = (x - center)^2
    float sigma2 = devSigma_ * devSigma_;
    float sigma2g = gaussSigma_ * gaussSigma_;

    Fixed sum_gauss(0, HARRIS_Q); // needed for normalization
    Fixed sum_gauss_der(0, HARRIS_Q);
    Fixed sumGauss(0, HARRIS_Q);

    Logger::debug(Logger::HARRIS, "calculating kernels");

    devKernel_gauss_ = (short*) dsp_malloc(devKernelSize_ * sizeof(short));
    devKernel_gauss_der_ = (short*) dsp_malloc(devKernelSize_ * sizeof(short));
    kernel_gauss_ = (short*) dsp_malloc(gaussKernelSize_ * sizeof(short));

    if(!devKernel_gauss_ || !devKernel_gauss_der_ || !kernel_gauss_)
    {
      Logger::error(Logger::HARRIS, "failed to allocate memory for convolution kernels!");
      return;
    }

    devKernelX_ = new Fixed[devKernelSize_];  // we borrow this for devKernel_gauss_ intermediate results
    devKernelY_ = new Fixed[devKernelSize_];  // we borrow this for devKernel_gauss2_ intermediate results
    gaussKernel_ = new Fixed[gaussKernelSize_];

    startTimer("_harris_kernels_arm");

    // step 1: calculate derived Gauss function for dev kernels
    center = (devKernelSize_ - 1) / 2;

    for(x = 0; x < devKernelSize_; x++)
    {
        xc2 = (devKernelSize_ - 1 - x - center) * (devKernelSize_ - 1 - x - center);  // the convolution kernel must be mirrored!

        devKernelX_[x] = Fixed(-((float) devKernelSize_ - 1 - x - center) * exp(((float) -xc2) / (2 * sigma2)), HARRIS_Q);  // this is a derived Gauss function

        devKernelY_[x] = Fixed(exp(((float) -(xc2)) / (2 * sigma2)), HARRIS_Q);  // this is a standard Gauss function

        sum_gauss_der += abs(devKernelX_[x]);
        sum_gauss += abs(devKernelY_[x]);
    }

    // step 2: normalize kernels and convert to Q15 short
    for(x = 0; x < devKernelSize_; x++)
    {
        devKernelX_[x] = devKernelX_[x] / sum_gauss_der;
        devKernelY_[x] = devKernelY_[x] / sum_gauss;

        devKernel_gauss_der_[x] = devKernelX_[x].toQ15();
        devKernel_gauss_[x] = devKernelY_[x].toQ15();
    }

    // step 3: calculate Gauss function for gauss kernels
    center = (gaussKernelSize_ - 1) / 2;

    for(x = 0; x < gaussKernelSize_; x++)
    {
        xc2 = (gaussKernelSize_ - 1 - x - center) * (gaussKernelSize_ - 1 - x - center);  // the convolution kernels must be mirrored!

        gaussKernel_[x] = Fixed(exp(((float) -xc2) / (2 * sigma2g)), HARRIS_Q);

        sumGauss += gaussKernel_[x];
    }

    // step 2: normalize kernel and convert to Q15 short
    for(x = 0; x < gaussKernelSize_; x++)
    {
        gaussKernel_[x] = gaussKernel_[x] / sumGauss;

        kernel_gauss_[x] = gaussKernel_[x].toQ15();
    }

    stopTimer("_harris_kernels_arm");
}


vector<HarrisCornerPoint> HarrisCornerDetectorDSP::performHarris(Fixed **hcr)
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
	vector<HarrisCornerPoint> dummyVector;


	// step 1: convolve the image with the derives of Gaussians
	offset = (devKernelSize_ - 1) / 2;
	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	Logger::debug(Logger::HARRIS, "step 1: convolving with derivates of Gaussians");

	ImageBitstream extendedImg = input_.extend(offset);


	// convert bitstream to DSP format (Q15)
	short *input_dsp = (short*) dsp_malloc((extWidth * extHeight + devKernelSize_ - 1) * sizeof(short));
	if(!input_dsp)
	{
	  Logger::error(Logger::HARRIS, "failed to allocate memory for DSP input array!");
	  return dummyVector;
	}

	// set "border" elements to 0
  memset((short*) (input_dsp + extWidth * extHeight), 0, (devKernelSize_ - 1));

	for(row = 0; row < extHeight; row++)
	{
	  for(col = 0; col < extWidth; col++)
	  {
	    input_dsp[row * extWidth + col] = ((short) extendedImg.pixel(row, col)) << 7;  // shifted by 7 because highest bit is sign bit!
	  }
	}


	short *diffXX = (short*) dsp_malloc(width_ * height_ * sizeof(short));
	short *diffYY = (short*) dsp_malloc(width_ * height_ * sizeof(short));
	short *diffXY = (short*) dsp_malloc(width_ * height_ * sizeof(short));

	if(!diffXX || !diffYY || !diffXY)
	{
	  Logger::error(Logger::HARRIS, "failed to allocate memory for DSP output array!");
    return dummyVector;
	}


	startTimer("_harris_conv_der_arm");


	dsp_harris_conv_params *params = (dsp_harris_conv_params*) dsp_malloc(sizeof(dsp_harris_conv_params));
	if(!params)
	{
	  Logger::error(Logger::HARRIS, "failed to allocate memory for DSP params!");
    return dummyVector;
	}

	params->input_ = (short*) dsp_get_mapped_addr(input_dsp);
	params->width_ = extWidth;
	params->height_ = extHeight;
	params->offset_ = offset;
	params->kernel_gauss_ = (short*) dsp_get_mapped_addr(devKernel_gauss_);
	params->kernel_gauss_der_ = (short*) dsp_get_mapped_addr(devKernel_gauss_der_);
	params->kSize_ = devKernelSize_;
	params->output_diffXX_ = (short*) dsp_get_mapped_addr(diffXX);
	params->output_diffYY_ = (short*) dsp_get_mapped_addr(diffYY);
  params->output_diffXY_ = (short*) dsp_get_mapped_addr(diffXY);

  dsp_dmm_buffer_begin(input_dsp);
  dsp_dmm_buffer_begin(devKernel_gauss_);
  dsp_dmm_buffer_begin(devKernel_gauss_der_);
  dsp_dmm_buffer_begin(diffXX);
  dsp_dmm_buffer_begin(diffYY);
  dsp_dmm_buffer_begin(diffXY);

	int *dsp_result = (int*) dsp_malloc(sizeof(int));

	// start calculation on DSP
	dspNode_->SendMessage(DSP_HARRIS_CALC_CONVOLUTION, (uint32_t) dsp_get_mapped_addr(params),
	    (uint32_t) dsp_get_mapped_addr(dsp_result), 0);

	dsp_msg message = dspNode_->GetMessage();

	dsp_dmm_buffer_end(diffXX);
	dsp_dmm_buffer_end(diffYY);
  dsp_dmm_buffer_end(diffXY);

	stopTimer("_harris_conv_der_arm");

	dsp_free(params);

	if(message.cmd == DSP_HARRIS_CALC_CONVOLUTION && message.arg_2 == DSP_STATUS_FINISHED)
	  Logger::debug(Logger::HARRIS, "DSP successfully calculated convolution with derives");
	else
	{
	  Logger::error(Logger::HARRIS, "error while calculating convolution with derives!");
	  return dummyVector;
	}


	//convert DSP format back to Fixed bitstream
	Fixed *diffXX2 = new Fixed[width_ * height_];
  Fixed *diffYY2 = new Fixed[width_ * height_];
  Fixed *diffXY2 = new Fixed[width_ * height_];

  for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      diffXX2[row * width_ + col] = Fixed(diffXX[row * width_ + col], HARRIS_Q - 15);  // restore old format
      diffYY2[row * width_ + col] = Fixed(diffYY[row * width_ + col], HARRIS_Q - 15);  // restore old format
      diffXY2[row * width_ + col] = Fixed(diffXY[row * width_ + col], HARRIS_Q - 15);  // restore old format
    }
  }


#ifdef DEBUG_OUTPUT_PICS
	tempImg.read(width_, height_, "I", FloatPixel, diffXX);
	tempImg.write("./output/diffXX.png");
	tempImg.read(width_, height_, "I", FloatPixel, diffYY);
	tempImg.write("./output/diffYY.png");
	tempImg.read(width_, height_, "I", FloatPixel, diffXY);
	tempImg.write("./output/diffXY.png");
#endif


	// step 2: apply Gaussian filters to convolved image
	offset = (gaussKernelSize_ - 1) / 2;
	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	Fixed sumX(0, HARRIS_Q);
  Fixed sumY(0, HARRIS_Q);
  Fixed sumXY(0, HARRIS_Q);

	Logger::debug(Logger::HARRIS, "step 2: applying Gauss filters to convolved image");

	Fixed *extDiffXX = ImageBitstream::extend(diffXX2, width_, height_, offset);
	Fixed *extDiffYY = ImageBitstream::extend(diffYY2, width_, height_, offset);
	Fixed *extDiffXY = ImageBitstream::extend(diffXY2, width_, height_, offset);

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

			diffXX2[(imgrow - offset) * width_ + (imgcol - offset)] = sumX;
			diffYY2[(imgrow - offset) * width_ + (imgcol - offset)] = sumY;
			diffXY2[(imgrow - offset) * width_ + (imgcol - offset)] = sumXY;
		}
	}

	stopTimer("_harris_conv_gauss_arm");

	delete[] extDiffXX;
	delete[] extDiffYY;
	delete[] extDiffXY;

#ifdef DEBUG_OUTPUT_PICS
	tempImg.read(width_, height_, "I", FloatPixel, diffXX);
	tempImg.write("./output/diffXX-gauss.png");
	tempImg.read(width_, height_, "I", FloatPixel, diffYY);
	tempImg.write("./output/diffYY-gauss.png");
	tempImg.read(width_, height_, "I", FloatPixel, diffXY);
	tempImg.write("./output/diffXY-gauss.png");
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

	dsp_free(diffXX);
	dsp_free(diffYY);
  dsp_free(diffXY);

	delete[] diffXX2;
	delete[] diffYY2;
	delete[] diffXY2;

#ifdef DEBUG_OUTPUT_PICS
	tempImg.read(width_, height_, "I", FloatPixel, hcrIntern);
	tempImg.write("./output/hcrIntern.png");
#endif


	// step 4: perform non-maximum-suppression
	Logger::debug(Logger::HARRIS, "step 4: performing Non-Maximum-suppression");

	NonMaxSuppressor nonMax;
	Fixed *hcrNonMax;

	hcrNonMax = nonMax.performNonMax(hcrIntern, width_, height_);

	delete[] hcrIntern;

#ifdef DEBUG_OUTPUT_PICS
	tempImg.read(width_, height_, "I", FloatPixel, hcrNonMax);
	tempImg.write("./output/hcrNonMax.png");
#endif


	// step 5: normalize the image to a range 0...1 and threshold
	Logger::debug(Logger::HARRIS, "step 5: normalizing and thresholding image");

	vector<HarrisCornerPoint> cornerPoints = normalizeAndThreshold(hcrNonMax, width_ * height_, 1.0f, threshold_);

#ifdef DEBUG_OUTPUT_PICS
	tempImg.read(width_, height_, "I", FloatPixel, hcrNonMax);
	tempImg.write("./output/hcrNonMax-tresh.png");
#endif


	// return HCR if user wants to, delete it otherwise
	if(hcr)
		*hcr = hcrNonMax;
	else
		delete[] hcrNonMax;

	return cornerPoints;
}


void HarrisCornerDetectorDSP::normalize(float *data, int n, float newMax)
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

vector<HarrisCornerPoint> HarrisCornerDetectorDSP::treshold(float *data, int n, float threshold)
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

vector<HarrisCornerPoint> HarrisCornerDetectorDSP::normalizeAndThreshold(float *data, int n, float newMax, float threshold)
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

vector<HarrisCornerPoint> HarrisCornerDetectorDSP::normalizeAndThreshold(Fixed *data, int n, Fixed newMax, Fixed threshold)
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
