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

	if(nonMaxKernel_)
	  dsp_free(nonMaxKernel_);

	if(nonMaxKernel1_)
    dsp_free(nonMaxKernel1_);
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

    //step 3: generate non-maximum suppression kernel (simple 1st deviation kernel)
    nonMaxKernel_ = (short*) dsp_malloc(nonMaxKernelSize_ * sizeof(short));
    //nonMaxKernel_[0] = Fixed(-0.5f).toQ15(); nonMaxKernel_[1] = Fixed(0).toQ15(); nonMaxKernel_[2] = Fixed(0.5f).toQ15();  // TODO: reverse?
    nonMaxKernel_[0] = Fixed(-1).toQ15(); nonMaxKernel_[1] = Fixed(0).toQ15(); nonMaxKernel_[2] = Fixed(1).toQ15();  // TODO: reverse?
    nonMaxKernel1_ = (short*) dsp_malloc(nonMaxKernelSize_ * sizeof(short));
    //nonMaxKernel1_[0] = Fixed(0.33333f).toQ15(); nonMaxKernel1_[1] = Fixed(0.33333f).toQ15(); nonMaxKernel1_[2] = Fixed(0.33333f).toQ15();
    nonMaxKernel1_[0] = Fixed(1).toQ15(); nonMaxKernel1_[1] = Fixed(1).toQ15(); nonMaxKernel1_[2] = Fixed(1).toQ15();


    stopTimer("_harris_kernels_arm");
}


vector<HarrisCornerPoint> HarrisCornerDetectorDSP::performHarris(Fixed **ret_hcr)
{
	int row;  // row and col of image pixel for current kernel position
	int col;

	int offset;
	int extWidth;
	int extHeight;

	Image tempImg;
	vector<HarrisCornerPoint> dummyVector;


	// step 1: convolve the image with the derives of Gaussians
	if(devKernelSize_ > gaussKernelSize_)  // set right offset (resizing to fit largest kernel)
	  offset = (devKernelSize_ - 1) / 2;
	else
	  offset = (gaussKernelSize_ - 1) / 2;

	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	Logger::debug(Logger::HARRIS, "step 1: convolving with derivates of Gaussians");

	ImageBitstream extendedImg = input_.extend(offset);


	// convert bitstream to DSP format (Q15)
	short *input_dsp = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
	if(!input_dsp)
	{
	  Logger::error(Logger::HARRIS, "failed to allocate memory for DSP input array!");
	  return dummyVector;
	}

	// set DSP convolution "border" elements to 0
  memset((short*) (input_dsp + extWidth * extHeight), 0, (2 * offset - 1));

	for(row = 0; row < extHeight; row++)
	{
	  for(col = 0; col < extWidth; col++)
	  {
	    input_dsp[row * extWidth + col] = ((short) extendedImg.pixel(row, col)) << 7;  // shifted by 7 because highest bit is sign bit!
	  }
	}


	// allocate memory for DSP variables
	short *convX = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
	if(!convX)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP convX array!");
    return dummyVector;
  }

	short *convY = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
	if(!convY)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP convY array!");
    return dummyVector;
  }


	short *diffXX = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
	if(!diffXX)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP diffXX array!");
    return dummyVector;
  }

	short *diffYY = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
	if(!diffYY)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP diffYY array!");
    return dummyVector;
  }

	short *diffXY = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
	if(!diffXY)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP diffXY array!");
    return dummyVector;
  }


	short *out_diffXX = (short*) dsp_malloc(width_ * height_ * sizeof(short));
	if(!out_diffXX)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP out_diffXX array!");
    return dummyVector;
  }

  short *out_diffYY = (short*) dsp_malloc(width_ * height_ * sizeof(short));
  if(!out_diffYY)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP out_diffYY array!");
    return dummyVector;
  }

  short *out_diffXY = (short*) dsp_malloc(width_ * height_ * sizeof(short));
  if(!out_diffXY)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP out_diffXY array!");
    return dummyVector;
  }


  short *nonmaxX = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!nonmaxX)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP nonmaxX array!");
    return dummyVector;
  }

  short *nonmaxY = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!nonmaxY)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP nonmaxY array!");
    return dummyVector;
  }

  short *nonmaxM2 = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!nonmaxM2)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP nonmaxM2 array!");
    return dummyVector;
  }


  short *hcr = (short*) dsp_malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!hcr)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP hcr array!");
    return dummyVector;
  }

  short *hcr_out = (short*) dsp_malloc((width_ * height_ + 2 * offset - 1) * sizeof(short));
  if(!hcr_out)
  {
    Logger::error(Logger::HARRIS, "failed to allocate memory for DSP hcr_out array!");
    return dummyVector;
  }


	startTimer("_harris_conv_der_arm");


	// set parameters for whole Harris corner detection
	dsp_harris_params *params = (dsp_harris_params*) dsp_malloc(sizeof(dsp_harris_params));
	if(!params)
	{
	  Logger::error(Logger::HARRIS, "failed to allocate memory for DSP params!");
    return dummyVector;
	}

	params->input_ = (short*) dsp_get_mapped_addr(input_dsp);
	params->width_ = extWidth;
	params->height_ = extHeight;
	params->offset_ = offset;
	params->devKernel_gauss_ = (short*) dsp_get_mapped_addr(devKernel_gauss_);
	params->devKernel_gauss_der_ = (short*) dsp_get_mapped_addr(devKernel_gauss_der_);
	params->devKernelSize_ = devKernelSize_;
	params->gaussKernel_ = (short*) dsp_get_mapped_addr(kernel_gauss_);
	params->gaussKernelSize_ = gaussKernelSize_;
	params->harris_k_ = Fixed(harrisK_).toQ15();
	params->nonMaxKernel_ = (short*) dsp_get_mapped_addr(nonMaxKernel_);
	params->nonMaxKernel1_ = (short*) dsp_get_mapped_addr(nonMaxKernel1_);
	params->nonMaxKernelSize_ = nonMaxKernelSize_;
	params->convX_ = (short*) dsp_get_mapped_addr(convX);
  params->convY_ = (short*) dsp_get_mapped_addr(convY);
	params->diffXX_ = (short*) dsp_get_mapped_addr(diffXX);
	params->diffYY_ = (short*) dsp_get_mapped_addr(diffYY);
  params->diffXY_ = (short*) dsp_get_mapped_addr(diffXY);
  params->hcr_ = (short*) dsp_get_mapped_addr(hcr);
  params->nonmaxX_ = (short*) dsp_get_mapped_addr(nonmaxX);
  params->nonmaxY_ = (short*) dsp_get_mapped_addr(nonmaxY);
  params->nonmaxM2_ = (short*) dsp_get_mapped_addr(nonmaxM2);
  params->output_diffXX_ = (short*) dsp_get_mapped_addr(out_diffXX);
  params->output_diffYY_ = (short*) dsp_get_mapped_addr(out_diffYY);
  params->output_diffXY_ = (short*) dsp_get_mapped_addr(out_diffXY);
  params->hcr_out_ = (short*) dsp_get_mapped_addr(hcr_out);

  dsp_dmm_buffer_begin(input_dsp);
  dsp_dmm_buffer_begin(devKernel_gauss_);
  dsp_dmm_buffer_begin(devKernel_gauss_der_);
  dsp_dmm_buffer_begin(kernel_gauss_);
  dsp_dmm_buffer_begin(nonMaxKernel_);
  dsp_dmm_buffer_begin(nonMaxKernel1_);
  dsp_dmm_buffer_begin(convX);
  dsp_dmm_buffer_begin(convY);
  dsp_dmm_buffer_begin(diffXX);
  dsp_dmm_buffer_begin(diffYY);
  dsp_dmm_buffer_begin(diffXY);
  dsp_dmm_buffer_begin(hcr);
  dsp_dmm_buffer_begin(nonmaxX);
  dsp_dmm_buffer_begin(nonmaxY);
  dsp_dmm_buffer_begin(nonmaxM2);
  dsp_dmm_buffer_begin(out_diffXX);
  dsp_dmm_buffer_begin(out_diffYY);
  dsp_dmm_buffer_begin(out_diffXY);
  dsp_dmm_buffer_begin(hcr_out);


  // start calculation on DSP
	dspNode_->SendMessage(DSP_PERFORM_HARRIS, (uint32_t) dsp_get_mapped_addr(params), 0, 0);

	dsp_msg message = dspNode_->GetMessage();


	dsp_dmm_buffer_end(convX);
  dsp_dmm_buffer_end(convY);
	dsp_dmm_buffer_end(diffXX);
	dsp_dmm_buffer_end(diffYY);
  dsp_dmm_buffer_end(diffXY);
  dsp_dmm_buffer_end(hcr);
  dsp_dmm_buffer_end(nonmaxX);
  dsp_dmm_buffer_end(nonmaxY);
  dsp_dmm_buffer_end(nonmaxM2);
  dsp_dmm_buffer_end(out_diffXX);
  dsp_dmm_buffer_end(out_diffYY);
  dsp_dmm_buffer_end(out_diffXY);
  dsp_dmm_buffer_end(hcr_out);

	stopTimer("_harris_conv_der_arm");

	dsp_free(params);

	if(message.cmd == DSP_PERFORM_HARRIS && message.arg_2 == DSP_STATUS_FINISHED)
	  Logger::debug(Logger::HARRIS, "DSP successfully calculated Harris corner response");
	else
	{
	  Logger::error(Logger::HARRIS, "error while calculating Harris corner response on DSP!");
	  return dummyVector;
	}


//#ifdef DEBUG_OUTPUT_PICS
  float *outXX = new float[width_ * height_];
  float *outYY = new float[width_ * height_];
  float *outXY = new float[width_ * height_];

  //printf("diffXX, diffYY, diffXY: ");
  for(row = 0; row < height_; row++)
  {
    for(col = 0; col < width_; col++)
    {
      outXX[row * width_ + col] = Q15toFixed(out_diffXX[row * width_ + col]).toFloat();  // restore old format
      outYY[row * width_ + col] = Q15toFixed(out_diffYY[row * width_ + col]).toFloat();  // restore old format
      outXY[row * width_ + col] = Q15toFixed(out_diffXY[row * width_ + col]).toFloat();  // restore old format

      //printf("%d=%f %d=%f %d=%f, ", out_diffXX[row * width_ + col], outXX[row * width_ + col], out_diffYY[row * width_ + col], outYY[row * width_ + col], out_diffXY[row * width_ + col], outXY[row * width_ + col]);
    }
    //printf("; ");
  }
  //printf("done\n");


	tempImg.read(width_, height_, "I", FloatPixel, outXX);
	tempImg.write("./output/diffXX.png");
	tempImg.read(width_, height_, "I", FloatPixel, outYY);
	tempImg.write("./output/diffYY.png");
	tempImg.read(width_, height_, "I", FloatPixel, outXY);
	tempImg.write("./output/diffXY.png");

	delete[] outXX;
	delete[] outYY;
	delete[] outXY;
//#endif


	Fixed *hcrIntern = new Fixed[width_ * height_];


//#ifdef DEBUG_OUTPUT_PICS
	float *outhcr = new float[width_ * height_];
//#endif

	//printf("hcr: ");
	//printf("hcr, nonmaxX, nonmaxY, nonmaxM2: ");
	for(row = 0; row < height_; row++)
	{
		for(col = 0; col < width_; col++)
		{
		  hcrIntern[row * width_ + col] = Q15toFixed(hcr_out[row * width_ + col]);

//#ifdef DEBUG_OUTPUT_PICS
			outhcr[row * width_ + col] = hcrIntern[row * width_ + col].toFloat();
//#endif

			//printf("%d=%f ", hcr_out[row * width_ + col], outhcr[row * width_ + col]);
			//printf("%d %d %d %d, ", hcr_out[row * width_ + col], nonmaxX[(row + offset) * extWidth + col + offset], nonmaxY[(row + offset) * extWidth + col + offset], nonmaxM2[(row + offset) * extWidth + col + offset]);
		}
		//printf("; ");
	}
	//printf("done\n");

//#ifdef DEBUG_OUTPUT_PICS
  tempImg.read(width_, height_, "I", FloatPixel, outhcr);
  tempImg.write("./output/hcrIntern.png");

  delete[] outhcr;
//#endif


	stopTimer("_harris_hcr_arm");

	dsp_free(convX);  //TODO: use for rotation invariance
	dsp_free(convY);
	dsp_free(diffXX);
	dsp_free(diffYY);
  dsp_free(diffXY);
  dsp_free(out_diffXX);
  dsp_free(out_diffYY);
  dsp_free(out_diffXY);
  dsp_free(hcr);
  dsp_free(nonmaxX);
  dsp_free(nonmaxY);
  dsp_free(nonmaxM2);
  dsp_free(hcr_out);


	// step 5: normalize the image to a range 0...1 and threshold
	Logger::debug(Logger::HARRIS, "step 5: normalizing and thresholding image");

	vector<HarrisCornerPoint> cornerPoints = normalizeAndThreshold(hcrIntern, width_ * height_, 1.0f, threshold_);

#ifdef DEBUG_OUTPUT_PICS
	tempImg.read(width_, height_, "I", FloatPixel, hcrIntern);
	tempImg.write("./output/hcrNonMax-tresh.png");
#endif


	// return HCR if user wants to, delete it otherwise
	if(ret_hcr)
		*ret_hcr = hcrIntern;
	else
		delete[] hcrIntern;

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
