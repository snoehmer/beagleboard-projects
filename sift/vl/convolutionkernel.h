#ifndef _CONVOLUTIONKERNEL_INCLUDED_
#define _CONVOLUTIONKERNEL_INCLUDED_

#include "globals.h"

/*****************************/
// CONVOLUTION KERNEL STUFF
/*****************************/
typedef struct  {
  int width;
  float sigma;
  short * data;
}  ConvolutionKernelRef, *ConvolutionKernel;

static void _computeGaussKernel(
  float sigma,
  int overRideMaxSize,
  int powerOfTwo,
  ConvolutionKernel gauss);

ConvolutionKernel createConvolutionKernel(float sigma, int overRideMaxSize, int powerOfTwoScaling);
void destroyConvolutionKernel(ConvolutionKernel ker);

void filterImageGaussian(
	short* inputOutputImage,
	int width, int height, 
	ConvolutionKernel gauss);


#endif
