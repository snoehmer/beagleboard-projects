#include "convolutionkernel.h"
#include "generic.h"

/************************************************************************/
/*						GAUSS KERNEL STUFF 								*/
/*																		*/
/*																		*/
/************************************************************************/ 
ConvolutionKernel createConvolutionKernel(float sigma, int overRideMaxSize, int powerOfTwoScaling)
{
	ConvolutionKernel ker = (ConvolutionKernel) vl_malloc(sizeof(ConvolutionKernelRef));

#ifdef ARCH_ARM
	if(overRideMaxSize > 0)
	{
		ker->data = (short*)vl_memalign(8,overRideMaxSize*sizeof(short));
	}
	else
	{
		ker->data = (short*)vl_memalign(8,MAX_KERNEL_WIDTH_DEF*sizeof(short));
	}
#else //_ON_DSP_
#ifdef WIN32
	if(overRideMaxSize > 0)
		ker->data = (short*)vl_malloc(overRideMaxSize*sizeof(short));
	else
		ker->data = (short*)vl_malloc(MAX_KERNEL_WIDTH_DEF*sizeof(short));
#else // WIN32
	if(overRideMaxSize > 0)
		posix_memalign((void**)&(ker->data),8,overRideMaxSize*sizeof(short));
	else
		posix_memalign((void**)&(ker->data),8,MAX_KERNEL_WIDTH_DEF*sizeof(short));
#endif // WIN32
#endif //_ON_DSP_

	ker->width = 0;
	ker->sigma = sigma;
	if(!ker || !ker->data)
	{
		COM_PutStr("Error allocating kernel space!\n");
		exit(-1);
	}
	_computeGaussKernel(sigma,overRideMaxSize,powerOfTwoScaling,ker);
	return ker;
}

void destroyConvolutionKernel(ConvolutionKernel ker)
{
	vl_free(ker->data);
	vl_free(ker);
	ker = 0;
}

static void _computeGaussKernel(
  float sigma,
  int overRideMaxSize,
  int powerOfTwo,
  ConvolutionKernel gauss)
{
	const float factor = 0.005f;   // for truncating tail  ->0.1 works
	int hw;
	int MAX_KERNEL_WIDTH;
	float max_gauss = 1.0f;
	int i;
	int multiFact = (int)pow(2,powerOfTwo);
	char s[256];
	float *tmpKer = 0;
	if(overRideMaxSize > 0)
		MAX_KERNEL_WIDTH = overRideMaxSize;
	else
		MAX_KERNEL_WIDTH = MAX_KERNEL_WIDTH_DEF;

	hw = MAX_KERNEL_WIDTH >> 1;

	if((MAX_KERNEL_WIDTH % 2) == 0)
	{
#ifdef DEBUG
		COM_PutStr("Warning! Padding kernel!\r\n");
#endif // DEBUG
		MAX_KERNEL_WIDTH += 1;
	}
	assert(MAX_KERNEL_WIDTH % 2 == 1);
	assert(sigma >= 0.0);

	tmpKer = (float*)vl_malloc(MAX_KERNEL_WIDTH*sizeof(float));
	if(!tmpKer)
	{
		COM_PutStr("Error allocating tmpKer space!\n");
		exit(-1);
	}

	// Compute kernels, and automatically determine widths 
	// Compute gauss
	for (i = -hw ; i <= hw ; i++)  
	{
		tmpKer[i+hw]      = (float) exp(-i*i / (2*sigma*sigma));
	}

	// Compute widths 
	gauss->width = MAX_KERNEL_WIDTH;
	if(overRideMaxSize == 0)
	{
		for (i = -hw ; fabs(tmpKer[i+hw] / max_gauss) < factor ; i++, gauss->width -= 2);

		if (gauss->width == MAX_KERNEL_WIDTH)
		{
#ifdef DEBUG
			sprintf(s,"Warning: (_computeKernels) MAX_KERNEL_WIDTH %d is too small for "
	   			"a sigma of %f\r\n", MAX_KERNEL_WIDTH, sigma);
			COM_PutStr(s);
#endif // DEBUG
		}
		else if (gauss->width < 5)
			gauss->width = 5;
	}

	// Shift if width less than MAX_KERNEL_WIDTH 
	for (i = 0 ; i < gauss->width ; i++)
		tmpKer[i] = tmpKer[i+(MAX_KERNEL_WIDTH-gauss->width)/2];
	// Normalize gauss 
	{
		float den = 0.0;
		for (i = 0 ; i < gauss->width ; i++)  
			den += tmpKer[i];

		for (i = 0 ; i < gauss->width ; i++)  
			gauss->data[i] = (short)(tmpKer[i] * multiFact / den);
	}
	gauss->sigma = sigma;
	vl_free(tmpKer);
}

void filterImageGaussian(
	short* inputOutputImage,
	int width, int height, 
	ConvolutionKernel gauss)
{
	short * tmpSpace;
	int radius;
	char s[64];

	int kernelLen = gauss->width;
	radius = kernelLen >> 1;

	// alloc temporary space
#ifdef ARCH_ARM
	tmpSpace = (short*) vl_memalign(8,(8 + width*height + kernelLen - 1)*sizeof(short));
#else //_ON_DSP_
#ifdef WIN32
	tmpSpace = (short*) vl_malloc((8 + width*height + kernelLen - 1)*sizeof(short));
#else // WIN32
	posix_memalign((void**)&tmpSpace,8,(8 + width*height + kernelLen - 1)*sizeof(short));
#endif // WIN32
#endif //_ON_DSP_
	if(!tmpSpace)
	{
		COM_PutStr("Error allocating tmpSpace space!\n");
		exit(-1);
	}
	
	memset(tmpSpace,0x00,(8 + width*height + kernelLen - 1)*sizeof(short));

	// TO MAKE EVERYTHING WELL DEFINED!?!?!?!?
	memset(inputOutputImage + width*height,0x00,(kernelLen - 1)*sizeof(short));

	// filter horizontally
	DSP_fir_gen(inputOutputImage,
		gauss->data, 
		tmpSpace + 8, 
		kernelLen, 
		width*height);

	// transpose ver.1 // CAUSES NO MORE BAD SHIFT ERRORS!!!
	DSP_mat_trans(tmpSpace + (8-radius), height, width, inputOutputImage);

	// set output zero
	memset(tmpSpace,0x00,(8 + width*height + kernelLen - 1)*sizeof(short));

	// filter vertically
	DSP_fir_gen(inputOutputImage,
		gauss->data, 
		tmpSpace + 8, 
		kernelLen, 
		width*height);
	
	// transpose image again
	DSP_mat_trans(tmpSpace + (8-radius), width, height, inputOutputImage);
	
	// vl_free temporary space
	vl_free(tmpSpace);
}
