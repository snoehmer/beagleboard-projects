/*
 * DSP implementations of functions needed by Harris Corner Detector
 * and Normalized Cross Correlator to speed up calculations
 */

#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../../opt/dsplib/inc/dsplib.h"
#include <stddef.h>
#include <string.h>
#include "../common/harris_common.h"
#include "../arm/dspbridge/node.h"


void DSP_mat_trans_slow(const short* restrict x, const short rows, const short columns, short* restrict r);
void DSP_fir_gen_slow(const short* restrict x, const short* restrict h, short* restrict r, const int nh, const int nr);


//int dsp_perform_harris(dsp_harris_params *params);

int dsp_harris_convolve_harris(const short* restrict input, unsigned int extHeight,
    unsigned int extWidth, unsigned int offset, const short* restrict devKernel_gauss,
    const short* restrict devKernel_gauss_der, unsigned int devKernelSize, const short* restrict gaussKernel,
    unsigned int gaussKernelSize, short* restrict output_diffXX,
    short* restrict output_diffYY, short* restrict output_diffXY);

int dsp_convolve2d(const short* restrict input, unsigned int height, unsigned int width, unsigned int offset,
    const short* restrict kernel_hor, const short* restrict kernel_ver, unsigned int kernelSize,
    short* restrict output);


unsigned int dsp_harris_create(void)
{

	return 0x8000;
}

unsigned int dsp_harris_delete(void)
{
	return 0x8000;
}

unsigned int dsp_harris_execute(void *env)
{
	dsp_msg_t msg;
	unsigned char done = 0;
	

	while (!done)
	{
		NODE_getMsg(env, &msg, (unsigned) -1);

		switch(msg.cmd)
		{
      case DSP_PERFORM_HARRIS:
      {
        dsp_harris_params *params = (dsp_harris_params*) msg.arg_1;

        short *input = params->input_;

        BCACHE_inv((void*) params, sizeof(dsp_harris_params), 1);

        unsigned int extHeight = params->height_;
        unsigned int extWidth = params->width_;
        unsigned int offset = params->offset_;
        short *devKernel_gauss = params->devKernel_gauss_;
        short *devKernel_gauss_der = params->devKernel_gauss_der_;
        unsigned int devKernelSize = params->devKernelSize_;
        short *gaussKernel = params->gaussKernel_;
        unsigned int gaussKernelSize = params->gaussKernelSize_;
        short *output_diffXX = params->output_diffXX_;
        short *output_diffYY = params->output_diffYY_;
        short *output_diffXY = params->output_diffXY_;

        unsigned int extSize = extHeight * extWidth;

        unsigned int height = extHeight - 2 * offset;
        unsigned int width = extWidth - 2 * offset;
        unsigned int size = height * width;

        BCACHE_inv((void*) input, extSize * sizeof(short), 1);
        BCACHE_inv((void*) devKernel_gauss, devKernelSize * sizeof(short), 1);
        BCACHE_inv((void*) devKernel_gauss_der, devKernelSize * sizeof(short), 1);
        BCACHE_inv((void*) gaussKernel, gaussKernelSize * sizeof(short), 1);
        BCACHE_inv((void*) output_diffXX, size * sizeof(short), 1);
        BCACHE_inv((void*) output_diffYY, size * sizeof(short), 1);
        BCACHE_inv((void*) output_diffXY, size * sizeof(short), 1);

        int result = dsp_harris_convolve_harris(input, extHeight, extWidth, offset,
            devKernel_gauss, devKernel_gauss_der, devKernelSize,
            gaussKernel, gaussKernelSize,
            output_diffXX, output_diffYY, output_diffXY);

        BCACHE_wbInv((void*) output_diffXX, size * sizeof(short), 1);
        BCACHE_wbInv((void*) output_diffYY, size * sizeof(short), 1);
        BCACHE_wbInv((void*) output_diffXY, size * sizeof(short), 1);

        msg.arg_2 = result;

        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

      case 0x80000000:
      {
        done = 1;
        break;
      }
		}
	}

	return 0x8000;
}


// slow functions for images that do not fulfill requirements (taken from TIs DSPLIB)

void DSP_mat_trans_slow(const short* restrict x, const short rows, const short columns, short* restrict r)
{
  short i,j;
    for(i=0; i<columns; i++)
      for(j=0; j<rows; j++)
        *(r+i*rows+j)=*(x+i+columns*j);
}

void DSP_fir_gen_slow
(
  const short* restrict x, // Input ('nr + nh - 1' samples)
  const short* restrict h, // Filter coefficients (nh taps)
  short* restrict r, // Output array ('nr' samples)
  const int nh, // Length of filter (nh >= 5)
  const int nr // Length of output (nr >= 1)
)
{
  int i, j, sum;
  for (j = 0; j < nr; j++)
  {
    sum = 0;
    for (i = 0; i < nh; i++)
    {
      sum += ((int) x[i + j]) * ((int) h[i]);
    }
    r[j] = sum >> 15;
  }
}


int dsp_harris_convolve_harris(const short* restrict input, unsigned int extHeight,
    unsigned int extWidth, unsigned int offset, const short* restrict devKernel_gauss,
    const short* restrict devKernel_gauss_der, unsigned int devKernelSize, const short* restrict gaussKernel,
    unsigned int gaussKernelSize, short* restrict output_diffXX,
    short* restrict output_diffYY, short* restrict output_diffXY)
{
  unsigned int height = extHeight - 2 * offset;
  unsigned int width = extWidth - 2 * offset;


  // ============== perform edge detection by convolution with derived kernels ==============

  // results of the 2d convolution by 2 1d convolutions
  short *convX = (short*) malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!convX)
    return DSP_STATUS_FAILED;

  short *convY = (short*) malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!convY)
  {
    free(convX);
    return DSP_STATUS_FAILED;
  }


  // calculate x-derivate of image
  if(dsp_convolve2d(input, extHeight, extWidth, offset, devKernel_gauss_der, devKernel_gauss, devKernelSize, convX) != DSP_STATUS_FINISHED)
  {
    free(convX);
    return DSP_STATUS_FAILED;
  }


  //calculate y-derivate of image
  if(dsp_convolve2d(input, extHeight, extWidth, offset, devKernel_gauss, devKernel_gauss_der, devKernelSize, convY) != DSP_STATUS_FINISHED)
  {
    free(convX);
    free(convY);
    return DSP_STATUS_FAILED;
  }


  // now calculate diffXX (squared x), diffYY (squared y) and diffXY (x * y)
  short *diffXX = (short*) malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!diffXX)
  {
    free(convX);
    free(convY);
    return DSP_STATUS_FAILED;
  }

  short *diffYY = (short*) malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!diffYY)
  {
    free(convX);
    free(convY);
    free(diffXX);
    return DSP_STATUS_FAILED;
  }

  short *diffXY = (short*) malloc((extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  if(!diffXY)
  {
    free(convX);
    free(convY);
    free(diffXX);
    free(diffYY);
    return DSP_STATUS_FAILED;
  }


  memset(diffXX, 0, (extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  memset(diffYY, 0, (extWidth * extHeight + 2 * offset - 1) * sizeof(short));
  memset(diffXY, 0, (extWidth * extHeight + 2 * offset - 1) * sizeof(short));

  int row;
  int col;

  for(row = offset; row < offset + height; row++)
  {
    for(col = offset; col < offset + width; col++)
    {
      diffXX[row * extWidth + col] = (short) ((((int) convX[row * extWidth + col]) * ((int) convX[row * extWidth + col])) >> 15);
      diffYY[row * extWidth + col] = (short) ((((int) convY[row * extWidth + col]) * ((int) convY[row * extWidth + col])) >> 15);
      diffXY[row * extWidth + col] = (short) ((((int) convX[row * extWidth + col]) * ((int) convY[row * extWidth + col])) >> 15);
    }
  }

  free(convX);
  free(convY);



  // ============================ perform Gaussian filtering ===================================


  //calculate Gaussian filtered version of diffXX
  if(dsp_convolve2d(diffXX, extHeight, extWidth, offset, gaussKernel, gaussKernel, gaussKernelSize, diffXX) != DSP_STATUS_FINISHED)
  {
    free(diffXX);
    free(diffYY);
    free(diffXY);
    return DSP_STATUS_FAILED;
  }

  //calculate Gaussian filtered version of diffYY
  if(dsp_convolve2d(diffYY, extHeight, extWidth, offset, gaussKernel, gaussKernel, gaussKernelSize, diffYY) != DSP_STATUS_FINISHED)
  {
    free(diffXX);
    free(diffYY);
    free(diffXY);
    return DSP_STATUS_FAILED;
  }

  //calculate Gaussian filtered version of diffXY
  if(dsp_convolve2d(diffXY, extHeight, extWidth, offset, gaussKernel, gaussKernel, gaussKernelSize, diffXY) != DSP_STATUS_FINISHED)
  {
    free(diffXX);
    free(diffYY);
    free(diffXY);
    return DSP_STATUS_FAILED;
  }


  // now copy the derived and smoothed values in the output arrays
  for(row = 0; row < height; row++)
  {
    for(col = 0; col < width; col++)
    {
      output_diffXX[row * width + col] = diffXX[(row + offset) * extWidth + (col + offset)];
      output_diffYY[row * width + col] = diffYY[(row + offset) * extWidth + (col + offset)];
      output_diffXY[row * width + col] = diffXY[(row + offset) * extWidth + (col + offset)];
    }
  }

  free(diffXX);
  free(diffYY);
  free(diffXY);


  return DSP_STATUS_FINISHED;
}


int dsp_convolve2d(const short* restrict input, unsigned int height, unsigned int width, unsigned int offset,
    const short* restrict kernel_hor, const short* restrict kernel_ver, unsigned int kernelSize,
    short* restrict output)
{
  // temporary memory for the intermediate result between 1d convolutions
  short* temp = (short*) malloc((width * height + 2 * offset - 1) * sizeof(short));
  if(!temp)
    return DSP_STATUS_FAILED;


  // set all values of temp array to known values
  memset(temp, 0, (width * height + 2 * offset - 1) * sizeof(short));


  // calculate horizontal convolution with hor kernel
  if(width * height % 4 == 0)
    DSP_fir_gen(input, kernel_hor, temp + kernelSize/2, kernelSize, width * height);
  else
    DSP_fir_gen_slow(input, kernel_hor, temp + kernelSize/2, kernelSize, width * height);

  // transpose temporary image to compute vertical convolution
  if(width % 4 == 0 && height % 4 == 0)
    DSP_mat_trans(temp, height, width, output);
  else
    DSP_mat_trans_slow(temp, height, width, output);


  // set all values of temp array again to known values
  memset(temp, 0, (width * height + 2 * offset - 1) * sizeof(short));


  // now calculate vertical convolution ver kernel
  if(width * height % 4 == 0)
    DSP_fir_gen(output, kernel_ver, temp + kernelSize/2, kernelSize, width * height);
  else
    DSP_fir_gen_slow(output, kernel_ver, temp + kernelSize/2, kernelSize, width * height);

  // transpose image again to retrieve original image
  if(width % 4 == 0 && height % 4 == 0)
    DSP_mat_trans(temp, width, height, output);
  else
    DSP_mat_trans_slow(temp, width, height, output);


  free(temp);


  return DSP_STATUS_FINISHED;
}
