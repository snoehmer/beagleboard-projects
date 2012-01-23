/*
 * DSP implementations of functions needed by Harris Corner Detector
 * and Normalized Cross Correlator to speed up calculations
 */

#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>
#include <dsplib.h>
#include <stddef.h>
#include <string.h>
#include "../common/harris_common.h"
#include "../arm/dspbridge/node.h"


int dsp_harris_convolve_harris(const short* restrict input, unsigned int extHeight,
    unsigned int extWidth, unsigned int offset, const short* restrict kernel_gauss,
    const short* restrict kernel_gauss2, unsigned int kSize, short* restrict output_diffXX,
    short* restrict output_diffYY, short* restrict output_diffXY);


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
      case DSP_HARRIS_CALC_CONVOLUTION:
      {
        dsp_harris_conv_params *params = (dsp_harris_conv_params*) msg.arg_1;

        short *input = params->input_;

        BCACHE_inv((void*) params, sizeof(dsp_harris_conv_params), 1);

        unsigned int extHeight = params->height_;
        unsigned int extWidth = params->width_;
        unsigned int offset = params->offset_;
        short *kernel_gauss = params->kernel_gauss_;
        short *kernel_gauss2 = params->kernel_gauss2_;
        unsigned int kSize = params->kSize_;
        short *output_diffXX = params->output_diffXX_;
        short *output_diffYY = params->output_diffYY_;
        short *output_diffXY = params->output_diffXY_;

        unsigned int extSize = extHeight * extWidth;

        unsigned int height = extHeight - 2 * offset;
        unsigned int width = extWidth - 2 * offset;
        unsigned int size = height * width;

        BCACHE_inv((void*) input, extSize * sizeof(short), 1);
        BCACHE_inv((void*) kernel_gauss, kSize * kSize * sizeof(short), 1);
        BCACHE_inv((void*) kernel_gauss2, kSize * kSize * sizeof(short), 1);
        BCACHE_inv((void*) output_diffXX, size * sizeof(short), 1);
        BCACHE_inv((void*) output_diffYY, size * sizeof(short), 1);
        BCACHE_inv((void*) output_diffXY, size * sizeof(short), 1);

        int result = dsp_harris_convolve_harris(input, extHeight, extWidth, offset,
            kernel_gauss, kernel_gauss2, kSize, output_diffXX, output_diffYY, output_diffXY);

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

void DSP_mat_trans_slow(const short *x, const short rows, const short columns, short *r)
{
  short i,j;
    for(i=0; i<columns; i++)
      for(j=0; j<rows; j++)
        *(r+i*rows+j)=*(x+i+columns*j);
}

void DSP_fir_gen_slow
(
  const short * x, // Input ('nr + nh - 1' samples)
  const short * h, // Filter coefficients (nh taps)
  short * r, // Output array ('nr' samples)
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
      sum += x[i + j] * h[i];
    }
    r[j] = sum >> 15;
  }
}


int dsp_harris_convolve_harris(const short* restrict input, unsigned int extHeight,
    unsigned int extWidth, unsigned int offset, const short* restrict kernel_gauss,
    const short* restrict kernel_gauss2, unsigned int kSize, short* restrict output_diffXX,
    short* restrict output_diffYY, short* restrict output_diffXY)
{
  unsigned int height = extHeight - 2 * offset;
  unsigned int width = extWidth - 2 * offset;
  unsigned int size = height * width;
  unsigned int radius = size / 2;

  // results of the 2d convolution by 2 1d convolutions
  short *convX = (short*) memalign(8, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));
  short *convY = (short*) memalign(8, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));

  // temporary memory for the intermediate result between 1d convolutions
  short *temp = (short*) memalign(8, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));

  // set all values of temp array to known values
  memset(temp, 0, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));


  // calculate horizontal convolution with x-derived kernel (derived gauss)
  if(extWidth * extHeight % 4 == 0)
    DSP_fir_gen(input, kernel_gauss2, temp + 8, kSize, extWidth * extHeight);
  else
    DSP_fir_gen_slow(input, kernel_gauss2, temp + 8, kSize, extWidth * extHeight);

  // transpose temporary image to compute vertical convolution with x-derived kernel
  if(extWidth % 4 == 0 && extHeight % 4 == 0)
    DSP_mat_trans(temp + 8 - radius, extHeight, extWidth, convX);
  else
    DSP_mat_trans_slow(temp + 8 - radius, extHeight, extWidth, convX);

  // set all values of temp array again to known values
  memset(temp, 0, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));

  // now calculate vertical convolution with x-derived kernel (standard gauss)
  if(extWidth * extHeight % 4 == 0)
    DSP_fir_gen(convX, kernel_gauss, temp + 8, kSize, extWidth * extHeight);
  else
    DSP_fir_gen_slow(convX, kernel_gauss, temp + 8, kSize, extWidth * extHeight);

  // transpose image again to retrieve original image
  if(extWidth % 4 == 0 && extHeight % 4 == 0)
    DSP_mat_trans(temp + 8 - radius, extHeight, extWidth, convX);
  else
    DSP_mat_trans_slow(temp + 8 - radius, extHeight, extWidth, convX);


  // now calculate convolution with y-derived kernel in the same way (except swapped kernels)
  memset(temp, 0, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));

  if(extWidth * extHeight % 4 == 0)
    DSP_fir_gen(input, kernel_gauss2, temp + 8, kSize, extWidth * extHeight);
  else
    DSP_fir_gen_slow(input, kernel_gauss2, temp + 8, kSize, extWidth * extHeight);

  if(extWidth % 4 == 0 && extHeight % 4 == 0)
    DSP_mat_trans(temp + 8 - radius, extHeight, extWidth, convX);
  else
    DSP_mat_trans_slow(temp + 8 - radius, extHeight, extWidth, convX);

  memset(temp, 0, (8 + extWidth * extHeight + kSize - 1) * sizeof(short));

  if(extWidth * extHeight % 4 == 0)
    DSP_fir_gen(convX, kernel_gauss, temp + 8, kSize, extWidth * extHeight);
  else
    DSP_fir_gen_slow(convX, kernel_gauss, temp + 8, kSize, extWidth * extHeight);

  if(extWidth % 4 == 0 && extHeight % 4 == 0)
    DSP_mat_trans(temp + 8 - radius, extHeight, extWidth, convX);
  else
    DSP_mat_trans_slow(temp + 8 - radius, extHeight, extWidth, convX);


  free(temp);

  int row;
  int col;

  // now calculate diffXX (squared x), diffYY (squared y) and diffXY (x * y)
  for(row = 0; row < height; row++)
  {
    for(col = 0; col < width; col++)
    {
      output_diffXX[row * width + col] = (short) ((int) convX[(row + offset) * extWidth + (col + offset)] * (int) convX[(row + offset) * extWidth + (col + offset)]) >> 15;
      output_diffYY[row * width + col] = (short) ((int) convY[(row + offset) * extWidth + (col + offset)] * (int) convY[(row + offset) * extWidth + (col + offset)]) >> 15;
      output_diffXY[row * width + col] = (short) ((int) convX[(row + offset) * extWidth + (col + offset)] * (int) convY[(row + offset) * extWidth + (col + offset)]) >> 15;
    }
  }


  free(convX);
  free(convY);


  return DSP_STATUS_FINISHED;
}
