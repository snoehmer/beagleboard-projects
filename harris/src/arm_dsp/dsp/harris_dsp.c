/*
 * DSP implementations of functions needed by Harris Corner Detector
 * and Normalized Cross Correlator to speed up calculations
 */

#define GLOBAL_Q 15

#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../../opt/dsplib/inc/dsplib.h"
#include "../../../opt/iqmath/include/IQmath.h"
#include <stddef.h>
#include <string.h>
#include "../common/harris_common.h"
#include "../arm/dspbridge/node.h"


inline void dsp_startTimer(void *env, unsigned char id)
{
  dsp_msg_t msg;

  msg.cmd = DSP_PERFORMANCE;
  msg.arg_1 = DSP_PERF_START;
  msg.arg_2 = id;

  NODE_putMsg(env, NULL, &msg, 0);
}

inline void dsp_stopTimer(void *env, unsigned char id)
{
  dsp_msg_t msg;

  msg.cmd = DSP_PERFORMANCE;
  msg.arg_1 = DSP_PERF_FIN;
  msg.arg_2 = id;

  NODE_putMsg(env, NULL, &msg, 0);
}

inline void dsp_harris_newcorner(void *env, unsigned int row, unsigned int col, unsigned int width, short strength)
{
  dsp_msg_t msg;

  msg.cmd = DSP_HARRIS_NEWCORNER;
  msg.arg_1 = row * width + col;
  msg.arg_2 = strength;

  NODE_putMsg(env, NULL, &msg, 0);
}


void DSP_mat_trans_slow(const short* restrict x, const short rows, const short columns, short* restrict r);
void DSP_fir_gen_slow(const short* restrict x, const short* restrict h, short* restrict r, const int nh, const int nr);


int dsp_perform_harris(void *env, dsp_harris_params *params);

int dsp_convolve2d(const short* restrict input, unsigned int height, unsigned int width, unsigned int offset,
    const short* restrict kernel_hor, const short* restrict kernel_ver, unsigned int kernelSize,
    short* restrict output);


int dsp_perform_ncc_std_getncc(void *env, dsp_ncc_std_getncc_params *params, int *ncc);
int dsp_perform_ncc_std_patchdata(void *env, dsp_ncc_std_patchdata_params *params);

int dsp_perform_ncc_intimg_getncc(void *env, dsp_ncc_intimg_getncc_params *params, int *ncc);
int dsp_perform_ncc_intimg_imagedata(void *env, dsp_ncc_intimg_imagedata_params *params);
int dsp_perform_ncc_intimg_patchdata(void *env, dsp_ncc_intimg_patchdata_params *params);


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
        BCACHE_inv((void*) params, sizeof(dsp_harris_params), 1);

        msg.arg_2 = dsp_perform_harris(env, params);

        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

      case DSP_NCC_STD_GETNCC:
      {
        dsp_ncc_std_getncc_params *params = (dsp_ncc_std_getncc_params*) msg.arg_1;
        BCACHE_inv((void*) params, sizeof(dsp_ncc_std_getncc_params), 1);

        msg.arg_1 = dsp_perform_ncc_std_getncc(env, params, (int*) &msg.arg_2);

        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

      case DSP_NCC_STD_PATCHDATA:
      {
        dsp_ncc_std_patchdata_params *params = (dsp_ncc_std_patchdata_params*) msg.arg_1;
        BCACHE_inv((void*) params, sizeof(dsp_ncc_std_patchdata_params), 1);

        msg.arg_1 = dsp_perform_ncc_std_patchdata(env, params);

        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

      case DSP_NCC_INTIMG_GETNCC:
      {
        dsp_ncc_intimg_getncc_params *params = (dsp_ncc_intimg_getncc_params*) msg.arg_1;
        BCACHE_inv((void*) params, sizeof(dsp_ncc_intimg_getncc_params), 1);

        msg.arg_1 = dsp_perform_ncc_intimg_getncc(env, params, (int*) &msg.arg_2);

        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

      case DSP_NCC_INTIMG_PATCHDATA:
      {
        dsp_ncc_intimg_patchdata_params *params = (dsp_ncc_intimg_patchdata_params*) msg.arg_1;
        BCACHE_inv((void*) params, sizeof(dsp_ncc_intimg_patchdata_params), 1);

        msg.arg_1 = dsp_perform_ncc_intimg_patchdata(env, params);

        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

      case DSP_NCC_INTIMG_IMAGEDATA:
      {
        dsp_ncc_intimg_imagedata_params *params = (dsp_ncc_intimg_imagedata_params*) msg.arg_1;
        BCACHE_inv((void*) params, sizeof(dsp_ncc_intimg_imagedata_params), 1);

        msg.arg_1 = dsp_perform_ncc_intimg_imagedata(env, params);

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


int dsp_perform_harris(void *env, dsp_harris_params *params)
{
  short *input = params->input_;
  unsigned int extHeight = params->height_;
  unsigned int extWidth = params->width_;
  unsigned int offset = params->offset_;

  // derived Gauss kernels for corner detection
  short *devKernel_gauss = params->devKernel_gauss_;
  short *devKernel_gauss_der = params->devKernel_gauss_der_;
  unsigned int devKernelSize = params->devKernelSize_;

  // Gauss kernel for image smoothing
  short *gaussKernel = params->gaussKernel_;
  unsigned int gaussKernelSize = params->gaussKernelSize_;

  // k factor of Harris corner detector
  short harris_k = params->harris_k_;

  // derived kernel for non-maximum suppression
  short *nonMaxKernel = params->nonMaxKernel_;
  short *nonMaxKernel1 = params->nonMaxKernel1_;
  unsigned int nonMaxKernelSize = params->nonMaxKernelSize_;

  // temporary memory for convolutions in x/y-direction
  short *convX = params->convX_;
  short *convY = params->convY_;

  // temporary memory for products of convolutions
  short *diffXX = params->diffXX_;
  short *diffYY = params->diffYY_;
  short *diffXY = params->diffXY_;

  // preview of the convolved images TODO: remove
  short *output_diffXX = params->output_diffXX_;
  short *output_diffYY = params->output_diffYY_;
  short *output_diffXY = params->output_diffXY_;

  // temporary memory for Harris corner response
  short *hcr = params->hcr_;
  short hcrMax = params->hcrMax_;
  short threshold = params->threshold_;

  // temporary memory for non-maximum suppression
  short *nonmaxX = params->nonmaxX_;
  short *nonmaxY = params->nonmaxY_;
  short *nonmaxM2 = params->nonmaxM2_;

  // result of Harris corner detection (normalized HCR)
  short *hcr_out = params->hcr_out_;


  unsigned int extSize = extHeight * extWidth;
  unsigned int extConvSize = extSize + 2 * offset - 1;
  unsigned int height = extHeight - 2 * offset;
  unsigned int width = extWidth - 2 * offset;
  unsigned int size = height * width;

  unsigned int row;
  unsigned int col;


  // invalidate cache for input arrays
  BCACHE_inv((void*) input, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) devKernel_gauss, devKernelSize * sizeof(short), 1);
  BCACHE_inv((void*) devKernel_gauss_der, devKernelSize * sizeof(short), 1);
  BCACHE_inv((void*) gaussKernel, gaussKernelSize * sizeof(short), 1);
  BCACHE_inv((void*) nonMaxKernel, nonMaxKernelSize * sizeof(short), 1);
  BCACHE_inv((void*) nonMaxKernel1, nonMaxKernelSize * sizeof(short), 1);
  BCACHE_inv((void*) convX, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) convY, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) diffXX, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) diffYY, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) diffXY, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) output_diffXX, size * sizeof(short), 1);
  BCACHE_inv((void*) output_diffYY, size * sizeof(short), 1);
  BCACHE_inv((void*) output_diffXY, size * sizeof(short), 1);
  BCACHE_inv((void*) nonmaxX, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) nonmaxY, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) nonmaxM2, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) hcr, extConvSize * sizeof(short), 1);
  BCACHE_inv((void*) hcr_out, size * sizeof(short), 1);


  // ================================ perform corner detection ======================================

  dsp_startTimer(env, DSP_PERF_CONV_CORNERS);

  // calculate x-derivate of image
  if(dsp_convolve2d(input, extHeight, extWidth, offset, devKernel_gauss_der, devKernel_gauss, devKernelSize, convX) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;

  //calculate y-derivate of image
  if(dsp_convolve2d(input, extHeight, extWidth, offset, devKernel_gauss, devKernel_gauss_der, devKernelSize, convY) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;


  // now calculate diffXX (squared x), diffYY (squared y) and diffXY (x * y)
  memset(diffXX, 0, extConvSize * sizeof(short));
  memset(diffYY, 0, extConvSize * sizeof(short));
  memset(diffXY, 0, extConvSize * sizeof(short));

  #pragma MUST_ITERATE(1,,1)
  for(row = offset; row < offset + height; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = offset; col < offset + width; col++)
    {
      // shift by 13 (instead of 15) to increase resolution with short variable (later normalized anyway)
      diffXX[row * extWidth + col] = (short) ((((int) convX[row * extWidth + col]) * ((int) convX[row * extWidth + col])) >> 13);
      diffYY[row * extWidth + col] = (short) ((((int) convY[row * extWidth + col]) * ((int) convY[row * extWidth + col])) >> 13);
      diffXY[row * extWidth + col] = (short) ((((int) convX[row * extWidth + col]) * ((int) convY[row * extWidth + col])) >> 13);
    }
  }

  dsp_stopTimer(env, DSP_PERF_CONV_CORNERS);


  // ================================ perform Gaussian filtering ===================================

  dsp_startTimer(env, DSP_PERF_CONV_GAUSS);

  //calculate Gaussian filtered version of diffXX
  if(dsp_convolve2d(diffXX, extHeight, extWidth, offset, gaussKernel, gaussKernel, gaussKernelSize, diffXX) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;

  //calculate Gaussian filtered version of diffYY
  if(dsp_convolve2d(diffYY, extHeight, extWidth, offset, gaussKernel, gaussKernel, gaussKernelSize, diffYY) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;

  //calculate Gaussian filtered version of diffXY
  if(dsp_convolve2d(diffXY, extHeight, extWidth, offset, gaussKernel, gaussKernel, gaussKernelSize, diffXY) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;

  dsp_stopTimer(env, DSP_PERF_CONV_GAUSS);


  // ============================== calculate Harris corner response ================================

  dsp_startTimer(env, DSP_PERF_HCR);

  memset(hcr, 0, extConvSize * sizeof(short));

  int Ixx, Iyy, Ixy, Ixxyy, Ixy2, Ixxpyy, Ixxpyy2, kIxxpyy2;

  #pragma MUST_ITERATE(1,,1)
  for(row = offset; row < offset + height; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = offset; col < offset + width; col++)
    {
      Ixx = diffXX[row * extWidth + col];
      Iyy = diffYY[row * extWidth + col];
      Ixy = diffXY[row * extWidth + col];

      Ixxyy = (((int) Ixx) * ((int) Iyy));
      Ixy2 = (((int) Ixy) * ((int) Ixy));
      Ixxpyy = ((int) Ixx) + ((int) Iyy);
      Ixxpyy2 = (((int) Ixxpyy) * ((int) Ixxpyy));
      kIxxpyy2 = (((long) harris_k) * ((long) Ixxpyy2)) >> 15;

      // shift by 13 to increase resolution with short variable (later normalized anyway)
      hcr[row * extWidth + col] = (short) ((Ixxyy - Ixy2 - kIxxpyy2) >> 13);
    }
  }

  dsp_stopTimer(env, DSP_PERF_HCR);


  // ============================== perform non-maximum suppression ================================

  dsp_startTimer(env, DSP_PERF_NONMAX_CONV);

  memset(nonmaxX, 0, extConvSize * sizeof(short));
  memset(nonmaxY, 0, extConvSize * sizeof(short));

  // convolve with standard derived kernels
  if(dsp_convolve2d(hcr, extHeight, extWidth, offset, nonMaxKernel, nonMaxKernel1, nonMaxKernelSize, nonmaxX) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;

  if(dsp_convolve2d(hcr, extHeight, extWidth, offset, nonMaxKernel1, nonMaxKernel, nonMaxKernelSize, nonmaxY) != DSP_STATUS_FINISHED)
    return DSP_STATUS_FAILED;


  // calculate squared magnitude
  #pragma MUST_ITERATE(1,,1)
  for(row = offset; row < offset + height; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = offset; col < offset + width; col++)
    {
      nonmaxM2[row * extWidth + col] = _IQsqrt(_IQmpy((int) nonmaxX[row * extWidth + col], (int) nonmaxX[row * extWidth + col]) + _IQmpy((int) nonmaxY[row * extWidth + col], (int) nonmaxY[row * extWidth + col]));
    }
  }

  dsp_stopTimer(env, DSP_PERF_NONMAX_CONV);

  dsp_startTimer(env, DSP_PERF_NONMAX_NONMAX);


  // perform non-maximum suppression algorithm
  int irow, icol;
  short dX, dY, a1, a2, b1, b2;
  int A, B, P;

  memset(hcr, 0, extConvSize * sizeof(short));

  #pragma MUST_ITERATE(1,,1)
  for(row = offset + 1; row < offset + height - 1; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = offset + 1; col < offset + width - 1; col++)
    {
      dX = nonmaxX[row * extWidth + col];
      dY = nonmaxY[row * extWidth + col];

      // set increments for different quadrants
      if(dX > 0) irow = 1;
      else irow = -1;

      if(dY > 0) icol = 1;
      else icol = -1;

      if(abs(dX) > abs(dY))
      {
        a1 = nonmaxM2[(row) * extWidth + (col + icol)];
        a2 = nonmaxM2[(row - irow) * extWidth + (col + icol)];
        b1 = nonmaxM2[(row) * extWidth + (col - icol)];
        b2 = nonmaxM2[(row + irow) * extWidth + (col - icol)];

        A = ((int) (abs(dX) - abs(dY))) * ((int) a1) + ((int) abs(dY)) * ((int) a2);
        B = ((int) (abs(dX) - abs(dY))) * ((int) b1) + ((int) abs(dY)) * ((int) b2);

        P = ((int) nonmaxM2[row * extWidth + col]) * ((int) abs(dX));

        if(P >= A && P > B)
        {
          hcr[row * extWidth + col] = abs(dX); //magnitude[row * width + col];
        }
      }
      else
      {
        a1 = nonmaxM2[(row - irow) * extWidth + (col)];
        a2 = nonmaxM2[(row - irow) * extWidth + (col + icol)];
        b1 = nonmaxM2[(row + irow) * extWidth + (col)];
        b2 = nonmaxM2[(row + irow) * extWidth + (col - icol)];

        A = ((int) (abs(dY) - abs(dX))) * ((int) a1) + ((int) abs(dX)) * ((int) a2);
        B = ((int) (abs(dY) - abs(dX))) * ((int) b1) + ((int) abs(dX)) * ((int) b2);

        P = ((int) nonmaxM2[row * extWidth + col]) * ((int) abs(dY));

        if(P >= A && P > B)
        {
          hcr[row * extWidth + col] = abs(dY); //magnitude[row * width + col];
        }
      }
    }
  }

  dsp_stopTimer(env, DSP_PERF_NONMAX_NONMAX);


  // ========================== normalize Harris corner response to 0..1 =============================

  dsp_startTimer(env, DSP_PERF_NORMTHRESH);

  int i;
  short min, max;

  if(extWidth * extHeight % 8 == 0 && extWidth * extHeight >= 32)
  {
    min = DSP_minval(hcr, extWidth * extHeight);
    max = DSP_maxval(hcr, extWidth * extHeight);
  }
  else
  {
    min = max = hcr[0];

    #pragma MUST_ITERATE(1,,1)
    for(i = 0; i < extWidth * extHeight; i++)
    {
      if(hcr[i] > max) max = hcr[i];
      if(hcr[i] < min) min = hcr[i];
    }
  }


  // =================== perform thresholding and generate cropped output values ====================

  #pragma MUST_ITERATE(1,,1)
  for(row = 0; row < height; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = 0; col < width; col++)
    {
      output_diffXX[row * width + col] = diffXX[(row + offset) * extWidth + (col + offset)];
      output_diffYY[row * width + col] = diffYY[(row + offset) * extWidth + (col + offset)];
      output_diffXY[row * width + col] = diffXY[(row + offset) * extWidth + (col + offset)];

      hcr_out[row * width + col] = _IQdiv(_IQmpy((int) (hcr[(row + offset) * extWidth + (col + offset)] - min), (int) hcrMax), (int) (max - min));

      if(hcr_out[row * width + col] >= threshold)
      {
        dsp_harris_newcorner(env, row, col, width, hcr_out[row * width + col]);
      }
      else
      {
        hcr_out[row * width + col] = 0;
      }
    }
  }

  dsp_stopTimer(env, DSP_PERF_NORMTHRESH);


  // invalidate cache (writeback) for output arrays
  BCACHE_wbInv((void*) convX, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) convY, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) diffXX, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) diffYY, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) diffXY, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) output_diffXX, size * sizeof(short), 1);
  BCACHE_wbInv((void*) output_diffYY, size * sizeof(short), 1);
  BCACHE_wbInv((void*) output_diffXY, size * sizeof(short), 1);
  BCACHE_wbInv((void*) hcr, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) nonmaxX, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) nonmaxY, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) nonmaxM2, extConvSize * sizeof(short), 1);
  BCACHE_wbInv((void*) hcr_out, size * sizeof(short), 1);


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


int dsp_perform_ncc_std_getncc(void *env, dsp_ncc_std_getncc_params *params, int *ncc)
{
  int* restrict input = params->input_;
  int width = params->width_;
  int height = params->height_;

  int row = params->row_;
  int col = params->col_;

  int patchAvg = params->patchAvg_;
  int* restrict patchNorm = params->patchNorm_;
  int* restrict patchNormSq = params->patchNormSq_;
  int patchSqSum = params->patchSqSum_;

  int patchSize = params->patchSize_;

  int irow, icol, prow, pcol;


  BCACHE_inv((void*) input, width * height * sizeof(int), 1);
  BCACHE_inv((void*) patchNorm, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) patchNormSq, patchSize * patchSize * sizeof(int), 1);


  // calculate average of image at current patch
  int iavg = 0;

  #pragma MUST_ITERATE(1,,1)
  for(prow = 0, irow = row - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(pcol = 0, icol = col - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
    {
      iavg += input[irow * width + icol]; // must be scale_uchar'ed!
    }
  }

  iavg = _IQdiv(iavg, _IQmpy(_IQ(patchSize), _IQ(patchSize)));


  // calculate NCC
  int inorm;

  int sumIP = 0;
  int sumPP = 0;
  int sumII = 0;

  #pragma MUST_ITERATE(1,,1)
  for(prow = 0, irow = row - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(pcol = 0, icol = col - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
    {
      inorm = input[irow * width + icol] - iavg;

      sumIP += _IQmpy(patchNorm[prow * patchSize + pcol], inorm);
      sumPP += patchNormSq[prow * patchSize + pcol];
      sumII += _IQmpy(inorm, inorm);
    }
  }

  if(sumPP == 0 || sumII == 0)  // we wont find any corners in a homogenous area
  {
    *ncc = 0;
  }
  else
  {
    *ncc = _IQdiv(sumIP, _IQsqrt(_IQmpy(sumPP, sumII)));
  }

  return DSP_STATUS_FINISHED;
}

int dsp_perform_ncc_std_patchdata(void *env, dsp_ncc_std_patchdata_params *params)
{
  int* restrict patch = params->patch_;

  int* restrict patchAvg = params->patchAvg_;
  int* restrict patchNorm = params->patchNorm_;
  int* restrict patchNormSq = params->patchNormSq_;
  int* restrict patchSqSum = params->patchSqSum_;

  int patchSize = params->patchSize_;


  BCACHE_inv((void*) patch, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) patchAvg, sizeof(int), 1);
  BCACHE_inv((void*) patchNorm, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) patchNormSq, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) patchSqSum, sizeof(int), 1);


  // calculate average of feature patch
  int row, col;
  int psum = 0;

  #pragma MUST_ITERATE(1,,1)
  for(row = 0; row < patchSize; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = 0; col < patchSize; col++)
    {
      psum += patch[row * patchSize + col];
    }
  }

  *patchAvg = _IQdiv(psum, _IQmpy(_IQ(patchSize), _IQ(patchSize)));


  // now calculate normalized patch and squared normalized patch
  #pragma MUST_ITERATE(1,,1)
  for(row = 0; row < patchSize; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = 0; col < patchSize; col++)
    {
      patchNorm[row * patchSize + col] = patch[row * patchSize + col] - *patchAvg;

      patchNormSq[row * patchSize + col] = _IQmpy(patchNorm[row * patchSize + col], patchNorm[row * patchSize + col]);
    }
  }


  BCACHE_wbInv((void*) patchAvg, sizeof(int), 1);
  BCACHE_wbInv((void*) patchNorm, patchSize * patchSize * sizeof(int), 1);
  BCACHE_wbInv((void*) patchNormSq, patchSize * patchSize * sizeof(int), 1);
  BCACHE_wbInv((void*) patchSqSum, sizeof(int), 1);


  return DSP_STATUS_FINISHED;
}


int dsp_perform_ncc_intimg_getncc(void *env, dsp_ncc_intimg_getncc_params *params, int *ncc)
{
  int* restrict input = params->input_;
  int width = params->width_;
  int height = params->height_;

  int row = params->row_;
  int col = params->col_;

  int* restrict imageSqSum = params->imageSqSum_;

  int* restrict patchNorm = params->patchNorm_;
  int patchSqSum = params->patchSqSum_;

  int patchSize = params->patchSize_;

  int irow, icol, prow, pcol;


  BCACHE_inv((void*) input, width * height * sizeof(int), 1);
  BCACHE_inv((void*) patchNorm, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) imageSqSum, width * height * sizeof(int), 1);


  int sumIP = 0;

  if(imageSqSum[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)] == 0)
  {
    ncc = 0;  // completely homogenous area -> we wont find matches here!
  }
  else
  {
    #pragma MUST_ITERATE(1,,1)
    for(prow = 0, irow = row - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
    {
      #pragma MUST_ITERATE(1,,1)
      for(pcol = 0, icol = col - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
      {
        sumIP += _IQmpy(patchNorm[prow * patchSize + pcol], input[irow * width + icol]);
      }
    }

    *ncc = _IQdiv(sumIP, (_IQmpy(patchSqSum, imageSqSum[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)])));
  }

  return DSP_STATUS_FINISHED;
}


int dsp_perform_ncc_intimg_imagedata(void *env, dsp_ncc_intimg_imagedata_params *params)
{
  int* restrict image = params->image_;

  unsigned int width = params->width_;
  unsigned int height = params->height_;

  int* restrict imageIntegral = params->imageIntegral_;
  int* restrict imageIntegral2 = params->imageIntegral2_;
  int* restrict imageAvg = params->imageAvg_;
  int* restrict imageSqSum = params->imageSqSum_;

  int patchSize = params->patchSize_;


  BCACHE_inv((void*) image, width * height * sizeof(int), 1);
  BCACHE_inv((void*) imageIntegral, (width + 1) * (height + 1) * sizeof(int), 1);
  BCACHE_inv((void*) imageIntegral2, (width + 1) * (height + 1) * sizeof(int), 1);
  BCACHE_inv((void*) imageAvg, (width - patchSize) * (height - patchSize) * sizeof(int), 1);
  BCACHE_inv((void*) imageSqSum, (width - patchSize) * (height - patchSize) * sizeof(int), 1);


  unsigned int row, col;


  // initialize integral arrays
  #pragma MUST_ITERATE(1,,1)
  for(col = 0; col < width + 1; col++)  // fill 1st row with zeros
  {
    imageIntegral[0 * (width + 1) + col] = 0;
    imageIntegral2[0 * (width + 1) + col] = 0;
  }

  #pragma MUST_ITERATE(1,,1)
  for(row = 1; row < height + 1; row++)  // fill 1st column with zeros
  {
    imageIntegral[row * (width + 1) + 0] = 0;
    imageIntegral2[row * (width + 1) + 0] = 0;
  }


  // calculate integral arrays
  #pragma MUST_ITERATE(1,,1)
  for(row = 0; row < height; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = 0; col < width; col++)
    {
      imageIntegral[(row + 1) * (width + 1) + (col + 1)] = image[row * width + col] + imageIntegral[row * (width + 1) + (col + 1)] + imageIntegral[(row + 1) * (width + 1) + col] - imageIntegral[row * (width + 1) + col];
      imageIntegral2[(row + 1) * (width + 1) + (col + 1)] = _IQmpy(image[row * width + col], image[row * width + col]) + imageIntegral2[row * (width + 1) + (col + 1)] + imageIntegral2[(row + 1) * (width + 1) + col] - imageIntegral2[row * (width + 1) + col];
    }
  }

  // calculate sqSum array
  unsigned int offset = patchSize / 2;
  unsigned int imageWidth = width - patchSize;

  int A, B;

  int dp = patchSize / 2;  // positive delta
  int dn = (patchSize - 1) / 2 + 1;  // negative delta

  #pragma MUST_ITERATE(1,,1)
  for(row = offset; row < height - offset; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = offset; col < width - offset; col++)
    {
      A = imageIntegral2[(row + dp + 1) * (width + 1) + (col + dp + 1)] - imageIntegral2[(row - dn + 1) * (width + 1) + (col + dp + 1)] - imageIntegral2[(row + dp + 1) * (width + 1) + (col - dn + 1)] + imageIntegral2[(row - dn + 1) * (width + 1) + (col - dn + 1)];
      B = imageIntegral[(row + dp + 1) * (width + 1) + (col + dp + 1)] - imageIntegral[(row - dn + 1) * (width + 1) + (col + dp + 1)] - imageIntegral[(row + dp + 1) * (width + 1) + (col - dn + 1)] + imageIntegral[(row - dn + 1) * (width + 1) + (col - dn + 1)];

      imageSqSum[(row - offset) * imageWidth + (col - offset)] = _IQsqrt(A - _IQdiv(_IQmpy(B, B), _IQmpy(_IQ(patchSize), _IQ(patchSize))));
      imageAvg[(row - offset) * imageWidth + (col - offset)] = _IQdiv(B, _IQmpy(_IQ(patchSize), _IQ(patchSize)));
    }
  }


  BCACHE_wbInv((void*) image, width * height * sizeof(int), 1);
  BCACHE_wbInv((void*) imageIntegral, (width + 1) * (height + 1) * sizeof(int), 1);
  BCACHE_wbInv((void*) imageIntegral2, (width + 1) * (height + 1) * sizeof(int), 1);
  BCACHE_wbInv((void*) imageAvg, (width - patchSize) * (height - patchSize) * sizeof(int), 1);
  BCACHE_wbInv((void*) imageSqSum, (width - patchSize) * (height - patchSize) * sizeof(int), 1);


  return DSP_STATUS_FINISHED;
}


int dsp_perform_ncc_intimg_patchdata(void *env, dsp_ncc_intimg_patchdata_params *params)
{
  int* restrict patch = params->patch_;

  int* restrict patchAvg = params->patchAvg_;
  int* restrict patchNorm = params->patchNorm_;
  int* restrict patchSqSum = params->patchSqSum_;

  int patchSize = params->patchSize_;


  BCACHE_inv((void*) patch, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) patchAvg, sizeof(int), 1);
  BCACHE_inv((void*) patchNorm, patchSize * patchSize * sizeof(int), 1);
  BCACHE_inv((void*) patchSqSum, sizeof(int), 1);


  // calculate average of feature patch
  unsigned int row, col;
  int psum = 0;

  #pragma MUST_ITERATE(1,,1)
  for(row = 0; row < patchSize; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = 0; col < patchSize; col++)
    {
      psum += patch[row * patchSize + col];
    }
  }

  *patchAvg = _IQdiv(psum, _IQmpy(_IQ(patchSize), _IQ(patchSize)));


  // now calculate normalized patch and sqSum
  int sqSum = 0;

  #pragma MUST_ITERATE(1,,1)
  for(row = 0; row < patchSize; row++)
  {
    #pragma MUST_ITERATE(1,,1)
    for(col = 0; col < patchSize; col++)
    {
      patchNorm[row * patchSize + col] = patch[row * patchSize + col] - *patchAvg;

      sqSum += _IQmpy(patchNorm[row * patchSize + col], patchNorm[row * patchSize + col]);
    }
  }

  *patchSqSum = _IQsqrt(sqSum);


  BCACHE_wbInv((void*) patchAvg, sizeof(int), 1);
  BCACHE_wbInv((void*) patchNorm, patchSize * patchSize * sizeof(int), 1);
  BCACHE_wbInv((void*) patchSqSum, sizeof(int), 1);

  return DSP_STATUS_FINISHED;
}
