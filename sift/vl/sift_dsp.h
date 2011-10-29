/*
 * sift_dsp.h
 *
 *  Created on: 03.09.2011
 *      Author: tom
 */



#ifndef SIFT_DSP_H_
#define SIFT_DSP_H_

#ifndef ARCH_DSP
#include "convolutionkernel.h"
#endif



enum
{
  DSP_CALC_IMCONVOL_VF = 1,
  DSP_CALC_GAUSSIAN_FIXEDPOINT,
  DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAIN,
  DSP_CALC_IMCONVOL_VF_FINISHED = 100,
  DSP_CALC_GAUSSIAN_FIXEDPOINT_FINISHED,
  DSP_CALC_IMCONVOL_VF_FAILED,
  DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAINED_FINISHED,
  DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAINED_FAILED
};

typedef struct _imconvol_vf_params
{
  float* dst;
  unsigned dst_size;
  int dst_stride;
  float const* src;
  unsigned src_size;
  int src_width;
  int src_height;
  int src_stride;
  float const* filt;
  unsigned filt_size;
  int filt_begin;
  int filt_end;
  int step;
  unsigned int flags;

  char* dbg_str;
  int dbg_str_size;
}imconvol_vf_params;

typedef struct _filterImageGaussian_params
{
  short* inputOutputImage;
  int inputOutputImageSize;
  int width;
  int height;
  ConvolutionKernelRef gauss;
}filterImageGaussian_params;

typedef struct
{
  short* inputImage;  //if inputImage == 0, then the last outputImage is used
  short* dogOutImage; //if dogOutImage == 0, no Dog calculation is performed!
  int inputOutputImageSize;
  short* outputImage;
  int width;
  int height;
  ConvolutionKernelRef gauss;
  int filterCount; //how many times the gaussian filter should be applied ...
}filterImageGaussian_chained_params;

typedef struct
{
  short* outputImage;
  float sigma;
  short* dogOutImage; //Difference is calced to previous image, if 0, then no Dog is calculated...
  int filterCount;//how many times the gaussian filter should be applied ...
}DestinationImage;

void vl_imconvcol_vf_on_dsp(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags);

void debugParams(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags);

void filterImageGaussian_on_dsp(short* inputOutputImage,
    int width, int height,
    ConvolutionKernel gauss);


void filterMultipleTimes_on_dsp(short* imputImage,
    int width, int height, DestinationImage destinations[], int numDestinations);

#ifdef ARCH_DSP
//defininition for DSP, to avoid including the whole vlfeat stuff...
void vl_imconvcol_vf(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags);

#define VL_PAD_BY_ZERO       (0x0 << 0) /**< @brief Pad with zeroes. */
#define VL_PAD_BY_CONTINUITY (0x1 << 0) /**< @brief Pad by continuity. */
#define VL_PAD_MASK          (0x3)      /**< @brief Padding field selector. */
#define VL_TRANSPOSE         (0x1 << 2) /**< @brief Transpose result. */

#define vl_bool int

/** ------------------------------------------------------------------
 ** @name Common operations
 ** @{ */

/** @brief Min operation
 ** @param x value
 ** @param y value
 ** @return the minimum of @a x and @a y.
 **/
#define VL_MIN(x,y) (((x)<(y))?(x):(y))

/** @brief Max operation
 ** @param x value.
 ** @param y value.
 ** @return the maximum of @a x and @a y.
 **/
#define VL_MAX(x,y) (((x)>(y))?(x):(y))

#endif

#endif /* SIFT_DSP_H_ */
