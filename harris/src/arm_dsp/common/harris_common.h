/*
 * harris_common.h
 *
 *  Created on: 22.01.2012
 *      Author: sn
 *
 *  This file contains definitions needed both on the ARM and the DSP.
 */

#ifndef HARRIS_COMMON_H_
#define HARRIS_COMMON_H_


// status identifiers (arg_2)
#define DSP_STATUS_FINISHED 0
#define DSP_STATUS_FAILED 1


// message identifiers:
#define DSP_HARRIS_CALC_CONVOLUTION 10
#define DSP_HARRIS_PERFORM_COMPLETE 30

#define DSP_NCC_CALC_NCC 40


// structures for DSP-ARM communication
typedef struct dsp_harris_conv_params_
{
  short *input_;                // input image in fixed point representation (with borders)
  unsigned int height_;         // size of (extended) input image
  unsigned int width_;
  unsigned int offset_;         // image offset (border for convolution)
  short *devKernel_gauss_;      // standard Gauss convolution kernel for edge detection (derivative)
  short *devKernel_gauss_der_;  // derived Gauss convolution kernel for edge detection (derivative)
  unsigned int devKernelSize_;  // size of the derivative kernel
  short *gaussKernel_;          // Gauss kernel for image smoothing
  unsigned int gaussKernelSize_;// size of the smoothing kernel
  short *output_diffXX_;        // output images: derives in different directions, Gaussian filtered
  short *output_diffYY_;
  short *output_diffXY_;
} dsp_harris_conv_params;



#endif /* HARRIS_COMMON_H_ */
