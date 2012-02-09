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
#define DSP_PERFORM_HARRIS 10
#define DSP_PERFORM_NCC 30


// structures for DSP-ARM communication
typedef struct dsp_harris_params_
{
  short *input_;                  // input image in fixed point representation (with borders)
  unsigned int height_;           // size of (extended) input image
  unsigned int width_;
  unsigned int offset_;           // image offset (border for convolution)

  short *devKernel_gauss_;        // standard Gauss convolution kernel for edge detection (derivative)
  short *devKernel_gauss_der_;    // derived Gauss convolution kernel for edge detection (derivative)
  unsigned int devKernelSize_;    // size of the derivative kernel

  short *gaussKernel_;            // Gauss kernel for image smoothing
  unsigned int gaussKernelSize_;  // size of the smoothing kernel

  short harris_k_;                // k factor of Harris corner detector

  short *nonMaxKernel_;           // kernel used for non-maximum suppression
  short *nonMaxKernel1_;
  unsigned int nonMaxKernelSize_; // size of the nonMax kernel

  //short *temp;                    // temporary storage between 2 1d convolutions

  short *convX_;                  // input convolved in x/y-direction
  short *convY_;

  short *diffXX_;                 // derives in different directions, Gaussian filtered
  short *diffYY_;
  short *diffXY_;

  short *output_diffXX_;          // derives in different directions, Gaussian filtered, cropped
  short *output_diffYY_;
  short *output_diffXY_;

  short *hcr_;                    // Harris corner response, non-maximum suppressed

  short *nonmaxX_;                // nonmax derivative in x direction
  short *nonmaxY_;                // nonmax derivative in y direction
  short *nonmaxM2_;               // nonmax squared magnitude

  short *hcr_out_;                // thresholded and cropped ready-to-go HCR
} dsp_harris_params;



#endif /* HARRIS_COMMON_H_ */
