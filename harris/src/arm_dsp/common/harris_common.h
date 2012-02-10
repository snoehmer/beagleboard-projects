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
#define DSP_HARRIS_NEWCORNER 11
#define DSP_PERFORMANCE 12

#define DSP_PERFORM_NCC 30
#define DSP_NCC_STD_PATCHDATA 31
#define DSP_NCC_STD_GETNCC 32
#define DSP_NCC_INTIMG_PATCHDATA 35
#define DSP_NCC_INTIMG_IMAGEDATA 36
#define DSP_NCC_INTIMG_GETNCC 37


// special arguments
#define DSP_PERF_START 99
#define DSP_PERF_FIN 100

#define DSP_PERF_CONV_CORNERS 101
#define DSP_PERF_CONV_GAUSS 102
#define DSP_PERF_HCR 103
#define DSP_PERF_NONMAX_CONV 104
#define DSP_PERF_NONMAX_NONMAX 105
#define DSP_PERF_NORMTHRESH 106


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
  short hcrMax_;                  // maximum value for HCR normalization (0..hcrMax)
  short threshold_;               // threshold used for HCR thresholding

  short *nonmaxX_;                // nonmax derivative in x direction
  short *nonmaxY_;                // nonmax derivative in y direction
  short *nonmaxM2_;               // nonmax squared magnitude

  short *hcr_out_;                // thresholded and cropped ready-to-go HCR
} dsp_harris_params;


typedef struct dsp_harris_corner_
{
  unsigned int row_;
  unsigned int col_;
  short strength_;
} dsp_harris_corner;


typedef struct dsp_ncc_std_getncc_params_
{
  int *input_;
  int width_;
  int height_;

  int row_;
  int col_;

  int patchAvg_;
  int *patchNorm_;
  int *patchNormSq_;
  int patchSqSum_;

  int patchSize_;
} dsp_ncc_std_getncc_params;


typedef struct dsp_ncc_std_patchdata_params_
{
  int *patch_;

  int *patchAvg_;
  int *patchNorm_;
  int *patchNormSq_;
  int *patchSqSum_;

  int patchSize_;
} dsp_ncc_std_patchdata_params;


typedef struct dsp_ncc_intimg_getncc_params_
{
  int *input_;
  int width_;
  int height_;

  int row_;
  int col_;

  int *imageSqSum_;

  int *patchNorm_;
  int patchSqSum_;

  int patchSize_;
} dsp_ncc_intimg_getncc_params;


typedef struct dsp_ncc_intimg_imagedata_params_
{
  int *image_;
  int width_;
  int height_;

  int *imageIntegral_;
  int *imageIntegral2_;
  int *imageAvg_;
  int *imageSqSum_;

  int patchSize_;
} dsp_ncc_intimg_imagedata_params;


typedef struct dsp_ncc_intimg_patchdata_params_
{
  int *patch_;

  int *patchAvg_;
  int *patchNorm_;
  int *patchNormSq_;
  int *patchSqSum_;

  int patchSize_;
} dsp_ncc_intimg_patchdata_params;


#endif /* HARRIS_COMMON_H_ */
