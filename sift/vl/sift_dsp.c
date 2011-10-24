/*
 * sift_dsp.c
 *
 *  Created on: 03.09.2011
 *      Author: tom
 */

#include "convolutionkernel.h"
#include "sift.h"
#include "sift_dsp.h"
#include "imopv.h"

#include <string.h>

#define UDSP


int
my_write_pgm_image_fixed (const vl_sift_pix_fixed* image, int width, int height, const char* filename);


void filterMultipleTimes_on_dsp(short* inputImage,
    int width, int height, DestinationImage destinations[], int numDestinations)
{
  VL_PRINTF("calling filterMultipleTimes_on_dsp, numDestinations:%d", numDestinations);
  static int counter = 0;
  char filenameafter[31];
  int i;


  if(numDestinations == 0)
    return;

  for(i = 0; i < numDestinations; i++)
  {
    if(destinations[i].sigma > 2.5)
    {
      float newsigma = destinations[i].sigma / sqrt(2);

      VL_PRINTF("replacing index %d with sigma=%f by sigma=%f and %f", i, destinations[i].sigma, newsigma, newsigma);

      for(int j = numDestinations; j > i; j--)
        destinations[j] = destinations[j-1];

      destinations[i].sigma = newsigma;
      destinations[i+1].sigma = newsigma;

      numDestinations++;
      i--; // check again if its still greater than the threshold
    }
  }


  snprintf(filenameafter, sizeof(filenameafter), "tmp/input_%02d.pgm", counter);
  my_write_pgm_image_fixed(inputImage, width, height, filenameafter);

  filterImageGaussian_chained_params* params = vl_malloc(sizeof(filterImageGaussian_chained_params));

  params->inputImage = (short*)vl_dsp_get_mapped_addr(inputImage);
  //params->inputOutputImageSize = (width * height + 23 - 1)*sizeof(short);
  params->inputOutputImageSize = (width * height)*sizeof(short);
  params->width = width;
  params->height = height;

  vl_dsp_dmm_buffer_begin((void*)inputImage);

  ConvolutionKernel gaussKernel;
  ConvolutionKernel preCalcedGaussKernel = createConvolutionKernel(destinations[0].sigma, 0, 15);


  for(i = 0; i < numDestinations; i++)
  {

    VL_PRINTF("filterMultipleTimes_on_dsp: filtering image %d with sigma=%f, width:%d, height:%d", i, destinations[i].sigma, width, height);
    VL_PRINTF("inputimage:%x, outputImage:%x", inputImage, destinations[i].outputImage);

    gaussKernel = preCalcedGaussKernel;

    params->outputImage = vl_dsp_get_mapped_addr(destinations[i].outputImage);
    params->gauss = *gaussKernel;
    params->gauss.data = (short*)vl_dsp_get_mapped_addr(gaussKernel->data);

    vl_dsp_dmm_buffer_begin((void*)gaussKernel->data);
    vl_dsp_dmm_buffer_begin((void*)params);
//#undef UDSP
#ifdef UDSP

    //if(destinations[i].outputImage != inputImage)
    //  memcpy(destinations[i].outputImage, inputImage, params->inputOutputImageSize);

    vl_dsp_send_message(DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAIN, (uint32_t)vl_dsp_get_mapped_addr(params), 0);

    //while DSP calculates GAUSSIAN of image, precalculate newxt GaussKernel
    if(i < numDestinations -1)
    {
      preCalcedGaussKernel = createConvolutionKernel(destinations[i+1].sigma, 0, 15);
    }


    //wait until previous gaussian smoothing is finished
    vl_dsp_get_message();
#else
    void* input;
    if(i != 0)
      input = destinations[i-1].outputImage;
    else
      input = inputImage;

    memcpy(destinations[i].outputImage, input, width*height*sizeof(short));
    filterImageGaussian(destinations[i].outputImage, width, height, gaussKernel);
    if(i < numDestinations -1)
    {
      preCalcedGaussKernel = createConvolutionKernel(destinations[i+1].sigma, 0, 15);
    }
#endif

    params->inputImage = NULL;  //to use the last OutputImage as the next InputImage
  }

#ifdef UDSP
  vl_dsp_dmm_buffer_end((void*)destinations[0].outputImage);
#endif

  for(i = 0; i < numDestinations; i++)
  {
    counter++;

    snprintf(filenameafter, sizeof(filenameafter), "tmp/after_%02d.pgm", counter);
    my_write_pgm_image_fixed(destinations[i].outputImage, width, height, filenameafter);
  }
}

void filterImageGaussian_on_dsp(short* inputOutputImage,
    int width, int height,
    ConvolutionKernel gauss)
{
#ifdef UDSP
  VL_PRINTF("filterImageGaussian_on_dsp: using DSP....");
  filterImageGaussian_params* params = vl_malloc(sizeof(filterImageGaussian_params));

  params->inputOutputImage = (short*)vl_dsp_get_mapped_addr(inputOutputImage);
  params->inputOutputImageSize = (width * height + 23 - 1)*sizeof(short);
  params->width = width;
  params->height = height;
  params->gauss = *gauss;

  VL_PRINTF("gauss->data: %x", gauss->data);
  params->gauss.data = (short*)vl_dsp_get_mapped_addr(gauss->data);
  VL_PRINTF("gauss->data: %x", gauss->data);

  vl_dsp_dmm_buffer_begin((void*)inputOutputImage);
  vl_dsp_dmm_buffer_begin((void*)gauss->data);
  vl_dsp_dmm_buffer_begin((void*)params);


  vl_dsp_send_message(DSP_CALC_GAUSSIAN_FIXEDPOINT, (uint32_t)vl_dsp_get_mapped_addr(params), 0);

  //dsp_msg_t msg =
  vl_dsp_get_message();

  vl_dsp_dmm_buffer_end((void*)inputOutputImage);

  vl_free(params);
#else
  VL_PRINTF("filterImageGaussian_on_dsp: NOT using DSP....");
  filterImageGaussian(inputOutputImage, width, height, gauss);
#endif
}

void vl_imconvcol_vf_on_dsp(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags)
{
  debugParams(dst, dst_stride,
      src, src_width, src_height, src_stride,
      filt,
      filt_begin, filt_end,
      step, flags);

#ifdef UDSP


  imconvol_vf_params* params = vl_malloc(sizeof(imconvol_vf_params));

  params->dst = (float*)vl_dsp_get_mapped_addr((void*)dst);  //ATTENTION --> Pointer
  params->dst_stride = dst_stride;
  params->src = (float*)vl_dsp_get_mapped_addr((void*)src);  //ATTENTION --> Pointer
  params->src_width = src_width;
  params->src_height = src_height;
  params->src_stride = src_stride;
  params->filt = (float*)vl_dsp_get_mapped_addr((void*)filt);//ATTENTION --> Pointer
  params->filt_begin = filt_begin;
  params->filt_end = filt_end;
  params->step = step;
  params->flags = flags;

  //params->dst_size = src_width*src_height*sizeof(float);
  params->dst_size = src_stride*src_height*sizeof(float);
  params->src_size = params->dst_size;
  params->filt_size = (filt_end - filt_begin)*sizeof(float);

  char* dbg_str = vl_malloc(1024);
  params->dbg_str = vl_dsp_get_mapped_addr(dbg_str);
  params->dbg_str_size = 1024;


  /*for(int y = 0; y < src_height; y++)
  {
    for(int x = 0; x < src_width; x++)
    {
      dst[y*src_width + x] = 0;
    }
  }*/
  dbg_str[0] = 0;


  vl_dsp_dmm_buffer_begin((void*)dst);
  vl_dsp_dmm_buffer_begin((void*)src);
  vl_dsp_dmm_buffer_begin((void*)filt);
  vl_dsp_dmm_buffer_begin((void*)params);


  vl_dsp_send_message(DSP_CALC_IMCONVOL_VF, (uint32_t)vl_dsp_get_mapped_addr(params), 0);

  //dsp_msg_t msg =
  vl_dsp_get_message();

  vl_dsp_dmm_buffer_end((void*)dst);
  vl_dsp_dmm_buffer_end((void*)src);
  vl_dsp_dmm_buffer_end((void*)filt);
  vl_dsp_dmm_buffer_end((void*)dbg_str);

  VL_PRINT("DSPDBG: %s", dbg_str);

  vl_free(params);
#else
  vl_imconvcol_vf (dst, dst_stride,
        src, src_width, src_height, src_stride,
        filt,
        filt_begin, filt_end,
        step, flags) ;
#endif


  debugParams(dst, dst_stride,
      src, src_width, src_height, src_stride,
      filt,
      filt_begin, filt_end,
      step, flags);
}

void debugParams(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags)
{
  static int counter = 0;
  int x,y;

  counter++;

  char buf[501] = {0};

  VL_PRINT("------------DEBUGPARAMS(%d)----------", counter);
  VL_PRINT("src:%x  ", (unsigned)src);
  VL_PRINT("%3.3f ", src[0]);


  for(y = 0; y < src_height; y++)
  {
    buf[0] = 0;
    for(x = 0; x < src_width; x++)
    {
      int size = strnlen(buf, sizeof(buf));
      snprintf(buf + size, sizeof(buf) - size, "%3.3f ", src[y*src_width + x]);
    }
    VL_PRINT(buf);
  }

  VL_PRINT("dst:%x  ", (unsigned)dst);


  for(y = 0; y < src_height; y++)
  {
    buf[0] = 0;
    for(x = 0; x < src_width; x++)
    {
      int size = strnlen(buf, sizeof(buf));
      snprintf(buf + size, sizeof(buf) - size, "%3.3f ", dst[y*src_width + x]);
    }
    VL_PRINT(buf);
  }

  VL_PRINT("filt:%x  ", (unsigned)filt);


  buf[0] = 0;
  for(x = 0; x < (filt_end - filt_begin); x++)
  {
    int size = strnlen(buf, sizeof(buf));
    snprintf(buf + size, sizeof(buf) - size, "%3.3f ", filt[x]);
  }
  VL_PRINT(buf);

  VL_PRINT("------------END OF DEBUGPARAMS----------");

}


