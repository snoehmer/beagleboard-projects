/*
 * Copyright (C) 2008-2009 Nokia Corporation
 * Copyright (C) 2009 Igalia S.L
 *
 * Author: Felipe Contreras <felipe.contreras@nokia.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <stddef.h>
#include <stdio.h>
#include <c6x.h>
#include <stdlib.h>
#include <string.h>
#include <dsplib.h>


typedef struct  {
  int width;
  float sigma;
  short * data;
}  ConvolutionKernelRef, *ConvolutionKernel;


#include "../../lib/common/node.h"
//#include "../../../vl/sift.h"
//#include "../../../vl/imopv.h"
#include "../../../vl/sift_dsp.h"


int filterImageGaussian(
  short* inputOutputImage,
  int width, int height,
  ConvolutionKernel gauss);

unsigned int dsp_sift_create(void)
{
  return 0x8000;
}

unsigned int dsp_sift_delete(void)
{
	return 0x8000;
}



unsigned int dsp_sift_execute(void *env)
{
	dsp_msg_t msg;
	unsigned char done = 0;
	short* gaussChain_inputImage = NULL;



	while (!done) {
		NODE_getMsg(env, &msg, (unsigned) -1);

		switch (msg.cmd) {
		case DSP_CALC_IMCONVOL_VF:
			{
			  printf("SIFT: convolving :)");
			  imconvol_vf_params * params = (imconvol_vf_params*) msg.arg_1;


			  BCACHE_inv((void*) params, sizeof(params), 1);
			  BCACHE_inv((void*) params->src, params->src_size, 1);
			  BCACHE_inv((void*) params->dst, params->dst_size, 1);
			  BCACHE_inv((void*) params->filt, params->filt_size, 1);

			  //params->dst[0] = 1;
			  vl_imconvcol_vf (params->dst, params->dst_stride,
			      params->src, params->src_width, params->src_height, params->src_stride,
			      params->filt,
			      params->filt_begin, params->filt_end,
			      params->step, params->flags);
			  //params->dst[0] = 2;

			  memcpy(params->dbg_str, "abc", 4);

			  BCACHE_wbInv((void*) params, sizeof(params), 1);
			  BCACHE_wbInv((void*) params->src, params->src_size, 1);
			  BCACHE_wbInv((void*) params->dst, params->dst_size, 1);
			  BCACHE_wbInv((void*) params->filt, params->filt_size, 1);
			  BCACHE_wbInv((void*) params->dbg_str, params->dbg_str_size, 1);



        msg.cmd = DSP_CALC_IMCONVOL_VF_FINISHED;

				NODE_putMsg(env, NULL, &msg, 0);
				break;
			}

		case DSP_CALC_GAUSSIAN_FIXEDPOINT:
		  {
		    filterImageGaussian_params * params = (filterImageGaussian_params*) msg.arg_1;
		    int result;

        BCACHE_inv((void*) params, sizeof(filterImageGaussian_params), 1);
        BCACHE_inv((void*) params->inputOutputImage, params->inputOutputImageSize, 1);
        BCACHE_inv((void*) params->gauss.data, params->gauss.width * sizeof(short), 1);


        result = filterImageGaussian(params->inputOutputImage, params->width, params->height, &params->gauss);


        BCACHE_wbInv((void*) params, sizeof(filterImageGaussian_params), 1);
        BCACHE_wbInv((void*) params->inputOutputImage, params->inputOutputImageSize, 1);


        if(result == 0)
          msg.cmd = DSP_CALC_IMCONVOL_VF_FINISHED;
        else
          msg.cmd = DSP_CALC_IMCONVOL_VF_FAILED;

        msg.arg_2 = (uint32_t)params->width;

        NODE_putMsg(env, NULL, &msg, 0);

		    break;
		  }


    case DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAIN:
      {
        filterImageGaussian_chained_params * params = (filterImageGaussian_chained_params*) msg.arg_1;
        int result;

        BCACHE_inv((void*) params, sizeof(filterImageGaussian_chained_params), 1);

        if(params->inputImage)
        {
          BCACHE_inv((void*) params->inputImage, params->inputOutputImageSize, 1);
          gaussChain_inputImage = params->inputImage;
        }

        BCACHE_inv((void*) params->gauss.data, params->gauss.width * sizeof(short), 1);
        BCACHE_inv((void*) params->outputImage, params->inputOutputImageSize, 1);

        if(gaussChain_inputImage != params->outputImage)
        {
          /*if(params->inputOutputImageSize%8 != 0)
          {
            msg.arg_2 = 123;
            */
            memcpy(params->outputImage, gaussChain_inputImage, params->inputOutputImageSize);
          /*}
          else
          {
            msg.arg_2 = 456;
            DSP_blk_move(gaussChain_inputImage, params->outputImage, params->inputOutputImageSize);
          }*/
        }

        gaussChain_inputImage = params->outputImage;

        result = filterImageGaussian(params->outputImage, params->width, params->height, &params->gauss);


        BCACHE_wbInv((void*) params, sizeof(filterImageGaussian_chained_params), 1);
        BCACHE_wbInv((void*) params->outputImage, params->inputOutputImageSize, 1);


        if(result == 0)
          msg.cmd = DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAINED_FINISHED;
        else
          msg.cmd = DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAINED_FAILED;


        NODE_putMsg(env, NULL, &msg, 0);

        break;
      }

		case 0x80000000:
			done = 1;
			break;
		}
	}

	return 0x8000;
}

void vl_imconvcol_vf(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags)
{
#define T float
  int x = 0 ;
  int y ;
  int dheight = (src_height - 1) / step + 1 ;
  vl_bool transp = flags & VL_TRANSPOSE ;
  vl_bool zeropad = (flags & VL_PAD_MASK) == VL_PAD_BY_ZERO ;

  // let filt point to the last sample of the filter
  filt += filt_end - filt_begin ;

  while (x < src_width) {
    // Calculate dest[x,y] = sum_p image[x,p] filt[y - p]
    // where supp(filt) = [filt_begin, filt_end] = [fb,fe].
    //
    //  CHUNK_A: y - fe <= p < 0
    //          completes VL_MAX(fe - y, 0) samples
    // CHUNK_B: VL_MAX(y - fe, 0) <= p < VL_MIN(y - fb, height - 1)
    //          completes fe - VL_MAX(fb, height - y) + 1 samples
    // CHUNK_C: completes all samples
    //
    T const *filti ;
    int stop ;

    for (y = 0 ; y < src_height ; y += step) {
      T acc = 0 ;
      T v = 0, c ;
      T const* srci ;



      filti = filt ;
      stop = filt_end - y ;
      srci = src + x - stop * src_stride ;

      if (stop > 0) {
        if (zeropad) {
          v = 0 ;
        } else {
          v = *(src + x) ;
        }
        while (filti > filt - stop) {
          c = *filti-- ;
          acc += v * c ;
          srci += src_stride ;
        }
      }

      stop = filt_end - VL_MAX(filt_begin, y - src_height + 1) + 1 ;
      while (filti > filt - stop) {
        v = *srci ;
        c = *filti-- ;
        acc += v * c ;
        srci += src_stride ;
      }


      if (zeropad) v = 0 ;

      stop = filt_end - filt_begin + 1 ;
      while (filti > filt - stop) {
        c = *filti-- ;
        acc += v * c ;
      }

      if (transp) {
        *dst = acc ; dst += 1 ;
      } else {
        *dst = acc ; dst += dst_stride ;
      }

    } // next y
    if (transp) {
      dst += 1 * dst_stride - dheight * 1 ;
    } else {
      dst += 1 * 1 - dheight * dst_stride ;
    }

    x += 1 ;
  } //next x

}

// img Transpose
void DSP_mat_trans_slow(short *x, short rows, short columns, short *r)
{
  short i,j;
    for(i=0; i<columns; i++)
      for(j=0; j<rows; j++)
        *(r+i*rows+j)=*(x+i+columns*j);
}

int filterImageGaussian(
  short* inputOutputImage,
  int width, int height,
  ConvolutionKernel gauss)
{
  short * tmpSpace;
  int radius;

  int kernelLen = gauss->width;
  radius = kernelLen >> 1;


  // alloc temporary space
#ifdef ARCH_DSP
  tmpSpace = (short*) memalign(8,(8 + width*height + kernelLen - 1)*sizeof(short));
#else //_ON_DSP_
#ifdef WIN32
  tmpSpace = (short*) vl_malloc((8 + width*height + kernelLen - 1)*sizeof(short));
#else // WIN32
  posix_memalign((void**)&tmpSpace,8,(8 + width*height + kernelLen - 1)*sizeof(short));
#endif // WIN32
#endif //_ON_DSP_

  if(!tmpSpace)
    return -1;


  memset(tmpSpace,0x00,(8 + width*height + kernelLen - 1)*sizeof(short));
  //inputOutputImage[0] = 0;

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
  if(width%4 == 0 && height%4 == 0)
    DSP_mat_trans(tmpSpace + (8-radius), width, height, inputOutputImage);
  else
    DSP_mat_trans_slow(tmpSpace + (8-radius), width, height, inputOutputImage);


  // vl_free temporary space
  free(tmpSpace);

  return 0;
}

