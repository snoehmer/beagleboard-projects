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

void calcDOG(short * restrict dst, const short* restrict a, const short * restrict b, int len);
int detectKeypoints(const short* dog_fixed, int w, int h, int nscales, short tp_fixed, VlSiftKeypoint* keys, int* nkeys, int max_nkeys);

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
	int i;
  int nkeys = 0;
  int result;


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
          memcpy(params->outputImage, gaussChain_inputImage, params->inputOutputImageSize);
        }


        //Gaussian Filtering:
        for(i = 0; i < params->filterCount; i++)
          result = filterImageGaussian(params->outputImage, params->width, params->height, &params->gauss);




        BCACHE_wbInv((void*) params, sizeof(filterImageGaussian_chained_params), 1);
        BCACHE_wbInv((void*) params->outputImage, params->inputOutputImageSize, 1);


        if(result == 0)
          msg.cmd = DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAINED_FINISHED;
        else
          msg.cmd = DSP_CALC_GAUSSIAN_FIXEDPOINT_CHAINED_FAILED;


        NODE_putMsg(env, NULL, &msg, 0);// gaussian smoothing finished
/*
        NODE_getMsg(env, &msg, (unsigned) -1); //wait for start signal of DOG

        //DOG
        if(params->dogOutImage)
        {
          calcDOG(params->dogOutImage, params->outputImage, gaussChain_inputImage, params->inputOutputImageSize/2);

          BCACHE_wbInv((void*) params->dogOutImage, params->inputOutputImageSize, 1);
        }

        NODE_putMsg(env, NULL, &msg, 0); //DOG finished ...
*/

        //remember last output (to use it as an input image the next time:
        gaussChain_inputImage = params->outputImage;
        break;
      }

    case DSP_CALC_DETECT_KEYS:
      {
        dspdetect_params * params = (dspdetect_params*) msg.arg_1;

        BCACHE_inv((void*) params, sizeof(dspdetect_params), 1);

        BCACHE_inv((void*) params->octave_smin, params->nscales*params->w*params->h*sizeof(short), 1);
        //BCACHE_inv((void*) params->dog_fixed, params->nscales*params->w*params->h*sizeof(short), 1);
        //BCACHE_inv((void*) params->keys, params->max_nkeys*sizeof(VlSiftKeypoint), 1);

        int const    so = params->w * params->h ;     // s-stride



        for(i = 0; i < params->nscales - 1; i++)
        {
          calcDOG(params->dog_fixed + i*so, params->octave_smin + (i+1)*so, params->octave_smin + i*so, so);
        }



        BCACHE_wbInv((void*) params, sizeof(dspdetect_params), 1);
        BCACHE_wbInv((void*) params->dog_fixed, (params->nscales-1)*params->w*params->h*sizeof(short), 1);


        msg.cmd = DSP_CALC_DETECT_KEYS_FINISHED;

        NODE_putMsg(env, NULL, &msg, 0); //DOG finished ...
/*
        //now detect keypoints:

        nkeys = 0;
        result = detectKeypoints(params->dog_fixed, params->w, params->h, params->nscales, params->tp_fixed, params->keys, &nkeys, params->max_nkeys);

        BCACHE_wbInv((void*) params->keys, params->max_nkeys*sizeof(VlSiftKeypoint), 1);

        if(result == 0)
          msg.arg_1 = DSP_CALC_DETECT_KEYS_FINISHED;
        else
          msg.arg_1 = DSP_CALC_DETECT_KEYS_FAILED;

        msg.arg_2 = nkeys;
        NODE_putMsg(env, NULL, &msg, 0); //detection finished ...*/
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

void DSP_fir_gen_slow
(
  short     * x,  // Input ('nr + nh - 1' samples)
  short     * h,  // Filter coefficients (nh taps)
  short       * r,  // Output array ('nr' samples)
  int          nh, // Length of filter (nh >= 5)
  int          nr  // Length of output (nr >= 1)
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
  //memset(inputOutputImage + width*height,0x00,(kernelLen - 1)*sizeof(short));




  // filter horizontally
  DSP_fir_gen(inputOutputImage,
    gauss->data,
    tmpSpace + 8,
    kernelLen,
    width*height);


  if(width%4 == 0 && height%4 == 0)
    DSP_mat_trans(tmpSpace + (8-radius), height, width, inputOutputImage);
  else
    DSP_mat_trans_slow(tmpSpace + (8-radius), height, width, inputOutputImage);


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

void calcDOG(short * restrict dst, const short* restrict a, const short * restrict b, int len)
//void calcDOG(short * dst, const short* a, const short* b, int len)
{
  int i;

  _nassert((int) dst % 4 == 0); // output is 4-byte aligned
  _nassert((int) a % 4 == 0); // input2 is 4-byte aligned
  _nassert((int) b % 4 == 0); // input1 is 4-byte aligned

  #pragma MUST_ITERATE(4,,4)
  for(i = 0; i < len; i++)
    dst[i] = a[i] - b[i];
}

int detectKeypoints(const short* dog_fixed, int w, int h, int nscales, short tp_fixed, VlSiftKeypoint* keys, int* nkeys, int max_nkeys)
{
  int s, y, x;

  int const xo    = 1 ;                         // x-stride
  int const yo    = w ;                 // y-stride
  int const    so = w * h ;     // s-stride

  const short *pt_fixed;
  short v_fixed;
  VlSiftKeypoint* k;
  int index;
  short values [32] = {0};

  *nkeys = 0;



  pt_fixed  = dog_fixed + xo + yo + so ;



#pragma MUST_ITERATE(3, 3, 3) //works only for 3 scales!!!!!!
  for(s = 0 ; s < nscales - 3 ; ++s) {
    #pragma MUST_ITERATE(2,,2)
    for(y = 1 ; y < h - 1 ; ++y) {
      #pragma MUST_ITERATE(2,,2)
      for(x = 1 ; x < w - 1 ; ++x) {
        v_fixed = *pt_fixed ;


#define CHECK_NEIGHBORS(CMP,SGN)                    \
        ( v_fixed CMP ## = SGN tp_fixed &&                \
          v_fixed CMP *(pt_fixed + xo) &&                       \
          v_fixed CMP *(pt_fixed - xo) &&                       \
          v_fixed CMP *(pt_fixed + so) &&                       \
          v_fixed CMP *(pt_fixed - so) &&                       \
          v_fixed CMP *(pt_fixed + yo) &&                       \
          v_fixed CMP *(pt_fixed - yo) &&                       \
                                                    \
          v_fixed CMP *(pt_fixed + yo + xo) &&                  \
          v_fixed CMP *(pt_fixed + yo - xo) &&                  \
          v_fixed CMP *(pt_fixed - yo + xo) &&                  \
          v_fixed CMP *(pt_fixed - yo - xo) &&                  \
                                                    \
          v_fixed CMP *(pt_fixed + xo      + so) &&             \
          v_fixed CMP *(pt_fixed - xo      + so) &&             \
          v_fixed CMP *(pt_fixed + yo      + so) &&             \
          v_fixed CMP *(pt_fixed - yo      + so) &&             \
          v_fixed CMP *(pt_fixed + yo + xo + so) &&             \
          v_fixed CMP *(pt_fixed + yo - xo + so) &&             \
          v_fixed CMP *(pt_fixed - yo + xo + so) &&             \
          v_fixed CMP *(pt_fixed - yo - xo + so) &&             \
                                                    \
          v_fixed CMP *(pt_fixed + xo      - so) &&             \
          v_fixed CMP *(pt_fixed - xo      - so) &&             \
          v_fixed CMP *(pt_fixed + yo      - so) &&             \
          v_fixed CMP *(pt_fixed - yo      - so) &&             \
          v_fixed CMP *(pt_fixed + yo + xo - so) &&             \
          v_fixed CMP *(pt_fixed + yo - xo - so) &&             \
          v_fixed CMP *(pt_fixed - yo + xo - so) &&             \
          v_fixed CMP *(pt_fixed - yo - xo - so) )


        if (CHECK_NEIGHBORS(>,+) ||
            CHECK_NEIGHBORS(<,-) ) {


          // make room for more keypoints
          if (*nkeys >= max_nkeys) {
            return -1;  //reallocation of keypoint-buffer not supported!
          }



          k = keys + ((*nkeys)++) ;

          k-> ix = x ;
          k-> iy = y ;
          k-> is = s ;

        }

        pt_fixed += 1 ;

      }
      pt_fixed += 2 ;
    }
    pt_fixed += 2 * yo ;
  }


  return 0;
}
