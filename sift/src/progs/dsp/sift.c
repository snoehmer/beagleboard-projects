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
#include "../../lib/common/node.h"
//#include "../../../vl/sift.h"
//#include "../../../vl/imopv.h"
#include "../../../vl/sift_dsp.h"


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
	

	while (!done) {
		NODE_getMsg(env, &msg, (unsigned) -1);

		switch (msg.cmd) {
		case 1:
			{
			  imconvol_vf_params * params = (imconvol_vf_params*) msg.arg_1;


			  BCACHE_inv((void*) params, sizeof(params), 1);
			  BCACHE_inv((void*) params->src, params->src_size, 1);
			  BCACHE_inv((void*) params->dst, params->dst_size, 1);
			  BCACHE_inv((void*) params->filt, params->filt_size, 1);

			  vl_imconvcol_vf (params->dst, params->dst_stride,
			      params->src, params->src_width, params->src_height, params->src_stride,
			      params->filt,
			      params->filt_begin, params->filt_end,
			      params->step, params->flags);


			  BCACHE_wbInv((void*) params, sizeof(params), 1);
			  BCACHE_wbInv((void*) params->src, params->src_size, 1);
			  BCACHE_wbInv((void*) params->dst, params->dst_size, 1);
			  BCACHE_wbInv((void*) params->filt, params->filt_size, 1);


        msg.cmd = 2;

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
