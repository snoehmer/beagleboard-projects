/*
 * sift_dsp.c
 *
 *  Created on: 03.09.2011
 *      Author: tom
 */

#include "sift.h"
#include "sift_dsp.h"
#include "imopv.h"

void vl_imconvcol_vf_on_dsp(float* dst, int dst_stride,
    float const* src,
    int src_width, int src_height, int src_stride,
    float const* filt, int filt_begin, int filt_end,
    int step, unsigned int flags)
{
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

  params->dst_size = src_width*src_height*sizeof(float);
  params->src_size = params->dst_size;
  params->filt_size = (filt_end - filt_begin)*sizeof(float);

  vl_dsp_dmm_buffer_begin((void*)dst);
  vl_dsp_dmm_buffer_begin((void*)src);
  vl_dsp_dmm_buffer_begin((void*)filt);
  vl_dsp_dmm_buffer_begin((void*)params);

  vl_dsp_send_message(1, (uint32_t)vl_dsp_get_mapped_addr(params), 0);

  /*vl_imconvcol_vf (dst, dst_stride,
      src, src_width, src_height, src_stride,
      filt,
      filt_begin, filt_end,
      step, flags) ;*/

  //dsp_msg_t msg =
  vl_dsp_get_message();

  vl_dsp_dmm_buffer_end((void*)dst);
  vl_dsp_dmm_buffer_end((void*)src);
  vl_dsp_dmm_buffer_end((void*)filt);

  vl_free(params);
}
