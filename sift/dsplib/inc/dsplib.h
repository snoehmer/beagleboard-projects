/* ========================================================================== */
/**
*  @file   dsplib.h
*
*  path    dsplib/inc
*
*  @brief  dsplib API File
*
*  ============================================================================
*  Copyright (c) Texas Instruments Incorporated 2011
*
*  ============================================================================
*/

#include <ti/dsplib/src/DSP_autocor/DSP_autocor.h>
#include <ti/dsplib/src/DSP_bexp/DSP_bexp.h>
#include <ti/dsplib/src/DSP_blk_eswap16/DSP_blk_eswap16.h>
#include <ti/dsplib/src/DSP_blk_eswap32/DSP_blk_eswap32.h>
#include <ti/dsplib/src/DSP_blk_eswap64/DSP_blk_eswap64.h>
#include <ti/dsplib/src/DSP_blk_move/DSP_blk_move.h>
#include <ti/dsplib/src/DSP_dotp_sqr/DSP_dotp_sqr.h>
#include <ti/dsplib/src/DSP_dotprod/DSP_dotprod.h>
#include <ti/dsplib/src/DSP_fft16x16/DSP_fft16x16.h>
#include <ti/dsplib/src/DSP_fft16x16_imre/DSP_fft16x16_imre.h>
#include <ti/dsplib/src/DSP_fft16x16r/DSP_fft16x16r.h>
#include <ti/dsplib/src/DSP_fft16x32/DSP_fft16x32.h>
#include <ti/dsplib/src/DSP_fft32x32/DSP_fft32x32.h>
#include <ti/dsplib/src/DSP_fft32x32s/DSP_fft32x32s.h>
#include <ti/dsplib/src/DSP_fir_cplx/DSP_fir_cplx.h>
#include <ti/dsplib/src/DSP_fir_cplx_hM4X4/DSP_fir_cplx_hM4X4.h>
#include <ti/dsplib/src/DSP_fir_gen/DSP_fir_gen.h>
#include <ti/dsplib/src/DSP_fir_gen_hM17_rA8X8/DSP_fir_gen_hM17_rA8X8.h>
#include <ti/dsplib/src/DSP_fir_r4/DSP_fir_r4.h>
#include <ti/dsplib/src/DSP_fir_r8/DSP_fir_r8.h>
#include <ti/dsplib/src/DSP_fir_r8_hM16_rM8A8X8/DSP_fir_r8_hM16_rM8A8X8.h>
#include <ti/dsplib/src/DSP_fir_sym/DSP_fir_sym.h>
#include <ti/dsplib/src/DSP_firlms2/DSP_firlms2.h>
#include <ti/dsplib/src/DSP_fltoq15/DSP_fltoq15.h>
#include <ti/dsplib/src/DSP_ifft16x16/DSP_ifft16x16.h>
#include <ti/dsplib/src/DSP_ifft16x16_imre/DSP_ifft16x16_imre.h>
#include <ti/dsplib/src/DSP_ifft16x32/DSP_ifft16x32.h>
#include <ti/dsplib/src/DSP_ifft32x32/DSP_ifft32x32.h>
#include <ti/dsplib/src/DSP_iir/DSP_iir.h>
#include <ti/dsplib/src/DSP_iir_lat/DSP_iir_lat.h>
#include <ti/dsplib/src/DSP_iir_ss/DSP_iir_ss.h>
#include <ti/dsplib/src/DSP_mat_mul/DSP_mat_mul.h>
#include <ti/dsplib/src/DSP_mat_trans/DSP_mat_trans.h>
#include <ti/dsplib/src/DSP_maxidx/DSP_maxidx.h>
#include <ti/dsplib/src/DSP_maxval/DSP_maxval.h>
#include <ti/dsplib/src/DSP_minerror/DSP_minerror.h>
#include <ti/dsplib/src/DSP_minval/DSP_minval.h>
#include <ti/dsplib/src/DSP_mul32/DSP_mul32.h>
#include <ti/dsplib/src/DSP_neg32/DSP_neg32.h>
#include <ti/dsplib/src/DSP_q15tofl/DSP_q15tofl.h>
#include <ti/dsplib/src/DSP_recip16/DSP_recip16.h>
#include <ti/dsplib/src/DSP_vecsumsq/DSP_vecsumsq.h>
#include <ti/dsplib/src/DSP_w_vec/DSP_w_vec.h>
#include <ti/dsplib/src/DSP_add16/DSP_add16.h>
#include <ti/dsplib/src/DSP_add32/DSP_add32.h>
#include <ti/dsplib/src/DSP_mat_mul_cplx/DSP_mat_mul_cplx.h>
#include <ti/dsplib/src/DSP_autocor/DSP_autocor.h>
#include <ti/dsplib/src/DSP_bexp/DSP_bexp.h>
#include <ti/dsplib/src/DSP_blk_eswap16/DSP_blk_eswap16.h>
#include <ti/dsplib/src/DSP_blk_eswap32/DSP_blk_eswap32.h>
#include <ti/dsplib/src/DSP_blk_eswap64/DSP_blk_eswap64.h>
#include <ti/dsplib/src/DSP_blk_move/DSP_blk_move.h>
#include <ti/dsplib/src/DSP_dotp_sqr/DSP_dotp_sqr.h>
#include <ti/dsplib/src/DSP_dotprod/DSP_dotprod.h>
#include <ti/dsplib/src/DSP_fft16x16/DSP_fft16x16.h>
#include <ti/dsplib/src/DSP_fft16x16_imre/DSP_fft16x16_imre.h>
#include <ti/dsplib/src/DSP_fft16x16r/DSP_fft16x16r.h>
#include <ti/dsplib/src/DSP_fft16x32/DSP_fft16x32.h>
#include <ti/dsplib/src/DSP_fft32x32/DSP_fft32x32.h>
#include <ti/dsplib/src/DSP_fft32x32s/DSP_fft32x32s.h>
#include <ti/dsplib/src/DSP_fir_cplx/DSP_fir_cplx.h>
#include <ti/dsplib/src/DSP_fir_cplx_hM4X4/DSP_fir_cplx_hM4X4.h>
#include <ti/dsplib/src/DSP_fir_gen/DSP_fir_gen.h>
#include <ti/dsplib/src/DSP_fir_gen_hM17_rA8X8/DSP_fir_gen_hM17_rA8X8.h>
#include <ti/dsplib/src/DSP_fir_r4/DSP_fir_r4.h>
#include <ti/dsplib/src/DSP_fir_r8/DSP_fir_r8.h>
#include <ti/dsplib/src/DSP_fir_r8_hM16_rM8A8X8/DSP_fir_r8_hM16_rM8A8X8.h>
#include <ti/dsplib/src/DSP_fir_sym/DSP_fir_sym.h>
#include <ti/dsplib/src/DSP_firlms2/DSP_firlms2.h>
#include <ti/dsplib/src/DSP_fltoq15/DSP_fltoq15.h>
#include <ti/dsplib/src/DSP_ifft16x16/DSP_ifft16x16.h>
#include <ti/dsplib/src/DSP_ifft16x16_imre/DSP_ifft16x16_imre.h>
#include <ti/dsplib/src/DSP_ifft16x32/DSP_ifft16x32.h>
#include <ti/dsplib/src/DSP_ifft32x32/DSP_ifft32x32.h>
#include <ti/dsplib/src/DSP_iir/DSP_iir.h>
#include <ti/dsplib/src/DSP_iir_lat/DSP_iir_lat.h>
#include <ti/dsplib/src/DSP_iir_ss/DSP_iir_ss.h>
#include <ti/dsplib/src/DSP_mat_mul/DSP_mat_mul.h>
#include <ti/dsplib/src/DSP_mat_trans/DSP_mat_trans.h>
#include <ti/dsplib/src/DSP_maxidx/DSP_maxidx.h>
#include <ti/dsplib/src/DSP_maxval/DSP_maxval.h>
#include <ti/dsplib/src/DSP_minerror/DSP_minerror.h>
#include <ti/dsplib/src/DSP_minval/DSP_minval.h>
#include <ti/dsplib/src/DSP_mul32/DSP_mul32.h>
#include <ti/dsplib/src/DSP_neg32/DSP_neg32.h>
#include <ti/dsplib/src/DSP_q15tofl/DSP_q15tofl.h>
#include <ti/dsplib/src/DSP_recip16/DSP_recip16.h>
#include <ti/dsplib/src/DSP_vecsumsq/DSP_vecsumsq.h>
#include <ti/dsplib/src/DSP_w_vec/DSP_w_vec.h>
#include <ti/dsplib/src/DSP_add16/DSP_add16.h>
#include <ti/dsplib/src/DSP_add32/DSP_add32.h>
#include <ti/dsplib/src/DSP_mat_mul_cplx/DSP_mat_mul_cplx.h>
