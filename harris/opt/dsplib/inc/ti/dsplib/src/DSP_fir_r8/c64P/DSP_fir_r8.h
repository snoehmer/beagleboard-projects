/* ======================================================================= */
/* DSP_fir_r8.h -- FIR Filter (Radix 8)                                    */
/*                 Optimized C Implementation (w/ Intrinsics)              */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/  */ 
/*                                                                         */
/*                                                                         */
/*  Redistribution and use in source and binary forms, with or without     */
/*  modification, are permitted provided that the following conditions     */
/*  are met:                                                               */
/*                                                                         */
/*    Redistributions of source code must retain the above copyright       */
/*    notice, this list of conditions and the following disclaimer.        */
/*                                                                         */
/*    Redistributions in binary form must reproduce the above copyright    */
/*    notice, this list of conditions and the following disclaimer in the  */
/*    documentation and/or other materials provided with the               */
/*    distribution.                                                        */
/*                                                                         */
/*    Neither the name of Texas Instruments Incorporated nor the names of  */
/*    its contributors may be used to endorse or promote products derived  */
/*    from this software without specific prior written permission.        */
/*                                                                         */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
/*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
/*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
/*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   */
/*                                                                         */
/* ======================================================================= */

#ifndef _DSP_FIR_R8_H_
#define _DSP_FIR_R8_H_ 1

/** @ingroup FILTCONV */
/* @{ */

/** @defgroup  DSP_fir_r8 */
/** @ingroup DSP_fir_r8 */
/* @{ */

/**
 *     Computes a real FIR filter (direct-form) using coefficients    
 *     stored in vector h.  The real data input is stored in vector x.    
 *     The filter output result is stored in vector r.  Input data and   
 *     filter taps are 16-bit, with intermediate values kept at 32-bit  
 *     precision.  Filter taps are expected in Q15 format.                                                    
 *                                                                         
 *      @param  x   =  Input array [nr+nh-1 elements]
 *      @param  h   =  Coeff array [nh elements]      
 *      @param  r   =  Output array [nr elements] 
 *      @param  nh  =  Number of coefficients
 *      @param  nr  =  Number of output samples       
 *
 * @par Algorithm:
 * DSP_fir_r8_cn.c is the natural C equivalent of the optimized intrinsic C 
 * code without restrictions. Note that the intrinsic C code is optimized
 * and restrictions may apply.  
 *
 *  Performance Note: For better cycle performance when using Radix 8
 *                    coefficients nh <= 24 please use
 *                    DSP_fir_r8_h8, DSP_fir_r8_h16, DSP_fir_r8_h24 module         
 *                                                 
 * @par Assumptions:
 *     Arrays x, h, and r do not overlap. <BR>                    
 *     nr >= 4; nr % 4 == 0. <BR>               
 *     nh >= 8; nh % 8 == 0. <BR> 
 *
 * @par Implementation Notes:
 * @b Endian Support: The code supports both big and little endian modes. 
 * @b Interruptibility: The code is interruptible
 *
 */

void DSP_fir_r8 (
    const short *restrict x,    /* Input array [nr+nh-1 elements] */
    const short *restrict h,    /* Coeff array [nh elements]      */
    short       *restrict r,    /* Output array [nr elements]     */
    int nh,                     /* Number of coefficients         */
    int nr                      /* Number of output samples       */
);

#endif /* _DSP_FIR_R8_H_ */

/* ======================================================================= */
/*  End of file:  DSP_fir_r8.h                                             */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2011 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

