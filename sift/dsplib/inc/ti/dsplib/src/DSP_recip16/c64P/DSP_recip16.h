/* ======================================================================= */
/* DSP_recip16.h -- Reciprocal Function                                    */
/*                  Optimized C Implementation (w/ Intrinsics)             */
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

#ifndef _DSP_RECIP16_H_
#define _DSP_RECIP16_H_ 1

/** @ingroup MATH */
/* @{ */

/** @defgroup DSP_recip16 */
/** @ingroup DSP_recip16 */
/* @{ */

/**
 *     This program performs a reciprocal on a vector of Q15 numbers.      
 *     The result is stored in two parts: a Q15 part and an exponent       
 *     (power of two) of the fraction.                                     
 *     First, the input is loaded, then its absolute value is taken,       
 *     then it is normalized, then divided using a loop of conditional     
 *     subtracts, and finally it is negated if the original input was      
 *     negative.                                                           
 *  
 * 			@param x = Input data array of shorts
 * 			@param rfrac = Output data array containing fractions
 * 			@param rexp = Output data array containing exponents
 * 			@param nx = Number of elements in the arrays
 * 
 * @par Algorithm:
 * DSP_recip16_cn.c is the natural C equivalent of the optimized intrinsic 
 * C code without restrictions note that the intrinsic C code is optimized
 * and restrictions may apply.
 *
 * @par Assumptions: 
 *     Arrays x, rfrac, and rexp do not overlap. <BR>         
 *     nx must be a multiple of 4 and greater than or equal to 4. <BR>
 *     x must be double-word aligned. <BR>
 *
 * @par Implementation notes:
 * @b Endian Support: The code supports both big and little endian modes. <BR> 
 * @b Interruptibility: The code is interruptible. <BR>
 *  
 */

void DSP_recip16 (
    const short *restrict x,    /* Input array                       */
    short *restrict rfrac,      /* Output array containg Fractions   */
    short *restrict rexp,       /* Output array containing Exponents */
    short nx                    /* Number of elements in arrays      */
);

#endif /* _DSP_RECIP16_H_ */
/* ======================================================================= */
/*  End of file:  DSP_recip16.h                                            */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2011 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

