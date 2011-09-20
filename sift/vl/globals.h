#ifndef _GLOBALS_INCLUDED_
#define _GLOBALS_INCLUDED_

#include "generic.h"


#define DEBUG

// #define _WRITE_IMAGES_ 					// for writing images
//#define _DRAW_CIRCLE_						// drawing a circle


// WARNING "EXACT MIGHT CAUSE PROBLEMS IN RECOGNITION DEMO!!! 
// USE FLOAT
// #define _USE_EXACT_							// means using float instead of short in calculation of PCA SIFT
// #define  _USE_EUCLIDEANAPPROX_					// use acos approximation to euclidean distance (Lowe style)

// PCA STUFF
#define PCALEN 36
#define EPCALEN 36 //< Effective number of pca components (calculation and file output).
#define PatchSize 41
#define PatchLength (PatchSize * PatchSize)
#define GPLEN ((PatchSize - 2) * (PatchSize - 2) * 2)

#define GRAYLEVELS 					256

// define min and max
#define MAX(x,y) (x) > (y) ? (x) : (y)
#define MIN(x,y) (x) < (y) ? (x) : (y) 

#define MAX_KERNEL_WIDTH_DEF 	23 			// default maximum width of convolution kernel
//#define MAX_NUM_KEYPOINTS 		2000		// maximum number of keypoints to detect (preferring small ones!!!)
/*
// Threshold settings for DOG detection
// and Orientation Assignment
#define REGIONFACTOR			6			// radius around point for orientation calculation = REGIONFACTOR*sigma
#define MULTI_ORIENT_FACTOR 	0.75		// factor for multiple orientation assignment
#define CONTRAST_THR			600 		// = 0.01 ( Suri's 0.03	= 980 has to be converted to short, 
											// but depends on octave scaling! 
#define CURVATURE_THR			8			// 12
#define NUM_OCTAVES 			3 			// 3 is enough - 4 is maximum!!!!!!!!!!!!!!
#define INTERVAL_S				4			// means 7 images
#define PREFILTER_SIGMA			0.8f		// prefiltering 0.8
#define START_SIGMA				1.0f		// starting sigma
#define POW_FACTOR				2			// power factor for octave scaling
*/
// we changed some of the default values:
// Setting 1:
// MAX_KERNEL_WIDTH_DEF = 17, ContrThr = 800 / Curve = 8; Prefilter = 0.8; startSigma = 1.0f; 
// POW_FACTOR = 2 ---> getSigma function now using 2^x!
// Setting 2:
// MAX_KERNEL_WIDTH_DEF = 27, ContrThr = 1120 / Curve = 12; Prefilter = 0.8; startSigma = 0.8f; 
// POW_FACTOR = 3 ---> getSigma function now using 3^x!

// Settings for Patch Rotator and Descriptor
/*
#define ANGLERES					360		// 360� range for rotating an image patch is 
											// divided into ANGLERES bins (= 360 means 1� rotation resolution)
#define SCALERANGE					15		// range of scaling has length 2^SCALERANGE (for 8 => 256 possible scales)
											// ranging from 1/2^SCALERESOL ... 2^SCALERANGE/2^SCALERESOL (for 8,6 =>
											// 1/64 ... 256/64);
#define SCALERESOL					6		// resolution of scaling is 1/2^SCALERESOL;
*/
//#define PATCHSIDE					34		// means, that the descriptor is calculated on a window of size 
//#define DESC_SCALE_FACT_POW_OF_TWO	19		// 50:20 or 34:19 -> otherwise there are overflow errors!
											// (PATCHSIDE-2)x(PATCHSIDE-2);
											// WARNING: ALGORITHM IS DESIGNED TO NOT OVERFLOW ON 50x50 PATCHES! 
											// DO NOT CHANGE SIZE TO HIGHER VALUES -> CORRECT WORK IS ONLY 
											// GUARANTEED FOR SMALLER VALUES (MINIMUM = 34 (32x32 patch)
											// because of scaling stuff, the distance calculations in matlab
											// script might have to be changed (not they have to be kept 
											// at 0.6 for realistic results); 
											// CHANGE TO 42:20 for PCASIFT!!!!
// Mathematical Stuff
#define APPROX_SQRT					15		// approximation steps for sqrt 
											// number of iterations has a big impact on 
											// performance (15 = 1.7s, 2 = 0.7s)
											// getting rid of it would be the best!!!

// SOME OTHER FLAGS; MOSTLY FOR PERFORMANCE GAINS
// QUANTIZATION WORKS IN MATLAB-MEX MODE ONLY!!!
//#define _QUANTIZE_DESCS_					// quantization of the PCA DESCRIPTORS! USE WITH PCA ONLY!!!!!!!!!
//#define NUM_QUANT_BYTES				1		// 1 = char (256), 2 = short (65536), 4 = int (2^32)
// SHORT DEFINITION TO BE SET IN pcavectors.h AND in PCASiftMaker.h!!!
//#define	_USE_SHORT_							// use short-quantized descriptors
//
//#define	_CALC_SIFT_FEATURES_				// calculate sift features as short vectors -> or calc pca-sift features	
//#define	DESC_SIZE					128		// size of the descriptors (128 = sift, 36 = pca-sift)
//#define _USE_OLD_DESC_CALCULATION_			// use old version using atan2 orientation calculation 
											// (instead of gradient filter responses -> we have to change the
											// matching distance to a higher value to get comparable results to
											// the old matching method/ratio (0.6 -> 0.75)
//#define _USE_ORIENTATION_INTERPOLATION_		// use parabolic fitting to find orientation
//#define _USE_BILINEAR_INTERPOLATION_		// bilinear interpolation for patch rotation
// #define _SKIP_SQRT_							// skipping normalization using approxSqrt function using
											// simple squares and shifts // DO NOT USE TOGETHER WITH OLD_DESC_CALC


//---------------------------------------------------------------------------
// Descriptor Matching
// (currently works for SIFT / !USE_OLD only!!!!!
//---------------------------------------------------------------------------
#define MATCH_DIST_FACT				0.6		// distance factor for matching

#ifndef _USE_OLD_DESC_CALCULATION_				// IF WE USE APPROXIMATED DESCRIPTORS, WE HAVE TO USE ANOTHER
								// NORMALIZATION FOR MATCHING!!!
#define DIVISION_FACTOR				15		// factor 2^DIVFACT for normalizing the descriptors  
#endif


#ifdef _WRITE_IMAGES_
#include "pnmio.h"
#else
// REMOVE pnmio.c from dependency and link list!!!!!
#endif

#ifdef _ON_DSP_
//#include "..\dsp_project_UPLOAD\COM.h"
//#include "..\dsp_project_UPLOAD\icam2.h"

#ifndef M_PI
#define M_PI   3.14159265358979323846
#endif
#elif WINCE // ARM
#ifndef M_PI
#define M_PI   3.14159265358979323846
#endif

#define sdheap_memalign(a,b) malloc(b)
#define sdheap_malloc malloc
#define sdheap_free free
#define COM_PutStr printf
#include "ti_func_4_pc.h"

#else

#define COM_PutStr VL_PRINTF
#define sdheap_memalign memalign
#define sdheap_malloc malloc
#define sdheap_free free
#include "ti_func_4_pc.h"

#endif

#ifndef WIN32
#define FILESEP "/"
#define _nassert assert
#else
#define FILESEP "\\"
#endif // WIN32

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#endif
