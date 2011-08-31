/** @internal
 ** @file     sift.c
 ** @author   Andrea Vedaldi
 ** @brief    Scale Invariant Feature Transform (SIFT) - Driver
 **/

/* AUTORIGHTS
Copyright (C) 2007-09 Andrea Vedaldi and Brian Fulkerson

This file is part of VLFeat, available in the terms of the GNU
General Public License version 2.
*/

#define VL_SIFT_DRIVER_VERSION 0.1

#include "generic-driver.h"

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

#include <vl/generic.h>
#include <vl/stringop.h>
#include <vl/pgm.h>
#include <vl/sift.h>
#include <vl/getopt_long.h>

#ifdef __cplusplus /* If this is a C++ compiler, end C linkage */
}
#endif

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "sift.h"
#include "logger.h"
#include "SystemTimeMeasure.h"
#include "Dsp.h"

Sift::Sift()
{
  data = 0;
  fdata = 0;

  //init some vlfeat stuff
  Logger::debug(Logger::SIFT, "Setting alloc functions");
  vl_set_alloc_func(dsp_malloc, dsp_realloc, dsp_calloc, dsp_free);
  vl_set_dsp_mem_func(dsp_get_mapped_addr, dsp_dmm_buffer_begin, dsp_dmm_buffer_end);
}

void Sift::ReadImageFromFile(char* filename)
{
  char basename [1024];
  FILE *in = 0;
  vl_size q;

  vl_bool  err    = VL_ERR_OK ;
  /* ...............................................................
   *                                                 Determine files
   * ............................................................ */

  Logger::info(Logger::SIFT, "ReadImageFromFile(%s)", filename);

  /* get basename from filename */
  q = vl_string_basename (basename, sizeof(basename), filename, 1) ;

  err = (q >= sizeof(basename)) ;

  if (err)
  {
    Logger::error(Logger::SIFT, "ReadImageFromFile: Basename of '%s' is too long", filename);
    throw SiftException("Basename of file is too long!");
  }


  /* open input file */
  in = fopen (filename, "rb") ;
  if (!in)
  {
    Logger::error(Logger::SIFT, "ReadImageFromFile: Failed to open file: '%s' with flags 'rb'", filename);
    throw SiftException("failed to open file for reading");
  }

  /* ...............................................................
   *                                                       Read data
   * ............................................................ */

  /* read PGM header */
  err = vl_pgm_extract_head (in, &pim) ;

  if (err) {
    //TODO: ERROR HANDLING
    switch (vl_get_last_error()) {
    case  VL_ERR_PGM_IO :
      Logger::error(Logger::SIFT, "ReadImageFromFile: Cannot read from '%s'.", filename);
      throw SiftException("Cannot read from file");
      break ;

    case VL_ERR_PGM_INV_HEAD :
      Logger::error(Logger::SIFT, "ReadImageFromFile: '%s' contains a malformed PGM header.", filename);
      throw SiftException("file contains a malformed PGM header.");
      break;
    }
  }

  /*if (verbose)
    printf ("sift: image is %d by %d pixels\n",
            pim. width,
            pim. height) ;*/

  /* allocate buffer */
  data  = (vl_uint8*)vl_malloc(vl_pgm_get_npixels (&pim) *
                 vl_pgm_get_bpp(&pim) * sizeof (vl_uint8)   ) ;
  fdata = (vl_sift_pix*)vl_malloc(vl_pgm_get_npixels (&pim) *
                 vl_pgm_get_bpp       (&pim) * sizeof (vl_sift_pix)) ;

  if (!data || !fdata)
  {
    Logger::error(Logger::SIFT, "ReadImageFromFile: out of mem while allocating buffers for image.");
    throw SiftException("out of mem while allocating buffers for image.");
  }

  /* read PGM body */
  err  = vl_pgm_extract_data (in, &pim, data) ;

  if (err) {
    Logger::error(Logger::SIFT, "ReadImageFromFile: '%s' contains a malformed PGM body.", filename);
    throw SiftException("file contains a malformed PGM body.");
    err = VL_ERR_IO ;
  }

  /* convert data type */
  for (q = 0 ; q < (unsigned) (pim.width * pim.height) ; ++q) {
    fdata [q] = data [q] ;
  }

  /* close files */
  if (in) {
    fclose (in) ;
    in = 0 ;
  }
}


/* ---------------------------------------------------------------- */
/** @brief SIFT driver entry point
 **/
int Sift::Detect()
{
  /* algorithm parameters */
  double   edge_thresh  = -1 ;
  double   peak_thresh  = -1 ;
  double   magnif       = -1 ;
  int      O = -1, S = 3, omin = -1 ;

  vl_bool  err    = VL_ERR_OK ;
  vl_bool  force_orientations = 0 ;


  VlSiftFilt      *filt = 0 ;
  vl_size          q ;
  int              i ;
  vl_bool          first ;

  TimeMeasureBase& measure = *TimeMeasureBase::getInstance();

  measure.startTimer("Sift::Detect()");

#define WERR(name,op)                                           \
  if (err == VL_ERR_OVERFLOW) {                               \
    snprintf(err_msg, sizeof(err_msg),                        \
             "Output file name too long.") ;                  \
    goto done ;                                               \
  } else if (err) {                                           \
    snprintf(err_msg, sizeof(err_msg),                        \
             "Could not open '%s' for " #op, name) ;          \
    goto done ;                                               \
  }


  /* ...............................................................
   *                                                     Make filter
   * ............................................................ */

  filt = vl_sift_new (pim.width, pim.height, O, S, omin) ;

  if (edge_thresh >= 0) vl_sift_set_edge_thresh (filt, edge_thresh) ;
  if (peak_thresh >= 0) vl_sift_set_peak_thresh (filt, peak_thresh) ;
  if (magnif      >= 0) vl_sift_set_magnif      (filt, magnif) ;

  if (!filt)
  {
    Logger::error(Logger::SIFT, "Detect: could not create SIFT-fiter.");
    throw SiftException("could not create SIFT-fiter.");
  }

  Logger::debug(Logger::SIFT, "sift: filter settings:") ;
  Logger::debug(Logger::SIFT, "sift:   octaves      (O)     = %d",
          vl_sift_get_noctaves     (filt)) ;
  Logger::debug(Logger::SIFT, "sift:   levels       (S)     = %d",
          vl_sift_get_nlevels      (filt)) ;
  Logger::debug(Logger::SIFT, "sift:   first octave (o_min) = %d",
          vl_sift_get_octave_first (filt)) ;
  Logger::debug(Logger::SIFT, "sift:   edge thresh           = %g",
          vl_sift_get_edge_thresh  (filt)) ;
  Logger::debug(Logger::SIFT, "sift:   peak thresh           = %g",
          vl_sift_get_peak_thresh  (filt)) ;
  Logger::debug(Logger::SIFT, "sift:   magnif                = %g",
          vl_sift_get_magnif       (filt)) ;
  Logger::debug(Logger::SIFT, "sift: will force orientations? %s",
          force_orientations ? "yes" : "no") ;

  /* ...............................................................
   *                                             Process each octave
   * ............................................................ */
  i     = 0 ;
  first = 1 ;
  while (1)
  {
    VlSiftKeypoint const *keys = 0 ;
    int                   nkeys ;

    Logger::debug(Logger::SIFT, "sift: computing octave");

    /* calculate the GSS for the next octave .................... */
    measure.startTimer("process_octave");
    if (first)
    {
      measure.startTimer("process_f_octave");
      first = 0 ;
      err = vl_sift_process_first_octave(filt, fdata) ;
      measure.stopTimer("process_f_octave");
    }
    else
    {
      measure.startTimer("process_n_octave");
      err = vl_sift_process_next_octave(filt);
      measure.stopTimer("process_n_octave");
    }
    measure.stopTimer("process_octave");

    if (err)
    {
      err = VL_ERR_OK ;
      break ;
    }

    Logger::debug(Logger::SIFT, "sift: GSS octave %d computed",
             vl_sift_get_octave_index (filt));


    /* run detector ............................................. */
    measure.startTimer("sift_detect");
    vl_sift_detect (filt) ;
    measure.stopTimer("sift_detect");

    keys  = vl_sift_get_keypoints(filt) ;
    nkeys = vl_sift_get_nkeypoints(filt) ;
    i     = 0 ;

    Logger::debug(Logger::SIFT, "sift: detected %d (unoriented) keypoints", nkeys) ;



    /* for each keypoint ........................................ */
    for (; i < nkeys ; ++i)
    {
      measure.startTimer("per_kpoint");
      double                angles [4] ;
      int                   nangles ;
      //VlSiftKeypoint        ik ;
      VlSiftKeypoint const *k ;

      /* obtain keypoint orientations ........................... */
      k = keys + i ;
      nangles = vl_sift_calc_keypoint_orientations
        (filt, angles, k) ;

      /* for each orientation ................................... */
      for (q = 0 ; q < (unsigned) nangles ; ++q)
      {
        KeyPointDescriptor newKeyPoint;

        /* compute descriptor (if necessary) */
        //if (out.active || dsc.active) {
        vl_sift_calc_keypoint_descriptor(filt, newKeyPoint.descr, k, angles [q]) ;
        //}

        newKeyPoint.keypoint = *k;
        newKeyPoint.angle = angles[q];

        detected_keypoints.push_back(newKeyPoint);
      }

      measure.stopTimer("per_kpoint");
    }
  }

  /* ...............................................................
   *                                                       Finish up
   * ............................................................ */


  /* release filter */
  if (filt)
  {
    vl_sift_delete (filt) ;
    filt = 0 ;
  }


  measure.stopTimer("Sift::Detect()");

  measure.printStatistic();

  /* quit */
  return 0;
}
