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

#include "../generic-driver.h"

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

void Sift::ReadImageFromFile(char* filename)
{
  char basename [1024];
  FILE *in = 0;
  vl_size q;

  vl_bool  err    = VL_ERR_OK ;
  /* ...............................................................
   *                                                 Determine files
   * ............................................................ */

  /* get basename from filename */
  q = vl_string_basename (basename, sizeof(basename), filename, 1) ;

  err = (q >= sizeof(basename)) ;

  if (err) {
    //TODO: ERROR HANDLING
    /*snprintf(err_msg, sizeof(err_msg),
             "Basename of '%s' is too long", name);*/
    err = VL_ERR_OVERFLOW ;
  }
/*
  if (verbose) {
    printf ("sift: <== '%s'\n", name) ;
  }

  if (verbose > 1) {
    printf ("sift: basename is '%s'\n", basename) ;
  }*/

  /* open input file */
  in = fopen (filename, "rb") ;
  if (!in) {
    err = VL_ERR_IO ;
    //TODO: ERROR HANDLING
    /*snprintf(err_msg, sizeof(err_msg),
             "Could not open '%s' for reading.", filename) ;*/
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
      /*snprintf(err_msg, sizeof(err_msg),
               "Cannot read from '%s'.", name) ;
      err = VL_ERR_IO ;*/
      break ;

    case VL_ERR_PGM_INV_HEAD :
      /*snprintf(err_msg, sizeof(err_msg),
               "'%s' contains a malformed PGM header.", name) ;
      err = VL_ERR_IO ;*/
      break;
    }
  }

  /*if (verbose)
    printf ("sift: image is %d by %d pixels\n",
            pim. width,
            pim. height) ;*/

  /* allocate buffer */
  data  = (vl_uint8*)malloc(vl_pgm_get_npixels (&pim) *
                 vl_pgm_get_bpp(&pim) * sizeof (vl_uint8)   ) ;
  fdata = (vl_sift_pix*)malloc(vl_pgm_get_npixels (&pim) *
                 vl_pgm_get_bpp       (&pim) * sizeof (vl_sift_pix)) ;

  if (!data || !fdata) {
    err = VL_ERR_ALLOC ;
    //TODO: ERROR HANDLING
    /*snprintf(err_msg, sizeof(err_msg),
             "Could not allocate enough memory.") ;*/
  }

  /* read PGM body */
  err  = vl_pgm_extract_data (in, &pim, data) ;

  if (err) {
    //TODO: ERROR HANDLING
    //snprintf(err_msg, sizeof(err_msg), "PGM body malformed.") ;
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
  char     err_msg [1024] ;
  int      exit_code          = 0 ;
  int      verbose            = 0 ;
  vl_bool  force_orientations = 0 ;


#define ERRF(msg, arg) {                                        \
    err = VL_ERR_BAD_ARG ;                                      \
    snprintf(err_msg, sizeof(err_msg), msg, arg) ;              \
    break ;                                                     \
  }

#define ERR(msg) {                                              \
    err = VL_ERR_BAD_ARG ;                                      \
    snprintf(err_msg, sizeof(err_msg), msg) ;                   \
    break ;                                                     \
}

    VlSiftFilt      *filt = 0 ;
    vl_size          q ;
    int              i ;
    vl_bool          first ;

    double           *ikeys = 0 ;
    int              nikeys = 0, ikeys_size = 0 ;



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

    if (!filt) {
      snprintf (err_msg, sizeof(err_msg),
                "Could not create SIFT filter.") ;
      err = VL_ERR_ALLOC ;
      goto done ;
    }

    if (verbose > 1) {
      printf ("sift: filter settings:\n") ;
      printf ("sift:   octaves      (O)     = %d\n",
              vl_sift_get_noctaves     (filt)) ;
      printf ("sift:   levels       (S)     = %d\n",
              vl_sift_get_nlevels      (filt)) ;
      printf ("sift:   first octave (o_min) = %d\n",
              vl_sift_get_octave_first (filt)) ;
      printf ("sift:   edge thresh           = %g\n",
              vl_sift_get_edge_thresh  (filt)) ;
      printf ("sift:   peak thresh           = %g\n",
              vl_sift_get_peak_thresh  (filt)) ;
      printf ("sift:   magnif                = %g\n",
              vl_sift_get_magnif       (filt)) ;
      printf ("sift: will source frames? %s\n",
              ikeys ? "yes" : "no") ;
      printf ("sift: will force orientations? %s\n",
              force_orientations ? "yes" : "no") ;
    }

    /* ...............................................................
     *                                             Process each octave
     * ............................................................ */
    i     = 0 ;
    first = 1 ;
    while (1) {
      VlSiftKeypoint const *keys = 0 ;
      int                   nkeys ;

      /* calculate the GSS for the next octave .................... */
      if (first) {
        first = 0 ;
        err = vl_sift_process_first_octave (filt, fdata) ;
      } else {
        err = vl_sift_process_next_octave  (filt) ;
      }

      if (err) {
        err = VL_ERR_OK ;
        break ;
      }

      if (verbose > 1) {
        printf("sift: GSS octave %d computed\n",
               vl_sift_get_octave_index (filt));
      }


      /* run detector ............................................. */
      if (ikeys == 0) {
        vl_sift_detect (filt) ;

        keys  = vl_sift_get_keypoints     (filt) ;
        nkeys = vl_sift_get_nkeypoints (filt) ;
        i     = 0 ;

        if (verbose > 1) {
          printf ("sift: detected %d (unoriented) keypoints\n", nkeys) ;
        }
      } else {
        nkeys = nikeys ;
      }

      /* for each keypoint ........................................ */
      for (; i < nkeys ; ++i) {
        double                angles [4] ;
        int                   nangles ;
        VlSiftKeypoint        ik ;
        VlSiftKeypoint const *k ;

        /* obtain keypoint orientations ........................... */
        if (ikeys) {
          vl_sift_keypoint_init (filt, &ik,
                                 ikeys [4 * i + 0],
                                 ikeys [4 * i + 1],
                                 ikeys [4 * i + 2]) ;

          if (ik.o != vl_sift_get_octave_index (filt)) {
            break ;
          }

          k          = &ik ;

          /* optionally compute orientations too */
          if (force_orientations) {
            nangles = vl_sift_calc_keypoint_orientations
              (filt, angles, k) ;
          } else {
            angles [0] = ikeys [4 * i + 3] ;
            nangles    = 1 ;
          }
        } else {
          k = keys + i ;
          nangles = vl_sift_calc_keypoint_orientations
            (filt, angles, k) ;
        }

        /* for each orientation ................................... */
        for (q = 0 ; q < (unsigned) nangles ; ++q) {
          vl_sift_pix descr [128] ;

          /* compute descriptor (if necessary) */
          //if (out.active || dsc.active) {
            vl_sift_calc_keypoint_descriptor
              (filt, descr, k, angles [q]) ;
          //}

          /*if (out.active) {
            int l ;
            vl_file_meta_put_double (&out, k -> x     ) ;
            vl_file_meta_put_double (&out, k -> y     ) ;
            vl_file_meta_put_double (&out, k -> sigma ) ;
            vl_file_meta_put_double (&out, angles [q] ) ;
            for (l = 0 ; l < 128 ; ++l) {
              vl_file_meta_put_uint8 (&out, (vl_uint8) (512.0 * descr [l])) ;
            }
            if (out.protocol == VL_PROT_ASCII) fprintf(out.file, "\n") ;
          }

          if (frm.active) {
            vl_file_meta_put_double (&frm, k -> x     ) ;
            vl_file_meta_put_double (&frm, k -> y     ) ;
            vl_file_meta_put_double (&frm, k -> sigma ) ;
            vl_file_meta_put_double (&frm, angles [q] ) ;
            if (frm.protocol == VL_PROT_ASCII) fprintf(frm.file, "\n") ;
          }

          if (dsc.active) {
            int l ;
            for (l = 0 ; l < 128 ; ++l) {
              double x = 512.0 * descr[l] ;
              x = (x < 255.0) ? x : 255.0 ;
              vl_file_meta_put_uint8 (&dsc, (vl_uint8) (x)) ;
            }
            if (dsc.protocol == VL_PROT_ASCII) fprintf(dsc.file, "\n") ;
          }
          */
        }
      }
    }

    /* ...............................................................
     *                                                       Finish up
     * ............................................................ */


  done :
    /* release input keys buffer */
    if (ikeys) {
      free (ikeys) ;
      ikeys_size = nikeys = 0 ;
      ikeys = 0 ;
    }

    /* release filter */
    if (filt) {
      vl_sift_delete (filt) ;
      filt = 0 ;
    }

    /* release image data */
    if (fdata) {
      free (fdata) ;
      fdata = 0 ;
    }

    /* release image data */
    if (data) {
      free (data) ;
      data = 0 ;
    }



    /* TODO: in case of error ... */
    /*if (err) {
      fprintf
        (stderr,
         "sift: err: %s (%d)\n",
         err_msg,
         err) ;
      exit_code = 1 ;
    }*/

  /* quit */
  return exit_code ;
}
