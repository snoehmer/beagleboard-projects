/*
 * sifttest.cpp
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>


#include "generic-driver.h"

#include "lib/sift.h"
#include "lib/logger.h"


int main(int argc, char** argv)
{
  VlFileMeta out  = {1, "%.sift",  VL_PROT_ASCII, "", 0};
  vl_bool  err    = VL_ERR_OK ;
  vl_size          q;
  char basename[21];

  if(argc != 2)
  {
    printf("usage: sifttest <image.pgm>\n");
    return -1;
  }

  Logger::init();


  /* get basename from filename */
  q = vl_string_basename (basename, sizeof(basename), argv[1], 1) ;

  err = (q >= sizeof(basename));

  if (err)
  {
    printf("Basename of '%s' is too long", argv[1]);
    return -1;
  }


  err = vl_file_meta_open (&out, basename, "wb");

  if (err == VL_ERR_OVERFLOW)
  {
    printf("Output file name too long.\n") ;
    return -1;
  }
  else if (err)
  {
    printf("Could not open '%s' for writing\n", out.name);
    return -1;
  }

  Sift sift;

  Logger::info(Logger::SIFTTEST, "reading Image %s from file", argv[1]);

  sift.ReadImageFromFile(argv[1]);


  Logger::info(Logger::SIFTTEST, "detecting keypoints");
  sift.Detect();

  std::vector<KeyPointDescriptor>& keypoints = sift.GetDetectedKeypoints();

  Logger::info(Logger::SIFTTEST, "writing keypoints to file");

  for(unsigned i = 0; i < keypoints.size(); i++)
  {
    KeyPointDescriptor k = keypoints[i];

    vl_file_meta_put_double(&out, k.keypoint.x);
    vl_file_meta_put_double(&out, k.keypoint.y);
    vl_file_meta_put_double(&out, k.keypoint.sigma);
    vl_file_meta_put_double(&out, k.angle);

    for (int l = 0 ; l < 128 ; ++l)
    {
      vl_file_meta_put_uint8 (&out, (vl_uint8) (512.0 * k.descr [l])) ;
    }
    if (out.protocol == VL_PROT_ASCII) fprintf(out.file, "\n");
  }

  return 0;
}
