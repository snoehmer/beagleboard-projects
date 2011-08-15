/*
 * sift.h
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#ifndef SIFT_H_
#define SIFT_H_

#include <vector>
#include <vl/sift.h>
#include <vl/generic.h>
#include <vl/generic.h>
#include <vl/stringop.h>
#include <vl/pgm.h>

struct KeyPointDescriptor
{
  VlSiftKeypoint keypoint;
  vl_sift_pix descr [128] ;
  double angle;
};


class Sift
{
private:
  VlPgmImage pim;
  vl_uint8 *data;
  vl_sift_pix *fdata;
  std::vector<KeyPointDescriptor> detected_keypoints;

public:
  int Detect();
  void ReadImageFromFile(char* filename);

  Sift()
  {
    data = 0;
    fdata = 0;
  }

  ~Sift()
  {
    /* release image data */
    if (fdata)
    {
      free (fdata);
      fdata = 0;
    }

    /* release image data */
    if (data)
    {
      free (data) ;
      data = 0 ;
    }

  }

  std::vector<KeyPointDescriptor>& GetDetectedKeypoints()
  {
    return detected_keypoints;
  }

};

#endif /* SIFT_H_ */
