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

#include "Exception.h"

struct KeyPointDescriptor
{
  VlSiftKeypoint keypoint;
  vl_sift_pix descr [128] ;
  double angle;
};

class Dsp;

class Sift
{
protected:
  VlPgmImage pim;
  vl_uint8 *data;
  vl_sift_pix *fdata;
  std::vector<KeyPointDescriptor> detected_keypoints;
  Dsp* dsp;

public:
  virtual int Detect();
  virtual void ReadImageFromFile(char* filename);

  Sift();

  virtual ~Sift();

  virtual std::vector<KeyPointDescriptor>& GetDetectedKeypoints()
  {
    return detected_keypoints;
  }

};

class SiftException : public Exception
{
public:
  SiftException(const char* msg) : Exception(msg)
  {
  }
};

#endif /* SIFT_H_ */
