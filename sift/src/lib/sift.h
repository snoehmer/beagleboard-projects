/*
 * sift.h
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#ifndef SIFT_H_
#define SIFT_H_


class Sift
{
private:
  VlPgmImage pim;
  vl_uint8 *data;
  vl_sift_pix *fdata;

public:
  int Detect();
  void ReadImageFromFile(char* filename);

  Sift()
  {
    data = 0;
    fdata = 0;
  }
};

#endif /* SIFT_H_ */
