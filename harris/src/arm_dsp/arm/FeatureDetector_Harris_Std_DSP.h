/*
 * FeatureDetector.h
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#ifndef FEATUREDETECTOR_HARRIS_STD_DSP_H_
#define FEATUREDETECTOR_HARRIS_STD_DSP_H_

#include "../../pure_arm/HarrisCornerDetector.h"
#include "../../pure_arm/FeatureDetector_Harris_Std.h"
#include "../../util/FeatureDescriptor.h"
#include <Magick++.h>
#include "../../pure_arm/ImageBitstream.h"
#include <vector>
#include <cmath>
#include <iostream>
#include "../../pure_arm/FixedArithmetic.h"
#include "dspbridge/Dsp.h"

using namespace std;


/**
 * Feature detector, using NCC-based detector with Harris corner detection and DSP support
 */
class FeatureDetectorHarrisStdDSP : public FeatureDetectorHarrisStd
{
public:
	FeatureDetectorHarrisStdDSP(unsigned int featuresThreshold = 75, float nccThreshold = 0.8f, float harrisThreshold = 0.8f, float harrisK = 0.06f, float harrisDSigma = 1.0f, int harrisDKernelSize = 3, float harrisGSigma = 2.0f, int harrisGKernelSize = 5);
	virtual ~FeatureDetectorHarrisStdDSP();

	bool match(ImageBitstream image);

protected:

	Dsp *dspInstance_;
  DspNode *dspNode_;

	virtual bool getNCCResult(int *image, unsigned int width, unsigned int height, unsigned int row, unsigned int col, PatchData patchData);
  virtual PatchData calculatePatchData(unsigned char *patch);
};

#endif /* FEATUREDETECTOR_HARRIS_STD_DSP_H_ */
