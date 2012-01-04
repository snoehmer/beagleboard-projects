/*
 * FeatureDetector.h
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#ifndef FEATUREDETECTOR_HARRIS_STD_H_
#define FEATUREDETECTOR_HARRIS_STD_H_

#include "HarrisCornerDetector.h"
#include "FeatureDetector.h"
#include "../util/FeatureDescriptor.h"
#include <Magick++.h>
#include "ImageBitstream.h"
#include <vector>
#include <cmath>
#include <iostream>
#include "FixedArithmetic.h"

using namespace std;


/**
 * Feature detector, using NCC-based detector with Harris corner detection
 */
class FeatureDetectorHarrisStd : public FeatureDetector
{
public:
	FeatureDetectorHarrisStd(unsigned int featuresThreshold = 75, float nccThreshold = 0.8f, float harrisThreshold = 0.8f, float harrisK = 0.06f, float harrisDSigma = 1.0f, int harrisDKernelSize = 3, float harrisGSigma = 2.0f, int harrisGKernelSize = 5);
	virtual ~FeatureDetectorHarrisStd();

	virtual bool match(ImageBitstream image);


protected:
	float harrisThreshold_;
	float harrisK_;
	float harrisDSigma_;
	int harrisDKernelSize_;
	float harrisGSigma_;
	int harrisGKernelSize_;

	vector<HarrisCornerPoint> cornerPoints_;
	HarrisCornerDetector *hcd_;


	virtual bool getNCCResult(unsigned char *image, unsigned int width, unsigned int height, unsigned int row, unsigned int col, PatchData patchData);

};

#endif /* FEATUREDETECTOR_HARRIS_STD_H_ */
