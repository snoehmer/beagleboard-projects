/*
 * FeatureDetector.h
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#ifndef FEATUREDETECTOR_H_
#define FEATUREDETECTOR_H_

#include "../util/FeatureDescriptor.h"
#include <Magick++.h>
#include "ImageBitstream.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

struct PatchData
{
	int patchAvg_;
	int *patchNorm_;
	int *patchNormSq_;
	int patchSqSum_;
};


/**
 * Base class for feature detectors, standard and slow NCC-based detector
 */
class FeatureDetector
{
public:
	FeatureDetector(unsigned int featuresThreshold = 75, float nccThreshold = 0.8f);
	virtual ~FeatureDetector();

	virtual void setFeatures(vector<FeatureDescriptor> features);

	virtual bool match(ImageBitstream image);
	virtual bool match(Image image);


protected:

	unsigned int featuresThreshold_;
	float nccThreshold_;
	vector<FeatureDescriptor> features_;
	vector<PatchData> featureData_;


private:

	virtual bool getNCCResult(unsigned char *image, unsigned int width, unsigned int height, PatchData patchData);
	virtual PatchData calculatePatchData(unsigned char *patch);
};

#endif /* FEATUREDETECTOR_H_ */
