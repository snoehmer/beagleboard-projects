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
	int patchSqSum_;
};


class FeatureDetector
{
public:
	FeatureDetector(unsigned int featuresThreshold = 75, float nccThreshold = 0.8f);
	virtual ~FeatureDetector();

	void setFeatures(vector<FeatureDescriptor> features);

	bool match(ImageBitstream image);
	bool match(Image image);


private:

	unsigned int featuresThreshold_;
	float nccThreshold_;
	vector<FeatureDescriptor> features_;
	vector<PatchData> featureData_;


	bool getNCCResult(unsigned char *image, unsigned int width, unsigned int height, PatchData patchData, int *imageIntegral, int *imageIntegral2, int* imageSqSum, int *imageAvg);
	float getNCC(ImageBitstream image, int x, int y, FeatureDescriptor feature, int patchAvg, int *patchNorm, int patchSqSum, int *imageIntegral, int *imageIntegral2, int* imageSqSum);

	PatchData calculatePatchData(unsigned char *patch);
	void calculateImageData(unsigned char *image, unsigned int width, unsigned int height, int *imageIntegral, int *imageIntegral2, int *imageSqSum, int *imageAvg);
};

#endif /* FEATUREDETECTOR_H_ */
