/*
 * FeatureDetector.h
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#ifndef FEATUREDETECTOR_INTIMG_H_
#define FEATUREDETECTOR_INTIMG_H_

#include "../util/FeatureDescriptor.h"
#include <Magick++.h>
#include "ImageBitstream.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;


class FeatureDetectorIntImg : public FeatureDetector
{
public:
	FeatureDetectorIntImg(unsigned int featuresThreshold = 75, float nccThreshold = 0.8f);
	virtual ~FeatureDetectorIntImg();

	void setFeatures(vector<FeatureDescriptor> features);

	bool match(ImageBitstream image);
	bool match(Image image);


private:

	bool getNCCResult(unsigned char *image, unsigned int width, unsigned int height, PatchData patchData, int *imageIntegral, int *imageIntegral2, Fixed *imageSqSum, Fixed *imageAvg);

	PatchData calculatePatchData(unsigned char *patch);
	void calculateImageData(unsigned char *image, unsigned int width, unsigned int height, int *imageIntegral, int *imageIntegral2, Fixed *imageSqSum, Fixed *imageAvg);
};

#endif /* FEATUREDETECTOR_INTIMG_H_ */
