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

	virtual void setFeatures(vector<FeatureDescriptor> features);

	virtual bool match(ImageBitstream image);
	virtual bool match(Image image);


protected:

	virtual bool getNCCResult(unsigned char *image, unsigned int width, unsigned int height, PatchData patchData, Fixed *imageIntegral, Fixed *imageIntegral2, Fixed *imageSqSum, Fixed *imageAvg);

	virtual PatchData calculatePatchData(unsigned char *patch);
	virtual void calculateImageData(unsigned char *image, unsigned int width, unsigned int height, Fixed *imageIntegral, Fixed *imageIntegral2, Fixed *imageSqSum, Fixed *imageAvg);
};

#endif /* FEATUREDETECTOR_INTIMG_H_ */
