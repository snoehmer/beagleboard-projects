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

using namespace std;

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


	bool getNCCResult(ImageBitstream image, FeatureDescriptor feature);
	float getNCC(ImageBitstream image, int x, int y, FeatureDescriptor feature);
};

#endif /* FEATUREDETECTOR_H_ */
