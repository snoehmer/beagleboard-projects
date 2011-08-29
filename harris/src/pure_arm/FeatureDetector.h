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

using namespace std;

class FeatureDetector
{
public:
	FeatureDetector(int featuresThreshold = 50, float nccThreshold = 0.5f);
	virtual ~FeatureDetector();

	void setFeatures(vector<FeatureDescriptor> features);

	bool match(ImageBitstream image);
	bool match(Image image);


private:

	int featuresThreshold_;
	float nccThreshold_;
	vector<FeatureDescriptor> features_;


	float getNCCScore(ImageBitstream image, FeatureDescriptor feature);

};

#endif /* FEATUREDETECTOR_H_ */
