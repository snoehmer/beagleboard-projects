/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector.h"


FeatureDetector::FeatureDetector(int featuresThreshold, float nccThreshold)
{
	featuresThreshold_ = featuresThreshold;
	nccThreshold_ = nccThreshold;
}


FeatureDetector::~FeatureDetector()
{
	features_.clear();
}


void FeatureDetector::setFeatures(vector<FeatureDescriptor> features)
{
	features_ = features;
}


bool FeatureDetector::match(Image image)
{
	return false;
}


bool FeatureDetector::match(ImageBitstream image)
{
	return false;
}


float FeatureDetector::getNCCScore(ImageBitstream image, FeatureDescriptor feature)
{
	return 0.0f;
}
