/*
 * FeatureGenerator.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureGenerator.h"

using namespace std;


FeatureGenerator::FeatureGenerator()
{
}


FeatureGenerator::~FeatureGenerator()
{
}


vector<FeatureDescriptor> FeatureGenerator::generateFeatures(ImageBitstream image, vector<HarrisCornerPoint> corners)
{
	vector<FeatureDescriptor> features;
	int i;
	FeatureDescriptor feature;
	HarrisCornerPoint corner;

	features.resize(corners.size());  // pre-allocation for performance

	for(i = 0; i < corners.size(); i++)  // generate a descriptor for each corner
	{
		corner = corners[i];
		feature = FeatureDescriptor(image, corner);
		features[i] = feature;
	}

	return features;
}
