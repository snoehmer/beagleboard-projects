/*
 * FeatureGenerator.h
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#ifndef FEATUREGENERATOR_H_
#define FEATUREGENERATOR_H_

#include "HarrisCornerPoint.h"
#include "FeatureDescriptor.h"
#include "../pure_arm/ImageBitstream.h"
#include <vector>

using namespace std;

class FeatureGenerator
{
public:
	FeatureGenerator();
	virtual ~FeatureGenerator();

	vector<FeatureDescriptor> generateFeatures(ImageBitstream image, vector<HarrisCornerPoint> corners);
};

#endif /* FEATUREGENERATOR_H_ */
