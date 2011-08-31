/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector.h"
#include <Magick++.h>


FeatureDetector::FeatureDetector(unsigned int featuresThreshold, float nccThreshold)
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
	ImageBitstream bitstream(image);

	return match(bitstream);
}


bool FeatureDetector::match(ImageBitstream image)
{
	unsigned int i;
	unsigned int matchCount = 0;
	unsigned int nFeatures = features_.size();

	// resize image for cross correlation
	ImageBitstream extendedImg = image.extend(FeatureDescriptor::patchSize_ / 2);

	Image temp;
	temp.read(extendedImg.getWidth(), extendedImg.getHeight(), "I", CharPixel, extendedImg.getBitstream());
	temp.write("../output/extNCC.jpg");

	for(i = 0; i < nFeatures; i++)
	{
		if(getNCCResult(extendedImg, features_[i]))
			matchCount++;

		if((matchCount * 100) / nFeatures > featuresThreshold_)
			return true;
	}

	return false;
}


bool FeatureDetector::getNCCResult(ImageBitstream image, FeatureDescriptor feature)
{
	int row, col;
	int patchSize = FeatureDescriptor::patchSize_;
	int realWidth = image.getWidth() - patchSize/2 - (patchSize-1)/2;
	int realHeight = image.getHeight() - patchSize/2 - (patchSize-1)/2;
	static int idx = 0;

	float ncc;
	float *nccI = new float[realWidth * realHeight];

	for(row = (patchSize - 1) / 2; row < image.getHeight() - patchSize / 2; row++)
	{
		for(col = (patchSize - 1) / 2; col < image.getWidth() - patchSize / 2; col++)
		{
			ncc = getNCC(image, row, col, feature);
			nccI[(row - (patchSize-1)/2) * realWidth + (col - (patchSize-1)/2)] = ncc;

			//if(ncc >= nccThreshold_)  // match if one pixel has NCC >= threshold
				//return true;
		}
	}

	char tempName[30];
	Image temp;
	temp.read(realWidth, realHeight, "I", FloatPixel, nccI);
	sprintf(tempName, "../output/ncc%d.jpg", idx);
	temp.write(tempName);

	return false;
}

float FeatureDetector::getNCC(ImageBitstream image, int x, int y, FeatureDescriptor feature)
{
	int row, col;
	int patchSize = FeatureDescriptor::patchSize_;
	int width = image.getWidth();

	unsigned char *I = image.getBitstream();
	unsigned char *P = feature.get();

	// calculate average of image in feature patch
	float pavg = 0, iavg = 0;

	for(row = 0; row < patchSize; row++)
	{
		for(col = 0; col < patchSize; col++)
		{
			pavg += P[row * patchSize + col];

			iavg += I[(y + row - (patchSize - 1)/2) * width + (x + col - (patchSize - 1)/2)];
		}
	}

	pavg = pavg / (patchSize * patchSize);
	iavg = iavg / (patchSize * patchSize);


	// calculate NCC
	float pnorm, inorm;
	float sumII = 0, sumPP = 0, sumIP = 0;
	float ncc;

	for(row = 0; row < patchSize; row++)
	{
		for(col = 0; col < patchSize; col++)
		{
			pnorm = P[row * patchSize + col] - pavg;
			inorm = I[(y + row - (patchSize - 1)/2) * width + (x + col - (patchSize - 1)/2)] - iavg;

			sumIP += pnorm * inorm;
			sumPP += pnorm * pnorm;
			sumII += inorm * inorm;
		}
	}

	ncc = sumIP / (sqrt(sumPP) * sqrt(sumII));

	return ncc;
}
