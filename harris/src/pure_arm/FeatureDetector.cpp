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
	unsigned int featuresToMatch = (featuresThreshold_ * nFeatures) / 100;

	// resize image for cross correlation
	ImageBitstream extendedImg = image.extend(FeatureDescriptor::patchSize_ / 2);

	for(i = 0; i < nFeatures; i++)
	{
		if(getNCCResult(extendedImg, features_[i]))
			matchCount++;

		if(matchCount >= featuresToMatch)
			break;
	}

	cout << "matched " << matchCount << " of " << nFeatures << " features" << endl;

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}


bool FeatureDetector::getNCCResult(ImageBitstream image, FeatureDescriptor feature)
{
	int row, col;
	int patchSize = FeatureDescriptor::patchSize_;

	float ncc;

	int patchAvg;
	int *patchNorm = new int[patchSize * patchSize];
	int patchSqSum;

	// calculate patch parameters once per patch
	calculatePatchData(feature.get(), patchAvg, patchNorm, patchSqSum);

	for(row = patchSize / 2; row < image.getHeight() - patchSize / 2; row++)
	{
		for(col = patchSize / 2; col < image.getWidth() - patchSize / 2; col++)
		{
			ncc = getNCC(image, row, col, feature, patchAvg, patchNorm, patchSqSum);

			if(ncc >= nccThreshold_)  // match if one pixel has NCC >= threshold
				return true;
		}
	}

	delete[] patchNorm;

	return false;
}

float FeatureDetector::getNCC(ImageBitstream image, int x, int y, FeatureDescriptor feature, int patchAvg, int *patchNorm, int patchSqSum)
{
	int prow, pcol;
	int irow, icol;
	int patchSize = FeatureDescriptor::patchSize_;
	int width = image.getWidth();

	unsigned char *I = image.getBitstream();

	// calculate average of image in feature patch
	int iavg = 0;

	for(prow = 0, irow = y - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
	{
		for(pcol = 0, icol = x - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
		{
			iavg += I[irow * width + icol];
		}
	}

	iavg = iavg / (patchSize * patchSize);


	// calculate NCC
	int inorm;
	int sumII = 0, sumIP = 0;
	float ncc;

	for(prow = 0, irow = y - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
	{
		for(pcol = 0, icol = x - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
		{
			inorm = I[irow * width + icol] - iavg;

			sumIP += patchNorm[prow * patchSize + pcol] * inorm;
			sumII += inorm * inorm;
		}
	}

	ncc = sumIP / (patchSqSum * sqrt(sumII));

	return ncc;
}


void FeatureDetector::calculatePatchData(unsigned char *patch, int &patchAvg, int *patchNorm, int &patchSqSum)
{
	int row, col;
	int patchSize = FeatureDescriptor::patchSize_;

	// calculate average of feature patch
	int psum = 0;

	for(row = 0; row < patchSize; row++)
	{
		for(col = 0; col < patchSize; col++)
		{
			psum += patch[row * patchSize + col];
		}
	}

	patchAvg = psum / (patchSize * patchSize);


	// now calculate normalized patch and sqSum
	int sqSum = 0;

	for(row = 0; row < patchSize; row++)
	{
		for(col = 0; col < patchSize; col++)
		{
			patchNorm[row * patchSize + col] = patch[row * patchSize + col] - patchAvg;

			sqSum += patchNorm[row * patchSize + col] * patchNorm[row * patchSize + col];
		}
	}

	patchSqSum = (int) sqrt(sqSum);
}
