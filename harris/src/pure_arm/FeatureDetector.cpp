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
	unsigned int featuresToMatch = featuresThreshold_ * nFeatures / 100;

	unsigned int width = image.getWidth();
	unsigned int height = image.getHeight();

	cout << "resizing" << endl;

	// resize image for cross correlation
	ImageBitstream extendedImg = image.extend(FeatureDescriptor::patchSize_ / 2);
	unsigned int extWidth = width + FeatureDescriptor::patchSize_;
	unsigned int extHeight = height + FeatureDescriptor::patchSize_;

	// new image loaded, so image data must be updated
	bool *imageDataCalculated = new bool[extWidth * extHeight];
	int *imageAvg = new int[extWidth * extHeight];
	int *imageSqSum = new int[extWidth * extHeight];

	for(i = 0; i < extWidth * extHeight; i++)
		imageDataCalculated[i] = false;

	for(i = 0; i < nFeatures; i++)
	{
		cout << "calculating NCC result for feature #" << i << endl;

		if(getNCCResult(extendedImg, features_[i], imageAvg, imageSqSum, imageDataCalculated))
			matchCount++;

		if(matchCount >= featuresToMatch)
			break;

		if(i % (nFeatures / 10) == 0)
			cout << "#";
	}

	delete[] imageSqSum;
	delete[] imageDataCalculated;

	if(i < nFeatures)
		return true;
	else
		return false;
}


bool FeatureDetector::getNCCResult(ImageBitstream image, FeatureDescriptor feature, int *imageAvg, int *imageSqSum, bool *imageDataCalculated)
{
	int row, col;
	int patchSize = FeatureDescriptor::patchSize_;

	float ncc = 0;

	// new patch, so patch data must be updated
	bool patchDataCalculated = false;
	int patchAvg;
	int *patchNorm = new int[patchSize * patchSize];
	int patchSqSum;

	for(row = (patchSize - 1) / 2; row < image.getHeight() - patchSize / 2; row++)
	{
		for(col = (patchSize - 1) / 2; col < image.getWidth() - patchSize / 2; col++)
		{
			ncc = getNCC(image, row, col, feature, imageAvg, imageSqSum, imageDataCalculated, patchAvg, patchNorm, patchSqSum, patchDataCalculated);

			if(ncc >= nccThreshold_)  // match if one pixel has NCC >= threshold
				break;
		}

		if(row % (image.getHeight() / 20) == 0)
			cout << ".";
	}

	delete[] patchNorm;

	if(ncc >= nccThreshold_)
		return true;
	else
		return false;
}

float FeatureDetector::getNCC(ImageBitstream image, int x, int y, FeatureDescriptor feature, int *imageAvg, int *imageSqSum, bool *imageDataCalculated, int &patchAvg, int *patchNorm, int &patchSqSum, bool &patchDataCalculated)
{
	int prow, pcol;
	int irow, icol;
	int patchSize = FeatureDescriptor::patchSize_;
	int width = image.getWidth();

	unsigned char *I = image.getBitstream();
	unsigned char *P = feature.get();

	// calculate average of image in feature patch
	if(!patchDataCalculated || !imageDataCalculated[y * width + x])
	{
		int psum = 0, isum = 0;

		for(prow = 0, irow = y - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
		{
			for(pcol = 0, icol = x - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
			{
				if(!patchDataCalculated)
					psum += P[prow * patchSize + pcol];

				if(!imageDataCalculated[y * width + x])
					isum += I[irow * width + icol];
			}
		}

		if(!patchDataCalculated)
			patchAvg = psum / (patchSize * patchSize);

		if(!imageDataCalculated[y * width + x])
			imageAvg[y * width + x] = isum / (patchSize * patchSize);

	}


	// calculate NCC
	int inorm;
	int sumII = 0, sumPP = 0, sumIP = 0;
	float ncc;

	for(prow = 0, irow = y - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
	{
		for(pcol = 0, icol = x - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
		{
			if(!patchDataCalculated)
			{
				patchNorm[prow * patchSize + pcol] = P[prow * patchSize + pcol] - patchAvg;
				sumPP += patchNorm[prow * patchSize + pcol] * patchNorm[prow * patchSize + pcol];

			}

			if(!imageDataCalculated[y * width + x])
			{
				inorm = I[irow * width + icol] - imageAvg[y * width + x];
				sumII += inorm * inorm;
			}

			sumIP += patchNorm[prow * patchSize + pcol] * (I[irow * width + icol] - imageAvg[y * width + x]);
		}
	}

	if(!patchDataCalculated)
	{
		patchSqSum = sqrt(sumPP);
		patchDataCalculated = true;
	}

	if(!imageDataCalculated[y * width + x])
	{
		imageSqSum[y * width + x] = sqrt(sumII);
		imageDataCalculated[y * width + x] = true;
	}

	if(sumIP == 0)
		return 0.0f;

	ncc = sumIP / (patchSqSum * imageSqSum[y * width + x]);

	return ncc;
}
