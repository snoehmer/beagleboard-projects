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
	unsigned int extWidth = extendedImg.getWidth();
	unsigned int extHeight = extendedImg.getHeight();

	// calculate image parameters once per image
	int *imageIntegral = new int[(extWidth + 1) * (extHeight + 1)];
	int *imageIntegral2 = new int[(extWidth + 1) * (extHeight + 1)];
	int *imageSqSum = new int[image.getWidth() * image.getHeight()];

	calculateImageData(extendedImg.getBitstream(), extendedImg.getWidth(), extendedImg.getHeight(), imageIntegral, imageIntegral2, imageSqSum);


	// calculate NCC for each feature
	for(i = 0; i < nFeatures; i++)
	{
		if(getNCCResult(extendedImg, features_[i], imageIntegral, imageIntegral2, imageSqSum))
			matchCount++;

		if(matchCount >= featuresToMatch)
			break;
	}

	cout << "matched " << matchCount << " of " << nFeatures << " features" << endl;

	delete[] imageIntegral;
	delete[] imageIntegral2;
	delete[] imageSqSum;

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}


bool FeatureDetector::getNCCResult(ImageBitstream image, FeatureDescriptor feature, int *imageIntegral, int *imageIntegral2, int* imageSqSum)
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
			ncc = getNCC(image, row, col, feature, patchAvg, patchNorm, patchSqSum, imageIntegral, imageIntegral2, imageSqSum);

			if(ncc >= nccThreshold_)  // match if one pixel has NCC >= threshold
				return true;
		}
	}

	delete[] patchNorm;

	return false;
}

float FeatureDetector::getNCC(ImageBitstream image, int x, int y, FeatureDescriptor feature, int patchAvg, int *patchNorm, int patchSqSum, int *imageIntegral, int *imageIntegral2, int* imageSqSum)
{
	int prow, pcol;
	int irow, icol;
	int patchSize = FeatureDescriptor::patchSize_;
	int width = image.getWidth();

	unsigned char *I = image.getBitstream();

	// calculate NCC
	int sumIP = 0;
	float ncc;

	for(prow = 0, irow = y - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
	{
		for(pcol = 0, icol = x - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
		{
			sumIP += patchNorm[prow * patchSize + pcol] * I[irow * width + icol];
		}
	}

	ncc = sumIP / (patchSqSum * imageSqSum[(y - patchSize/2) * (width - patchSize) + (x - patchSize/2)]);

	return ncc;
}


void FeatureDetector::calculateImageData(unsigned char *image, unsigned int width, unsigned int height, int *imageIntegral, int *imageIntegral2, int *imageSqSum)
{
	unsigned int row, col;

	// initialize integral arrays
	for(col = 0; col < width + 1; col++)  // fill 1st row with zeros
	{
		imageIntegral[0 * (width + 1) + col] = 0;
		imageIntegral2[0 * (width + 1) + col] = 0;
	}

	for(row = 1; row < height + 1; row++)  // fill 1st column with zeros
	{
		imageIntegral[row * (width + 1) + 0] = 0;
		imageIntegral2[row * (width + 1) + 0] = 0;
	}


	// calculate integral arrays
	for(row = 0; row < height; row++)
	{
		for(col = 0; col < width; col++)
		{
			imageIntegral[(row + 1) * (width + 1) + (col + 1)] = image[row * width + col] + imageIntegral[row * (width + 1) + (col + 1)] + imageIntegral[(row + 1) * (width + 1) + col] - imageIntegral[row * (width + 1) + col];
			imageIntegral2[(row + 1) * (width + 1) + (col + 1)] = image[row * width + col] * image[row * width + col] + imageIntegral2[row * (width + 1) + (col + 1)] + imageIntegral2[(row + 1) * (width + 1) + col] - imageIntegral2[row * (width + 1) + col];
		}
	}

	// calculate sqSum array
	unsigned int patchSize = FeatureDescriptor::patchSize_;
	unsigned int offset = patchSize / 2;
	unsigned int imageWidth = width - patchSize;
	int A, B;
	int dp = patchSize / 2;  // positive delta
	int dn = (patchSize - 1) / 2 + 1;  // negative delta

	for(row = offset; row < height - offset; row++)
	{
		for(col = offset; col < width - offset; col++)
		{
			A = imageIntegral2[(row + dp + 1) * (width + 1) + (col + dp + 1)] - imageIntegral2[(row - dn + 1) * (width + 1) + (col + dp + 1)] - imageIntegral2[(row + dp + 1) * (width + 1) + (col - dn + 1)] + imageIntegral2[(row - dn + 1) * (width + 1) + (col - dn + 1)];
			B = imageIntegral[(row + dp + 1) * (width + 1) + (col + dp + 1)] - imageIntegral[(row - dn + 1) * (width + 1) + (col + dp + 1)] - imageIntegral[(row + dp + 1) * (width + 1) + (col - dn + 1)] + imageIntegral[(row - dn + 1) * (width + 1) + (col - dn + 1)];

			imageSqSum[(row - offset) * imageWidth + (col - offset)] = A - (B * B) / (patchSize * patchSize);
		}
	}
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
