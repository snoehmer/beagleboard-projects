/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector.h"
#include "FeatureDetector_IntImg.h"
#include "../util/TimeMeasureBase.h"
#include "../util/Logger.h"
#include <Magick++.h>


FeatureDetectorIntImg::FeatureDetectorIntImg(unsigned int featuresThreshold, float nccThreshold) : FeatureDetector(featuresThreshold, nccThreshold)
{
  Logger::debug(Logger::NCC, "initializing integral image NCC detector with features threshold %d\%, NCC threshold %f", featuresThreshold, nccThreshold);
}


FeatureDetectorIntImg::~FeatureDetectorIntImg()
{
	unsigned int i;

	for(i = 0; i < features_.size(); i++)
	{
		delete[] featureData_[i].patchNorm_;
	}

	featureData_.clear();

	features_.clear();
}


void FeatureDetectorIntImg::setFeatures(vector<FeatureDescriptor> features)
{
  Logger::debug(Logger::NCC, "reading %d features", features.size());

	unsigned int i;

	if(features_.size() > 0)  // cleanup old features
  {
    for(i = 0; i < features_.size(); i++)
    {
      delete[] featureData_[i].patchNorm_;
    }

    featureData_.clear();

    features_.clear();
  }

	features_ = features;
	featureData_.resize(features_.size());  // pre-allocate for performance

	startTimer("_ncc_patchdata_arm");

	for(i = 0; i < features_.size(); i++)
	{
		featureData_[i] = calculatePatchData(features_[i].get());
	}

	stopTimer("_ncc_patchdata_arm");
}


bool FeatureDetectorIntImg::match(Image image)
{
	ImageBitstream bitstream(image);

	return match(bitstream);
}


bool FeatureDetectorIntImg::match(ImageBitstream image)
{
  Logger::debug(Logger::NCC, "matching image (size %dx%d)", image.getWidth(), image.getHeight());

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
	float *imageSqSum = new float[image.getWidth() * image.getHeight()];
	float *imageAvg = new float[image.getWidth() * image.getHeight()];

	calculateImageData(extendedImg.getBitstream(), extendedImg.getWidth(), extendedImg.getHeight(), imageIntegral, imageIntegral2, imageSqSum, imageAvg);

	// calculate NCC for each feature
	for(i = 0; i < nFeatures; i++)
	{
		if(getNCCResult(extendedImg.getBitstream(), extWidth, extHeight, featureData_[i], imageIntegral, imageIntegral2, imageSqSum, imageAvg))
			matchCount++;

		if(matchCount >= featuresToMatch)
			break;
	}

	Logger::debug(Logger::NCC, "matched %d of %d features", matchCount, nFeatures);

	delete[] imageIntegral;
	delete[] imageIntegral2;
	delete[] imageSqSum;

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}


bool FeatureDetectorIntImg::getNCCResult(unsigned char *image, unsigned int width, unsigned int height, PatchData patchData, int *imageIntegral, int *imageIntegral2, float *imageSqSum, float *imageAvg)
{
	unsigned int row, col;
	unsigned int prow, pcol;
	unsigned int irow, icol;

	unsigned int patchSize = FeatureDescriptor::patchSize_;


	// calculate patch parameters once per patch
	float *patchNorm = patchData.patchNorm_;
	float patchSqSum = patchData.patchSqSum_;

	// calculate NCC
	float sumIP = 0;
	float ncc = 0.0f;

	for(row = patchSize / 2; row < height - patchSize / 2; row++)
	{
		for(col = patchSize / 2; col < width - patchSize / 2; col++)
		{
			sumIP = 0;

			if(imageSqSum[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)] == 0)
			{
				ncc = 0.0f;  // completely homogenous area -> we wont find matches here!
			}
			else
			{
				for(prow = 0, irow = row - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
				{
					for(pcol = 0, icol = col - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
					{
						sumIP += patchNorm[prow * patchSize + pcol] * ((float)image[irow * width + icol] - imageAvg[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)]);
						//sumIP += patchNorm[prow * patchSize + pcol] * image[irow * width + icol]; //TODO
					}
				}

				ncc = sumIP / (patchSqSum * imageSqSum[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)]);
			}

			if(ncc >= nccThreshold_)  // match if one pixel has NCC >= threshold
				break;
		}

		if(ncc >= nccThreshold_)
			break;
	}

	if(ncc >= nccThreshold_)
		return true;
	else
		return false;
}


void FeatureDetectorIntImg::calculateImageData(unsigned char *image, unsigned int width, unsigned int height, int *imageIntegral, int *imageIntegral2, float *imageSqSum, float *imageAvg)
{
	unsigned int row, col;

	//cout << "calculating image data...";

	startTimer("_ncc_imagedata_single_arm");

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

			imageSqSum[(row - offset) * imageWidth + (col - offset)] = sqrt((float)A - ((float)(B * B)) / ((float)(patchSize * patchSize)));
			imageAvg[(row - offset) * imageWidth + (col - offset)] = B / (float(patchSize * patchSize));
		}
	}

	stopTimer("_ncc_imagedata_single_arm");

	//cout << "finished" << endl;
}

PatchData FeatureDetectorIntImg::calculatePatchData(unsigned char *patch)
{
	int row, col;
	int patchSize = FeatureDescriptor::patchSize_;

	float patchAvg;
	float *patchNorm = new float[patchSize * patchSize];
	float patchSqSum;

	//cout << "calculating patch data...";

	startTimer("_ncc_patchdata_single_arm");

	// calculate average of feature patch
	int psum = 0;

	for(row = 0; row < patchSize; row++)
	{
		for(col = 0; col < patchSize; col++)
		{
			psum += patch[row * patchSize + col];
		}
	}

	patchAvg = (float)psum / ((float)(patchSize * patchSize));


	// now calculate normalized patch and sqSum
	float sqSum = 0;
	//int testPatchNorm = 0;

	for(row = 0; row < patchSize; row++)
	{
		for(col = 0; col < patchSize; col++)
		{
			patchNorm[row * patchSize + col] = (float)patch[row * patchSize + col] - patchAvg;

			sqSum += patchNorm[row * patchSize + col] * patchNorm[row * patchSize + col];
			//testPatchNorm += patchNorm[row * patchSize + col];
		}
	}

	patchSqSum = sqrt(sqSum);

	stopTimer("_ncc_patchdata_single_arm");

	//cout << "finished" << endl;
	//cout << "patch average is " << patchAvg << ", patch normalization sum is " << testPatchNorm << endl;

	PatchData pd;
	pd.patchAvg_ = patchAvg;
	pd.patchNorm_ = patchNorm;
	pd.patchSqSum_ = patchSqSum;

	return pd;
}
