/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector.h"
#include "../util/TimeMeasureBase.h"
#include "../util/Logger.h"
#include <Magick++.h>


FeatureDetector::FeatureDetector(unsigned int featuresThreshold, float nccThreshold)
{
  Logger::debug(Logger::NCC, "initializing standard NCC detector with features threshold %d\%, NCC threshold %f", featuresThreshold, nccThreshold);

	featuresThreshold_ = featuresThreshold;
	nccThreshold_ = nccThreshold;
}


FeatureDetector::~FeatureDetector()
{
  unsigned int i = 0;

  for(i = 0; i < features_.size(); i++)
  {
    delete[] featureData_[i].patchNorm_;
    delete[] featureData_[i].patchNormSq_;
  }

  featureData_.clear();

	features_.clear();
}


void FeatureDetector::setFeatures(vector<FeatureDescriptor> features)
{
  Logger::debug(Logger::NCC, "reading %d features", features.size());

  unsigned int i;

  if(features_.size() > 0)  // cleanup old features
  {
    for(i = 0; i < features_.size(); i++)
    {
      delete[] featureData_[i].patchNorm_;
      delete[] featureData_[i].patchNormSq_;
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


bool FeatureDetector::match(Image image)
{
	ImageBitstream bitstream(image);

	return match(bitstream);
}


bool FeatureDetector::match(ImageBitstream image)
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

	// calculate NCC for each feature
	for(i = 0; i < nFeatures; i++)
	{
		if(getNCCResult(extendedImg.getBitstream(), extWidth, extHeight, featureData_[i]))
			matchCount++;

		if(matchCount >= featuresToMatch)
			break;
	}

	Logger::debug(Logger::NCC, "matched %d of %d features", matchCount, nFeatures);

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}


bool FeatureDetector::getNCCResult(unsigned char *image, unsigned int width, unsigned int height, PatchData patchData)
{
	unsigned int row, col;
	unsigned int prow, pcol;
	unsigned int irow, icol;

	unsigned int patchSize = FeatureDescriptor::patchSize_;

	Fixed *patchNorm = patchData.patchNorm_;
  Fixed *patchNormSq = patchData.patchNormSq_;

	// calculate NCC
	Fixed iavg;
	Fixed inorm;
	Fixed sumIP;
	Fixed sumPP;
	Fixed sumII;
	Fixed ncc;

	for(row = patchSize / 2; row < height - patchSize / 2; row++)
	{
		for(col = patchSize / 2; col < width - patchSize / 2; col++)
		{
		  // calculate average of image at current patch
		  iavg = 0;

		  for(prow = 0, irow = row - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
      {
        for(pcol = 0, icol = col - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
        {
          iavg += scale_uchar(image[irow * width + icol]);
        }
      }

		  iavg = iavg / ((int) (patchSize * patchSize));


		  // calculate NCC
			sumIP = 0;
			sumPP = 0;
			sumII = 0;

			for(prow = 0, irow = row - (patchSize - 1)/2; prow < patchSize; prow++, irow++)
      {
        for(pcol = 0, icol = col - (patchSize - 1)/2; pcol < patchSize; pcol++, icol++)
        {
          inorm = scale_uchar(image[irow * width + icol]) - iavg;

          sumIP += patchNorm[prow * patchSize + pcol] * inorm;
          sumPP += patchNormSq[prow * patchSize + pcol];
          sumII += inorm * inorm;
        }
      }

			if(sumPP == 0 || sumII == 0)  // we wont find any corners in a homogenous area
			{
			  ncc = 0.0f;
			}
			else
			{
			  //ncc = sumIP / (sqrt(sumPP) * sqrt(sumII));
			  ncc = sumIP / ((sumPP * sumII).sqrt());
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


PatchData FeatureDetector::calculatePatchData(unsigned char *patch)
{
  int row, col;
  int patchSize = FeatureDescriptor::patchSize_;

  Fixed patchAvg;
  Fixed *patchNorm = new Fixed[patchSize * patchSize];
  Fixed *patchNormSq = new Fixed[patchSize * patchSize];

  startTimer("_ncc_patchdata_single_arm");

  // calculate average of feature patch
  Fixed psum = 0;

  for(row = 0; row < patchSize; row++)
  {
    for(col = 0; col < patchSize; col++)
    {
      psum += scale_uchar(patch[row * patchSize + col]);
    }
  }

  patchAvg = psum / (patchSize * patchSize);


  // now calculate normalized patch and squared normalized patch
  for(row = 0; row < patchSize; row++)
  {
    for(col = 0; col < patchSize; col++)
    {
      patchNorm[row * patchSize + col] = scale_uchar(patch[row * patchSize + col]) - patchAvg;

      patchNormSq[row * patchSize + col] = patchNorm[row * patchSize + col] * patchNorm[row * patchSize + col];
    }
  }

  stopTimer("_ncc_patchdata_single_arm");

  PatchData pd;
  pd.patchAvg_ = patchAvg;
  pd.patchNorm_ = patchNorm;
  pd.patchNormSq_ = patchNormSq;

  return pd;
}
