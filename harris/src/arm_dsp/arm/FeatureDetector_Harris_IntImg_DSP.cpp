/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector_Harris_IntImg.h"
#include "FeatureDetector.h"
#include "../arm_dsp/arm/HarrisCornerDetector_DSP.h"
#include "FeatureDetector_IntImg.h"
#include "../util/TimeMeasureBase.h"
#include "../util/Logger.h"
#include <Magick++.h>


FeatureDetectorHarrisIntImg::FeatureDetectorHarrisIntImg(unsigned int featuresThreshold, float nccThreshold, float harrisThreshold, float harrisK, float harrisDSigma, int harrisDKernelSize, float harrisGSigma, int harrisGKernelSize)
  : FeatureDetectorIntImg(featuresThreshold, nccThreshold)
{
  Logger::debug(Logger::NCC, "initializing integral image Harris NCC detector with features threshold %d\%, NCC threshold %f", featuresThreshold, nccThreshold);

  harrisThreshold_ = harrisThreshold;
  harrisK_ = harrisK;
  harrisDSigma_ = harrisDSigma;
  harrisDKernelSize_ = harrisDKernelSize;
  harrisGSigma_ = harrisGSigma;
  harrisGKernelSize_ = harrisGKernelSize;

  // find corners in image
#ifndef HARRIS_USE_DSP
  hcd_ = new HarrisCornerDetector(harrisThreshold_, harrisK_, harrisDSigma_, harrisDKernelSize_, harrisGSigma_, harrisGKernelSize_);
#else
  hcd_ = new HarrisCornerDetectorDSP(harrisThreshold_, harrisK_, harrisDSigma_, harrisDKernelSize_, harrisGSigma_, harrisGKernelSize_);
#endif

  Logger::debug(Logger::NCC, "initializing Harris Corner Detector for Harris-NCC with integral images");

  hcd_->init();
}


FeatureDetectorHarrisIntImg::~FeatureDetectorHarrisIntImg()
{
	cornerPoints_.clear();

	delete hcd_;
}


bool FeatureDetectorHarrisIntImg::match(ImageBitstream image)
{
  Logger::debug(Logger::NCC, "matching image (size %dx%d)", image.getWidth(), image.getHeight());

  Logger::debug(Logger::NCC, "searching for corners in image on ARM");

  // cleanup previously used corners
  cornerPoints_.clear();

  cornerPoints_ = hcd_->detectCorners(image);

  Logger::debug(Logger::NCC, "found %d corners in image", cornerPoints_.size());


  // begin matching
	unsigned int i, j;
	unsigned int matchCount = 0;
	unsigned int nFeatures = features_.size();
	unsigned int featuresToMatch = (featuresThreshold_ * nFeatures) / 100;

	// resize image for cross correlation
	unsigned int offset = FeatureDescriptor::patchSize_ / 2;
	ImageBitstream extendedImg = image.extend(offset);
	unsigned int extWidth = extendedImg.getWidth();
	unsigned int extHeight = extendedImg.getHeight();

	// calculate image parameters once per image
	Fixed *imageIntegral = new Fixed[(extWidth + 1) * (extHeight + 1)];
	Fixed *imageIntegral2 = new Fixed[(extWidth + 1) * (extHeight + 1)];
	Fixed *imageSqSum = new Fixed[image.getWidth() * image.getHeight()];
	Fixed *imageAvg = new Fixed[image.getWidth() * image.getHeight()];

	calculateImageData(extendedImg.getBitstream(), extendedImg.getWidth(), extendedImg.getHeight(), imageIntegral, imageIntegral2, imageSqSum, imageAvg);

	Logger::debug(Logger::NCC, "matching image");

	// calculate NCC for each feature
	for(i = 0; i < nFeatures; i++)
	{
	  startTimer("_ncc_match_single_arm");

	  for(j = 0; j < cornerPoints_.size(); j++)  // search for features in detected corners
    {
      if(getNCCResult(extendedImg.getBitstream(), extWidth, extHeight, cornerPoints_[i].getRow() + offset, cornerPoints_[i].getCol() + offset, featureData_[i], imageIntegral, imageIntegral2, imageSqSum, imageAvg))
        matchCount++;

      if(matchCount >= featuresToMatch)
        break;
    }

	  stopTimer("_ncc_match_single_arm");

	  if(matchCount >= featuresToMatch)
       break;
	}

	Logger::debug(Logger::NCC, "matched %d of %d features", matchCount, nFeatures);

	delete[] imageIntegral;
	delete[] imageIntegral2;
	delete[] imageSqSum;
	delete[] imageAvg;

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}


bool FeatureDetectorHarrisIntImg::getNCCResult(unsigned char *image, unsigned int width, unsigned int height, unsigned int row, unsigned int col, PatchData patchData, Fixed *imageIntegral, Fixed *imageIntegral2, Fixed *imageSqSum, Fixed *imageAvg)
{
	unsigned int prow, pcol;
	unsigned int irow, icol;

	unsigned int patchSize = FeatureDescriptor::patchSize_;


	// calculate patch parameters once per patch
	Fixed *patchNorm = patchData.patchNorm_;
	Fixed patchSqSum = patchData.patchSqSum_;

	// calculate NCC
	Fixed sumIP;
	Fixed ncc;

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
        //sumIP += patchNorm[prow * patchSize + pcol] * ((float)image[irow * width + icol] - imageAvg[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)]);
        sumIP += patchNorm[prow * patchSize + pcol] * scale_uchar(image[irow * width + icol]);
      }
    }

    ncc = sumIP / (patchSqSum * imageSqSum[(row - patchSize/2) * (width - patchSize) + (col - patchSize/2)]);
  }

  if(ncc >= nccThreshold_)
		return true;
	else
		return false;
}
