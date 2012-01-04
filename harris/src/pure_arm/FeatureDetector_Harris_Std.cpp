/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector_Harris_Std.h"
#include "HarrisCornerDetector.h"
#include "../util/HarrisCornerPoint.h"
#include "../util/FeatureGenerator.h"
#include "../util/FeatureDescriptor.h"
#include "../util/TimeMeasureBase.h"
#include "../util/Logger.h"
#include <Magick++.h>


FeatureDetectorHarrisStd::FeatureDetectorHarrisStd(unsigned int featuresThreshold, float nccThreshold, float harrisThreshold, float harrisK, float harrisDSigma, int harrisDKernelSize, float harrisGSigma, int harrisGKernelSize)
  : FeatureDetector(featuresThreshold, nccThreshold)
{
  Logger::debug(Logger::NCC, "initializing Harris NCC detector with features threshold %d\%, NCC threshold %f", featuresThreshold, nccThreshold);

	harrisThreshold_ = harrisThreshold;
	harrisK_ = harrisK;
	harrisDSigma_ = harrisDSigma;
	harrisDKernelSize_ = harrisDKernelSize;
	harrisGSigma_ = harrisGSigma;
	harrisGKernelSize_ = harrisGKernelSize;

	// find corners in image
  hcd_ = new HarrisCornerDetector(harrisThreshold_, harrisK_, harrisDSigma_, harrisDKernelSize_, harrisGSigma_, harrisGKernelSize_);

  Logger::debug(Logger::NCC, "initializing Harris Corner Detector for Harris-NCC");

  hcd_->init();
}


FeatureDetectorHarrisStd::~FeatureDetectorHarrisStd()
{
  cornerPoints_.clear();

  delete hcd_;
}


bool FeatureDetectorHarrisStd::match(ImageBitstream image)
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

	// calculate NCC for each feature
	for(i = 0; i < nFeatures; i++)
	{
	  for(j = 0; j < cornerPoints_.size(); j++)  // search for features in detected corners
	  {
      if(getNCCResult(extendedImg.getBitstream(), extWidth, extHeight, cornerPoints_[j].getCol() + offset, cornerPoints_[j].getRow() + offset, featureData_[i]))
        matchCount++;

      if(matchCount >= featuresToMatch)
        break;
	  }

	  if(matchCount >= featuresToMatch)
        break;
	}

	Logger::debug(Logger::NCC, "matched %d of %d features", matchCount, nFeatures);

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}

bool FeatureDetectorHarrisStd::getNCCResult(unsigned char *image, unsigned int width, unsigned int height, unsigned int row, unsigned int col, PatchData patchData)
{
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


  if(ncc >= nccThreshold_)
    return true;
  else
    return false;
}
