/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector_Harris_IntImg_DSP.h"
#include "../../pure_arm/FeatureDetector_Harris_IntImg.h"
#include "HarrisCornerDetector_DSP.h"
#include "../../pure_arm/FeatureDetector_IntImg.h"
#include "../../util/TimeMeasureBase.h"
#include "../../util/Logger.h"
#include <Magick++.h>
#include "../common/harris_common.h"


FeatureDetectorHarrisIntImgDSP::FeatureDetectorHarrisIntImgDSP(unsigned int featuresThreshold, float nccThreshold, float harrisThreshold, float harrisK, float harrisDSigma, int harrisDKernelSize, float harrisGSigma, int harrisGKernelSize)
  : FeatureDetectorHarrisIntImg(featuresThreshold, nccThreshold, harrisThreshold, harrisK, harrisDSigma, harrisDKernelSize, harrisGSigma, harrisGKernelSize)
{
  Logger::debug(Logger::NCC, "initializing integral image Harris NCC detector with DSP support with features threshold %d\%, NCC threshold %f", featuresThreshold, nccThreshold);

  dspInstance_ = &(Dsp::Instance());
  dspNode_ = &(dspInstance_->GetNode());
}


FeatureDetectorHarrisIntImgDSP::~FeatureDetectorHarrisIntImgDSP()
{
  unsigned int i = 0;

  for(i = 0; i < features_.size(); i++)
  {
    dsp_free(featureData_[i].patchNormDSP_);
  }
}


bool FeatureDetectorHarrisIntImgDSP::match(ImageBitstream image)
{
  Logger::debug(Logger::NCC, "matching image (size %dx%d)", image.getWidth(), image.getHeight());

  Logger::debug(Logger::NCC, "searching for corners in image");

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

	// convert input image into right format for DSP (Q15)
  int *input_dsp = (int*) dsp_malloc(extWidth * extHeight * sizeof(int));
  if(!input_dsp)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP input array!");
    return false;
  }

  for(i = 0; i < extHeight; i++)
  {
    for(j = 0; j < extWidth; j++)
    {
      input_dsp[i * extWidth + j] = scale_uchar(extendedImg.pixel(i, j)).toIntQ15();
    }
  }

	// calculate image parameters once per image
	int *imageIntegral = (int*) dsp_malloc((extWidth + 1) * (extHeight + 1) * sizeof(int));
	if(!imageIntegral)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP imageIntegral array!");
    return false;
  }

	int *imageIntegral2 = (int*) dsp_malloc((extWidth + 1) * (extHeight + 1) * sizeof(int));
	if(!imageIntegral2)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP imageIntegral2 array!");
    return false;
  }

	int *imageSqSum = (int*) dsp_malloc(image.getWidth() * image.getHeight() * sizeof(int));
	if(!imageSqSum)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP imageSqSum array!");
    return false;
  }

	int *imageAvg = (int*) dsp_malloc(image.getWidth() * image.getHeight() * sizeof(int));
	if(!imageAvg)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP imageAvg array!");
    return false;
  }


	// calculate image data one per image
	calculateImageData(input_dsp, extendedImg.getWidth(), extendedImg.getHeight(), imageIntegral, imageIntegral2, imageSqSum, imageAvg);


	Logger::debug(Logger::NCC, "matching image");

	// calculate NCC for each feature
	for(i = 0; i < nFeatures; i++)
	{
	  startTimer("_ncc_match_single_dsp");

	  for(j = 0; j < cornerPoints_.size(); j++)  // search for features in detected corners
    {
      if(getNCCResult(input_dsp, extWidth, extHeight, cornerPoints_[i].getRow() + offset, cornerPoints_[i].getCol() + offset, featureData_[i], imageIntegral, imageIntegral2, imageSqSum, imageAvg))
        matchCount++;

      if(matchCount >= featuresToMatch)
        break;
    }

	  stopTimer("_ncc_match_single_dsp");

	  if(matchCount >= featuresToMatch)
       break;
	}

	Logger::debug(Logger::NCC, "matched %d of %d features", matchCount, nFeatures);

	dsp_free(input_dsp);
	dsp_free(imageIntegral);
	dsp_free(imageIntegral2);
	dsp_free(imageSqSum);
	dsp_free(imageAvg);

	if(matchCount >= featuresToMatch)
		return true;
	else
		return false;
}


bool FeatureDetectorHarrisIntImgDSP::getNCCResult(int *image, unsigned int width, unsigned int height, unsigned int row, unsigned int col, PatchData patchData, int *imageIntegral, int *imageIntegral2, int *imageSqSum, int *imageAvg)
{
	unsigned int patchSize = FeatureDescriptor::patchSize_;


	// calculate patch parameters once per patch
	int *patchNorm = patchData.patchNormDSP_;

	// calculate NCC
	Fixed ncc;


	// set parameters for integral image NCC calculation
  dsp_ncc_intimg_getncc_params *params = (dsp_ncc_intimg_getncc_params*) dsp_malloc(sizeof(dsp_ncc_intimg_getncc_params));
  if(!params)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP params!");
    return false;
  }

  params->input_ = (int*) dsp_get_mapped_addr(image);
  params->width_ = width;
  params->height_ = height;

  params->row_ = row;
  params->col_ = col;

  params->imageSqSum_ = (int*) dsp_get_mapped_addr(imageSqSum);

  params->patchNorm_ = (int*) dsp_get_mapped_addr(patchNorm);
  params->patchSqSum_ = patchData.patchSqSumDSP_;

  params->patchSize_ = patchSize;

  dsp_dmm_buffer_begin(image);
  dsp_dmm_buffer_begin(imageSqSum);
  dsp_dmm_buffer_begin(patchNorm);


  dspNode_->SendMessage(DSP_NCC_INTIMG_GETNCC, (uint32_t) dsp_get_mapped_addr(params), 0, 0);

  dsp_msg msg = dspNode_->GetMessage();


  dsp_dmm_buffer_end(image);
  dsp_dmm_buffer_end(imageSqSum);
  dsp_dmm_buffer_end(patchNorm);

  dsp_free(params);

  if(msg.cmd == DSP_NCC_INTIMG_GETNCC && msg.arg_1 == DSP_STATUS_FINISHED)
  {
    ncc = IntQ15toFixed(msg.arg_2);

    //Logger::debug(Logger::NCC, "DSP successfully calculated NCC, ncc=%d (%f)", ncc.toIntQ15(), ncc.toFloat());
  }
  else
  {
    Logger::error(Logger::NCC, "Error calculating NCC on DSP!");
    return false;
  }


  if(ncc >= Fixed(nccThreshold_))
		return true;
	else
		return false;
}


void FeatureDetectorHarrisIntImgDSP::calculateImageData(int *image, unsigned int width, unsigned int height, int *imageIntegral, int *imageIntegral2, int *imageSqSum, int *imageAvg)
{
  Logger::debug(Logger::NCC, "calculating image data (size %dx%d)", width, height);

  unsigned int patchSize = FeatureDescriptor::patchSize_;


  // set parameters for integral image NCC imagedata calculation
  dsp_ncc_intimg_imagedata_params *params = (dsp_ncc_intimg_imagedata_params*) dsp_malloc(sizeof(dsp_ncc_intimg_imagedata_params));
  if(!params)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP params!");
    return;
  }

  params->image_ = (int*) dsp_get_mapped_addr(image);
  params->width_ = width;
  params->height_ = height;

  params->imageIntegral_ = (int*) dsp_get_mapped_addr(imageIntegral);
  params->imageIntegral2_ = (int*) dsp_get_mapped_addr(imageIntegral2);
  params->imageAvg_ = (int*) dsp_get_mapped_addr(imageAvg);
  params->imageSqSum_ = (int*) dsp_get_mapped_addr(imageSqSum);

  params->patchSize_ = patchSize;

  dsp_dmm_buffer_begin(image);
  dsp_dmm_buffer_begin(imageIntegral);
  dsp_dmm_buffer_begin(imageIntegral2);
  dsp_dmm_buffer_begin(imageAvg);
  dsp_dmm_buffer_begin(imageSqSum);


  startTimer("_ncc_imagedata_single_dsp");

  dspNode_->SendMessage(DSP_NCC_INTIMG_IMAGEDATA, (uint32_t) dsp_get_mapped_addr(params), 0, 0);

  dsp_msg msg = dspNode_->GetMessage();

  stopTimer("_ncc_imagedata_single_dsp");


  dsp_dmm_buffer_end(image);
  dsp_dmm_buffer_end(imageIntegral);
  dsp_dmm_buffer_end(imageIntegral2);
  dsp_dmm_buffer_end(imageAvg);
  dsp_dmm_buffer_end(imageSqSum);

  dsp_free(params);


  if(msg.cmd == DSP_NCC_INTIMG_IMAGEDATA && msg.arg_1 == DSP_STATUS_FINISHED)
  {
    //Logger::debug(Logger::NCC, "DSP successfully calculated patch data: avg %d", *patchAvg);
  }
  else
  {
    Logger::error(Logger::NCC, "Error calculating image data on DSP!");
    return;
  }
}


PatchData FeatureDetectorHarrisIntImgDSP::calculatePatchData(unsigned char *patch)
{
  PatchData pd;

  unsigned int patchSize = FeatureDescriptor::patchSize_;


  int *patch_dsp = (int*) dsp_malloc(patchSize * patchSize * sizeof(int));
  if(!patch_dsp)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP patch array!");
    return pd;
  }

  // convert patch to DSP format (Q15)
  for(unsigned int i = 0; i < patchSize; i++)
    for(unsigned int j = 0; j < patchSize; j++)
      patch_dsp[i * patchSize + j] = scale_uchar(patch[i * patchSize + j]).toIntQ15();


  int *patchAvg = (int*) dsp_malloc(sizeof(int));
  if(!patchAvg)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP patchAvg!");
    return pd;
  }

  int *patchNorm = (int*) dsp_malloc(patchSize * patchSize * sizeof(int));
  if(!patchNorm)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP patchNorm array!");
    return pd;
  }

  int *patchSqSum = (int*) dsp_malloc(sizeof(int));
  if(!patchSqSum)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP patchSqSum!");
    return pd;
  }


  // set parameters for integral image NCC patchdata calculation
  dsp_ncc_intimg_patchdata_params *params = (dsp_ncc_intimg_patchdata_params*) dsp_malloc(sizeof(dsp_ncc_intimg_patchdata_params));
  if(!params)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP params!");
    return pd;
  }

  params->patch_ = (int*) dsp_get_mapped_addr(patch_dsp);

  params->patchAvg_ = (int*) dsp_get_mapped_addr(patchAvg);
  params->patchNorm_ = (int*) dsp_get_mapped_addr(patchNorm);
  params->patchSqSum_ = (int*) dsp_get_mapped_addr(patchSqSum);

  params->patchSize_ = patchSize;

  dsp_dmm_buffer_begin(patch_dsp);
  dsp_dmm_buffer_begin(patchAvg);
  dsp_dmm_buffer_begin(patchNorm);
  dsp_dmm_buffer_begin(patchSqSum);


  startTimer("_ncc_patchdata_single_dsp");

  dspNode_->SendMessage(DSP_NCC_INTIMG_PATCHDATA, (uint32_t) dsp_get_mapped_addr(params), 0, 0);

  dsp_msg msg = dspNode_->GetMessage();

  stopTimer("_ncc_patchdata_single_dsp");


  dsp_dmm_buffer_end(patch_dsp);
  dsp_dmm_buffer_end(patchAvg);
  dsp_dmm_buffer_end(patchNorm);
  dsp_dmm_buffer_end(patchSqSum);

  dsp_free(params);


  if(msg.cmd == DSP_NCC_INTIMG_PATCHDATA && msg.arg_1 == DSP_STATUS_FINISHED)
  {
    //Logger::debug(Logger::NCC, "DSP successfully calculated patch data: avg %d", *patchAvg);
  }
  else
  {
    Logger::error(Logger::NCC, "Error calculating patch data on DSP!");
    return pd;
  }


  dsp_free(patch_dsp);


  pd.patchAvgDSP_ = *patchAvg;
  pd.patchNormDSP_ = patchNorm;
  pd.patchSqSumDSP_ = *patchSqSum;
  pd.patchNorm_ = 0;
  pd.patchNormSq_ = 0;


  dsp_free(patchAvg);
  dsp_free(patchSqSum);

  return pd;
}
