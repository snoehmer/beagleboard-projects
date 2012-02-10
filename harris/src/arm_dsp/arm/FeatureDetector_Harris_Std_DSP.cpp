/*
 * FeatureDetector.cpp
 *
 *  Created on: 29.08.2011
 *      Author: sn
 */

#include "FeatureDetector_Harris_Std_DSP.h"
#include "../../pure_arm/FeatureDetector_Harris_Std.h"
#include "../../pure_arm/HarrisCornerDetector.h"
#include "HarrisCornerDetector_DSP.h"
#include "../../util/HarrisCornerPoint.h"
#include "../../util/FeatureGenerator.h"
#include "../../util/FeatureDescriptor.h"
#include "../../util/TimeMeasureBase.h"
#include "../../util/Logger.h"
#include <Magick++.h>
#include "../common/harris_common.h"


FeatureDetectorHarrisStdDSP::FeatureDetectorHarrisStdDSP(unsigned int featuresThreshold, float nccThreshold, float harrisThreshold, float harrisK, float harrisDSigma, int harrisDKernelSize, float harrisGSigma, int harrisGKernelSize)
  : FeatureDetectorHarrisStd(featuresThreshold, nccThreshold, harrisThreshold, harrisK, harrisDSigma, harrisDKernelSize, harrisGSigma, harrisGKernelSize)
{
  Logger::debug(Logger::NCC, "initializing Harris NCC detector with DSP support with features threshold %d\%, NCC threshold %f", featuresThreshold, nccThreshold);

  dspInstance_ = &(Dsp::Instance());
  dspNode_ = &(dspInstance_->GetNode());
}


FeatureDetectorHarrisStdDSP::~FeatureDetectorHarrisStdDSP()
{
  Logger::debug(Logger::NCC, "destroying DSP data");

  unsigned int i = 0;

  for(i = 0; i < features_.size(); i++)
  {
    dsp_free(featureData_[i].patchNormDSP_);
    dsp_free(featureData_[i].patchNormSqDSP_);
  }
}

bool FeatureDetectorHarrisStdDSP::match(ImageBitstream image)
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

  // calculate NCC for each feature
  for(i = 0; i < nFeatures; i++)
  {
    startTimer("_ncc_match_single_dsp");

    for(j = 0; j < cornerPoints_.size(); j++)  // search for features in detected corners
    {
      if(getNCCResult(input_dsp, extWidth, extHeight, cornerPoints_[j].getRow() + offset, cornerPoints_[j].getCol() + offset, featureData_[i]))
        matchCount++;

      if(matchCount >= featuresToMatch)
        break;
    }

    stopTimer("_ncc_match_single_dsp");

    if(matchCount >= featuresToMatch)
       break;
  }

  dsp_free(input_dsp);

  Logger::debug(Logger::NCC, "matched %d of %d features", matchCount, nFeatures);

  if(matchCount >= featuresToMatch)
    return true;
  else
    return false;
}


bool FeatureDetectorHarrisStdDSP::getNCCResult(int *image, unsigned int width, unsigned int height, unsigned int row, unsigned int col, PatchData patchData)
{
  //Logger::debug(Logger::NCC, "calculating NCC on DSP");

  unsigned int patchSize = FeatureDescriptor::patchSize_;

  int *patchNorm = patchData.patchNormDSP_;
  int *patchNormSq = patchData.patchNormSqDSP_;

  // calculate NCC
  Fixed ncc;


  // set parameters for standard NCC calculation
  dsp_ncc_std_getncc_params *params = (dsp_ncc_std_getncc_params*) dsp_malloc(sizeof(dsp_ncc_std_getncc_params));
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

  params->patchAvg_ = patchData.patchAvgDSP_;
  params->patchNorm_ = (int*) dsp_get_mapped_addr(patchNorm);
  params->patchNormSq_ = (int*) dsp_get_mapped_addr(patchNormSq);
  params->patchSqSum_ = patchData.patchSqSumDSP_;

  params->patchSize_ = patchSize;

  dsp_dmm_buffer_begin(image);
  dsp_dmm_buffer_begin(patchNorm);
  dsp_dmm_buffer_begin(patchNormSq);


  dspNode_->SendMessage(DSP_NCC_STD_GETNCC, (uint32_t) dsp_get_mapped_addr(params), 0, 0);

  dsp_msg msg = dspNode_->GetMessage();


  dsp_dmm_buffer_end(image);
  dsp_dmm_buffer_end(patchNorm);
  dsp_dmm_buffer_end(patchNormSq);

  dsp_free(params);

  if(msg.cmd == DSP_NCC_STD_GETNCC && msg.arg_1 == DSP_STATUS_FINISHED)
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

PatchData FeatureDetectorHarrisStdDSP::calculatePatchData(unsigned char *patch)
{
  //Logger::debug(Logger::NCC, "calculating patch data on DSP");
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

  int *patchNormSq = (int*) dsp_malloc(patchSize * patchSize * sizeof(int));
  if(!patchNormSq)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP patchNormSq array!");
    return pd;
  }

  int *patchSqSum = (int*) dsp_malloc(sizeof(int));
  if(!patchSqSum)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP patchSqSum!");
    return pd;
  }


  startTimer("_ncc_patchdata_single_dsp");


  // set parameters for standard NCC patchdata calculation
  dsp_ncc_std_patchdata_params *params = (dsp_ncc_std_patchdata_params*) dsp_malloc(sizeof(dsp_ncc_std_patchdata_params));
  if(!params)
  {
    Logger::error(Logger::NCC, "failed to allocate memory for DSP params!");
    return pd;
  }

  params->patch_ = (int*) dsp_get_mapped_addr(patch_dsp);

  params->patchAvg_ = (int*) dsp_get_mapped_addr(patchAvg);
  params->patchNorm_ = (int*) dsp_get_mapped_addr(patchNorm);
  params->patchNormSq_ = (int*) dsp_get_mapped_addr(patchNormSq);
  params->patchSqSum_ = (int*) dsp_get_mapped_addr(patchSqSum);;

  params->patchSize_ = patchSize;

  dsp_dmm_buffer_begin(patch_dsp);
  dsp_dmm_buffer_begin(patchAvg);
  dsp_dmm_buffer_begin(patchNorm);
  dsp_dmm_buffer_begin(patchNormSq);
  dsp_dmm_buffer_begin(patchSqSum);


  dspNode_->SendMessage(DSP_NCC_STD_PATCHDATA, (uint32_t) dsp_get_mapped_addr(params), 0, 0);

  dsp_msg msg = dspNode_->GetMessage();


  dsp_dmm_buffer_end(patch_dsp);
  dsp_dmm_buffer_end(patchAvg);
  dsp_dmm_buffer_end(patchNorm);
  dsp_dmm_buffer_end(patchNormSq);
  dsp_dmm_buffer_end(patchSqSum);

  dsp_free(params);


  if(msg.cmd == DSP_NCC_STD_PATCHDATA && msg.arg_1 == DSP_STATUS_FINISHED)
  {
    //Logger::debug(Logger::NCC, "DSP successfully calculated patch data: avg %d", *patchAvg);
  }
  else
  {
    Logger::error(Logger::NCC, "Error calculating patch data on DSP!");
    return pd;
  }


  stopTimer("_ncc_patchdata_single_dsp");

  dsp_free(patch_dsp);


  pd.patchAvgDSP_ = *patchAvg;
  pd.patchNormDSP_ = patchNorm;
  pd.patchNormSqDSP_ = patchNormSq;
  pd.patchSqSumDSP_ = *patchSqSum;
  pd.patchNorm_ = 0;
  pd.patchNormSq_ = 0;


  dsp_free(patchAvg);
  dsp_free(patchSqSum);

  return pd;
}
