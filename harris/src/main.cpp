/*
 * main.cpp
 *
 *  Created on: 16.08.2011
 *      Author: sn
 */

#include <Magick++.h>
#include "pure_arm/ImageBitstream.h"
#include "pure_arm/HarrisCornerDetector.h"
#include "pure_arm/FeatureDetector.h"
#include "pure_arm/FeatureDetector_IntImg.h"
#include "pure_arm/FeatureDetector_Harris_Std.h"
#include "pure_arm/FeatureDetector_Harris_IntImg.h"
#include "util/FeatureGenerator.h"
#include "util/FeatureDescriptor.h"
#include "util/HarrisCornerPoint.h"
#include "util/Logger.h"
#include "util/SystemTimeMeasure.h"
#include <vector>
#include <iostream>

//#define DEBUG_OUTPUT_CORNERS


//parameters for NCC matching
#define HARRIS_THRESH 0.7f  //threshold for Harris Corner Detector
#define NCC_STD_FEAT_THRESH 80  //threshold for matched features in percent for standard NCC
#define NCC_STD_NCC_THRESH 0.8f //threshold for NCC score for standard NCC
#define NCC_HARRIS_FEAT_THRESH 70  //threshold for matched features in percent for Harris-NCC
#define NCC_HARRIS_NCC_THRESH 0.7f //threshold for NCC score for Harris-NCC
#define NCC_HARRIS_HARRIS_THRESH 0.5f  //threshold for Harris Corner Detector when used in Harris-NCC


using namespace std;
using namespace Magick;


int main(int argc, char **argv)
{
    Logger::init();

    if(argc < 3)
    {
      cout << "usage: HarrisCornerDetector <reference image> <input image 1> [<output image 2> ...]" << endl;
      cout << "HCD searches for features in <reference image> und checks if they are contained in the input images" << endl << endl;
      Logger::error(Logger::MAIN, "usage: HarrisCornerDetector <reference image> <input image 1> [<output image 2> ...]");
      return -1;
    }

    InitializeMagick(0);

    ImageBitstream inputImg;

    try
    {
    	Logger::debug(Logger::MAIN, "reading reference image ('%s')", argv[1]);
    	inputImg = ImageBitstream(argv[1]);
    }
    catch(Exception &e)
    {
    	Logger::error(Logger::MAIN, "error reading reference image ('%s'), reason: %s", argv[1], e.what());
    	return -1;
    }

    /* ============================================================================
     * part 1: Harris Corner Detection
     */

    HarrisCornerDetector hcd(HARRIS_THRESH);
    vector<HarrisCornerPoint> cornerPoints;

    Logger::debug(Logger::MAIN, "initializing Harris Corner Detector on ARM");

    startTimer("harris_init_arm");
    hcd.init();  // generates kernels
    stopTimer("harris_init_arm");

    Logger::debug(Logger::MAIN, "searching for corners on ARM with threshold %f", HARRIS_THRESH);

    startTimer("harris_detect_arm");
    cornerPoints = hcd.detectCorners(inputImg);
    stopTimer("harris_detect_arm");

    Logger::debug(Logger::MAIN, "found %d corners", cornerPoints.size());



    /* ============================================================================
     * part 2: generate features from corners detected by HCD
     */

    FeatureGenerator featureGen;
    vector<FeatureDescriptor> features;

    Logger::debug(Logger::MAIN, "generating feature descriptors");

    features = featureGen.generateFeatures(inputImg, cornerPoints);


    /* ============================================================================
     * part 3: find features in other images
     */

    ImageBitstream currentImg;

    Logger::debug(Logger::MAIN, "initializing standard feature detector with feature threshold %d\%, NCC threshold %f", NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
    FeatureDetector *featureDet;

#ifndef USE_DSP
#if defined FEATDET_USE_NCCSTD
    featureDet = new FeatureDetector(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
#elif defined FEATDET_USE_NCCINTIMG
    featureDet = new FeatureDetectorIntImg(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
#elif defined FEATDET_USE_NCCHARRISSTD
    featureDet = new FeatureDetectorHarrisStd(NCC_HARRIS_FEAT_THRESH, NCC_HARRIS_NCC_THRESH, NCC_HARRIS_HARRIS_THRESH);
#elif defined FEATDET_USE_NCCHARRISINTIMG
    featureDet = new FeatureDetectorHarrisIntImg(NCC_HARRIS_FEAT_THRESH, NCC_HARRIS_NCC_THRESH, NCC_HARRIS_HARRIS_THRESH);
#else
#error You must choose a Feature Detector!
#endif
#else
#if defined FEATDET_USE_NCCSTD
    featureDet = new FeatureDetectorDSP(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
#elif defined FEATDET_USE_NCCINTIMG
    featureDet = new FeatureDetectorIntImgDSP(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
#elif defined FEATDET_USE_NCCHARRISSTD
    featureDet = new FeatureDetectorHarrisStdDSP(NCC_HARRIS_FEAT_THRESH, NCC_HARRIS_NCC_THRESH, NCC_HARRIS_HARRIS_THRESH);
#elif defined FEATDET_USE_NCCHARRISINTIMG
    featureDet = new FeatureDetectorHarrisIntImgDSP(NCC_HARRIS_FEAT_THRESH, NCC_HARRIS_NCC_THRESH, NCC_HARRIS_HARRIS_THRESH);
#else
#error You must choose a Feature Detector!
#endif
#endif

    Logger::debug(Logger::MAIN, "initializing features of feature detector");
    featureDet->setFeatures(features);

    for(int i = 2; i < argc; i++)
    {
    	Logger::debug(Logger::MAIN, "detecting features in file #%d ('%s')", i-1, argv[i]);

    	try
    	{
    		currentImg = ImageBitstream(argv[i]);
    	}
    	catch(Exception &e)
    	{
    		Logger::error(Logger::MAIN, "error detecting features in file #%d ('%s'), reason: %s", i-1, argv[i], e.what());
    		return -1;
    	}

    	startTimer("ncc_match_arm");
    	bool result = featureDet->match(currentImg);
    	stopTimer("ncc_match_arm");

    	if(result)
    	{
    		Logger::info(Logger::MAIN, "image #%d ('%s') is a match!", i-1, argv[i]);
    	}
    	else
    	{
    		Logger::info(Logger::MAIN, "image #%d ('%s') is no match!", i-1, argv[i]);
    	}
    }

    TimeMeasureBase::getInstance()->printStatistic();

    delete featureDet;


#ifdef DEBUG_OUTPUT_CORNERS
    // mark corners in output image
    Image input = inputImg.getImage();
    input.strokeColor("red");

    for(unsigned int i = 0; i < cornerPoints.size(); i++)
    {
    	Logger::debug(Logger::MAIN, "corner #%d at (%d,%d) with strength %f", i, cornerPoints[i].getCol(), cornerPoints[i].getRow(), cornerPoints[i].getStrength());
    	input.draw(DrawableCircle(cornerPoints[i].getCol(), cornerPoints[i].getRow(), cornerPoints[i].getCol() + 1, cornerPoints[i].getRow()));
    }

    // convert raw pixel data back to image
    input.write("./output/corners.png");
#endif

    Logger::cleanup();

    return 0;
}
