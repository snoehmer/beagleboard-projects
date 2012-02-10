/*
 * main.cpp
 *
 *  Created on: 16.08.2011
 *      Author: sn
 */

#include <Magick++.h>
#include "pure_arm/ImageBitstream.h"
#include "pure_arm/HarrisCornerDetector.h"
#include "arm_dsp/arm/HarrisCornerDetector_DSP.h"
#include "arm_dsp/arm/FeatureDetector_Harris_Std_DSP.h"
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

// include DSP stuff only if needed
#if defined(HARRIS_USE_DSP) || defined(FEATDET_USE_DSP)
#include "arm_dsp/arm/dspbridge/Dsp.h"
#include "arm_dsp/arm/dspbridge/dsp_bridge.h"
#endif
//#define DEBUG_OUTPUT_CORNERS


//parameters for NCC matching
#define HARRIS_THRESH 0.6f  //threshold for Harris Corner Detector
#define NCC_STD_FEAT_THRESH 80  //threshold for matched features in percent for standard NCC
#define NCC_STD_NCC_THRESH 0.8f //threshold for NCC score for standard NCC
#define NCC_HARRIS_FEAT_THRESH 80  //threshold for matched features in percent for Harris-NCC
#define NCC_HARRIS_NCC_THRESH 0.8f //threshold for NCC score for Harris-NCC
#define NCC_HARRIS_HARRIS_THRESH 0.6f  //threshold for Harris Corner Detector when used in Harris-NCC


using namespace std;
using namespace Magick;


int main(int argc, char **argv)
{
    Logger::init();

    if(argc < 2)
    {
      cout << "usage: HarrisCornerDetector <reference image> <input image 1> [<output image 2> ...]" << endl;
      cout << "HCD searches for features in <reference image> und checks if they are contained in the input images" << endl << endl;
      Logger::error(Logger::MAIN, "usage: HarrisCornerDetector <reference image> [<input image 1> <input image 2> ...]");
      return -1;
    }

    InitializeMagick(0);

    // initialize DSP: init DSP, load program and run node
#if defined(HARRIS_USE_DSP) || defined(FEATDET_USE_DSP)
    Logger::debug(Logger::MAIN, "initializing DSP: creating instance");

    Dsp *dsp = &(Dsp::Instance());

    Logger::debug(Logger::MAIN, "initializing DSP: running Init");
    dsp->Init();

    const struct dsp_uuid harris_uuid = { 0x3dac26d0, 0x6d4b, 0x11dd, 0xad, 0x8b, { 0x08, 0x00, 0x20, 0x0c, 0x9a, 0x66 } };

    Logger::debug(Logger::MAIN, "initializing DSP: loading node harris");
    DspNode& node = dsp->CreateNode(harris_uuid, "./harris.dll64P");

    Logger::debug(Logger::MAIN, "initializing DSP: running node harris");
    node.Run();
#endif

    ImageBitstream inputImg;

    try
    {
    	Logger::debug(Logger::MAIN, "reading reference image ('%s')", argv[1]);
    	inputImg = ImageBitstream(argv[1]);
    }
    catch(Magick::Exception &e)
    {
    	Logger::error(Logger::MAIN, "error reading reference image ('%s'), reason: %s", argv[1], e.what());
    	return -1;
    }

    /* ============================================================================
     * part 1: Harris Corner Detection
     */

    HarrisCornerDetector *hcd;
    vector<HarrisCornerPoint> cornerPoints;

#ifndef HARRIS_USE_DSP
    hcd = new HarrisCornerDetector(HARRIS_THRESH);
#else
    hcd = new HarrisCornerDetectorDSP(HARRIS_THRESH);
#endif

    Logger::debug(Logger::MAIN, "initializing Harris Corner Detector on ARM");

    startTimer("harris_init_arm");
    hcd->init();  // generates kernels
    stopTimer("harris_init_arm");

    Logger::debug(Logger::MAIN, "searching for corners on ARM with threshold %f", HARRIS_THRESH);

    startTimer("harris_detect_arm");
    cornerPoints = hcd->detectCorners(inputImg);
    stopTimer("harris_detect_arm");

    Logger::debug(Logger::MAIN, "found %d corners", cornerPoints.size());

    delete hcd;


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

#ifndef FEATDET_USE_DSP
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
    //featureDet = new FeatureDetectorDSP(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
    #error "non-Harris feature detectors are not implemented for DSP support!"
#elif defined FEATDET_USE_NCCINTIMG
    //featureDet = new FeatureDetectorIntImgDSP(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
    #error "non-Harris feature detectors are not implemented for DSP support!"
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
    	catch(Magick::Exception &e)
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
