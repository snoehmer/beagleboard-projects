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
#include "util/FeatureGenerator.h"
#include "util/FeatureDescriptor.h"
#include "util/HarrisCornerPoint.h"
#include "util/Logger.h"
#include "util/SystemTimeMeasure.h"
#include <vector>
#include <iostream>

//#define DEBUG_OUTPUT_CORNERS

//parameters for NCC matching
#define NCC_STD_FEAT_THRESH 80  //threshold for matched features in percent for standard NCC
#define NCC_STD_NCC_THRESH 0.8f //threshold for NCC score for standard NCC

using namespace std;
using namespace Magick;

inline void startTimer(char *id)
{
  TimeMeasureBase::getInstance()->startTimer(id);
}

inline void stopTimer(char *id)
{
  TimeMeasureBase::getInstance()->stopTimer(id);
}


int main(int argc, char **argv)
{
	if(argc < 3)
	{
		//cout << "usage: HarrisCornerDetector <reference image> <input image 1> [<output image 2> ...]" << endl;
		//cout << "HCD searches for features in <reference image> und checks if they are contained in the input images" << endl << endl;
		Logger::error(Logger::MAIN, "usage: HarrisCornerDetector <reference image> <input image 1> [<output image 2> ...]");
	  return -1;
	}

    InitializeMagick(0);

    Logger::init();
    ImageBitstream inputImg;

    try
    {
    	//cout << "reading reference image ('" << argv[1] << "')" << endl;
    	Logger::debug(Logger::MAIN, "reading reference image ('%s')", argv[1]);
    	inputImg = ImageBitstream(argv[1]);
    }
    catch(Exception &e)
    {
    	//cout << "Error reading reference image, reason: " << e.what() << endl;
      Logger::error(Logger::MAIN, "error reading reference image ('%s'), reason: %s", argv[1], e.what());
    	return -1;
    }


    /* ============================================================================
     * part 1: Harris Corner Detection
     */

    /* ----------------------------------------------------------------------------
     * variant a: on ARM only
     */

    HarrisCornerDetector hcd(0.7f);
    vector<HarrisCornerPoint> cornerPoints;
    float *hcr;

    //cout << "initializing Harris corner detector" << endl;
    Logger::debug(Logger::MAIN, "initializing Harris Corner Detector on ARM");

    startTimer("harris_init_arm");
    hcd.init();  // generates kernels
    stopTimer("harris_init_arm");

    //cout << "searching for corners" << endl;
    Logger::debug(Logger::MAIN, "searching for corners on ARM");

    startTimer("harris_detect_arm");
    cornerPoints = hcd.detectCorners(inputImg, &hcr);
    stopTimer("harris_detect_arm");

    //cout << "found " << cornerPoints.size() << " corners" << endl;
    Logger::debug(Logger::MAIN, "found %d corners", cornerPoints.size());



    /* ============================================================================
     * part 2: generate features from corners detected by HCD
     */

    FeatureGenerator featureGen;
    vector<FeatureDescriptor> features;

    //cout << "generating feature descriptors" << endl;
    Logger::debug(Logger::MAIN, "generating feature descriptors");

    features = featureGen.generateFeatures(inputImg, cornerPoints);


    /* ============================================================================
     * part 3: find features in other images
     */

    /* ----------------------------------------------------------------------------
     * variant a: on ARM only
     */

    ImageBitstream currentImg;

    Logger::debug(Logger::MAIN, "initializing standard feature detector with feature threshold $d\%, NCC threshold %f", NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);
    FeatureDetector featureDet(NCC_STD_FEAT_THRESH, NCC_STD_NCC_THRESH);

    Logger::debug(Logger::MAIN, "initializing features of feature detector");
    featureDet.setFeatures(features);

    for(int i = 2; i < argc; i++)
    {
    	//cout << "detecting features in file #" << (i-1) << " ('" << argv[i] << "')" << endl;
      Logger::debug(Logger::MAIN, "detecting features in file # %d ('%s')", i-1, argv[i]);

    	try
    	{
    		currentImg = ImageBitstream(argv[i]);
    	}
    	catch(Exception &e)
    	{
    		//cout << "Error: opening image failed, reason: " << e.what() << endl;
    	  Logger::error(Logger::MAIN, "error detecting features in file # %d ('%s'), reason: %s", i-1, argv[i], e.what());
    		return -1;
    	}

    	startTimer("ncc_std_match_arm");
    	bool result = featureDet.match(currentImg);
    	stopTimer("ncc_std_match_arm");

    	if(result)
    	{
    		//cout << "image #" << (i-1) << " ('" << argv[i] << "') is a match!" << endl;
    		Logger::info(Logger::MAIN, "image # %d ('%s') is a match!", i-1, argv[i]);
    	}
    	else
    	{
    		//cout << "image #" << (i-1) << " ('" << argv[i] << "') is no match!" << endl;
    	  Logger::info(Logger::MAIN, "image # %d ('%s') is no match!", i-1, argv[i]);
    	}
    }

    TimeMeasureBase::getInstance()->printStatistic();

#ifdef DEBUG_OUTPUT_CORNERS
    // mark corners in output image
    Image input = inputImg.getImage();
    input.strokeColor("red");

    for(unsigned int i = 0; i < cornerPoints.size(); i++)
    {
    	cout << "corner # " << i << " at (" << cornerPoints[i].getCol() << "," << cornerPoints[i].getRow() << ") with strength " << cornerPoints[i].getStrength() << endl;
    	input.draw(DrawableCircle(cornerPoints[i].getCol(), cornerPoints[i].getRow(), cornerPoints[i].getCol() + 1, cornerPoints[i].getRow()));
    }

    // convert raw pixel data back to image
    input.write("../output/corners.png");
#endif

    return 0;
}
