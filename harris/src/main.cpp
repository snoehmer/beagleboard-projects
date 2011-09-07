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
#include <vector>
#include <iostream>

//#define DEBUG_OUTPUT_CORNERS

using namespace std;
using namespace Magick;

int main(int argc, char **argv)
{
	if(argc < 3)
	{
		cout << "usage: HarrisCornerDetector <reference image> <input image 1> [<output image 2> ...]" << endl;
		cout << "HCD searches for features in <reference image> und checks if they are contained in the input images" << endl << endl;
		return 0;
	}

    InitializeMagick(0);

    ImageBitstream inputImg;

    try
    {
    	cout << "reading reference image ('" << argv[1] << "')" << endl;
    	inputImg = ImageBitstream(argv[1]);
    }
    catch(Exception &e)
    {
    	cout << "Error reading reference image, reason: " << e.what() << endl;
    	return -1;
    }


    // perform Harris corner detection
    HarrisCornerDetector hcd(0.75f);
    vector<HarrisCornerPoint> cornerPoints;
    float *hcr;

    cout << "initializing Harris corner detector" << endl;
    hcd.init();  // generates kernels

    cout << "searching for corners" << endl;
    cornerPoints = hcd.detectCorners(inputImg, &hcr);

    cout << "found " << cornerPoints.size() << " corners" << endl;


    // generate features from corners
    FeatureGenerator featureGen;
    vector<FeatureDescriptor> features;

    cout << "generating feature descriptors" << endl;
    features = featureGen.generateFeatures(inputImg, cornerPoints);


    // detect features in images
    ImageBitstream currentImg;
    FeatureDetector featureDet(20, 0.2f);

    featureDet.setFeatures(features);

    for(int i = 2; i < argc; i++)
    {
    	cout << "detecting features in file #" << (i-1) << " ('" << argv[i] << "')" << endl;

    	try
    	{
    		currentImg = ImageBitstream(argv[i]);
    	}
    	catch(Exception &e)
    	{
    		cout << "Error: opening image failed, reason: " << e.what() << endl;
    		return -1;
    	}

    	if(featureDet.match(currentImg))
    		cout << "image #" << (i-1) << " ('" << argv[i] << "') is a match!" << endl;
    	else
    		cout << "image #" << (i-1) << " ('" << argv[i] << "') is no match!" << endl;
    }


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
