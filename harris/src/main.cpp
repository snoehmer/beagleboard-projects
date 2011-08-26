/*
 * main.cpp
 *
 *  Created on: 16.08.2011
 *      Author: sn
 */

#include <Magick++.h>
#include "pure_arm/ImageBitstream.h"
#include "pure_arm/HarrisCornerDetector.h"
#include "util/HarrisCornerPoint.h"

using namespace std;
using namespace Magick;

int main(int argc, char **argv)
{
    InitializeMagick(0);

    ImageBitstream inputImg("../samples/pic1.png");

    // perform Harris corner detection
    HarrisCornerDetector hcd(0.9f);
    vector<HarrisCornerPoint> cornerPoints;
    float *hcr;

    hcd.init();  // generates kernels

    cornerPoints = hcd.detectCorners(inputImg, &hcr);

    printf("found %d corners\n", cornerPoints.size());

    // mark corners in output image
    Image input = inputImg.getImage();
    input.strokeColor("red");

    for(int i = 0; i < cornerPoints.size(); i++)
    {
    	//printf("corner #%d at (%d, %d) with strength %f\n", i, cornerPoints[i].getCol(), cornerPoints[i].getRow(), cornerPoints[i].getStrength());
    	input.draw(DrawableCircle(cornerPoints[i].getCol(), cornerPoints[i].getRow(), cornerPoints[i].getCol() + 3, cornerPoints[i].getRow()));
    }

    // convert raw pixel data back to image
    input.write("../output/corners.png");

    return 0;
}
