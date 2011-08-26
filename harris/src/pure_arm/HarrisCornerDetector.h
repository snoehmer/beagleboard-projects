/*
 * HarrisCornerDetector.h
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

#ifndef HARRISCORNERDETECTOR_H_
#define HARRISCORNERDETECTOR_H_

#include "ImageBitstream.h"
#include "../util/HarrisCornerPoint.h"
#include <vector>

using namespace std;

class HarrisCornerDetector
{
public:

    /**
     * constructor for HarrisCornerDetector
     * @param dSigma sigma value for the derives
     * @param sKernelSize kernel size for convolution of derives
     * @param gSigma sigma value for Gauss filter
     * @param gKernelSize kernel size for convolution of Gauss filter
     * @param k parameter for Harris detector
     * @param thresh threshold for the corner strength
     */
    HarrisCornerDetector(float threshold = 1.0f, float dSigma = 1.0f, int kernelSize = 5, float gSigma = 2.0f, float k = 0.04f);

    virtual ~HarrisCornerDetector();

    /**
     * does all the initialization stuff (generating kernels etc)
     */
    void init();

    /**
     * sets the (new) input image for the corner detector
     * @param input a raw bit stream of the input image
     * @param width the width of the input image
     * @param height the height of the input image
     */
    void inputImage(ImageBitstream img);

    /**
     * performs a full Harris corner detection
     * @param input the input image
     * @param width the width of the input image
     * @param height the height of the input image
     * @param cornerList a list of the detected corners
     * @param corners a image with the corner strength
     */
    vector<HarrisCornerPoint> detectCorners(ImageBitstream img, float **hcr = 0);


private:

    float devSigma_;
    int kernelSize_;
    float gaussSigma_;
    float harrisK_;
    float threshold_;
    ImageBitstream input_;
    float *devKernelX_;
    float *devKernelY_;
    float *gaussKernel_;
    int width_;
    int height_;


    /**
     * extends the image beyond its borders for the convolution
     * every pixel on the border is copied outside, so that the image is
     * extended by (kernelSize - 1)/2 pixels on each border
     * @returns the buffer containing the extended result
     */
    float* extendImage(float *input, int borderSize);

    /**
     * performs the convolution with both derived kernels for every pixel
     * calculates the products of derives for every pixel
     * applies a Gauss filter on the products
     * calculates the Harris corner response for every pixel
     * applies a threshold to the Harris corner response
     * the results are cropped and stored in output
     * @param extendedImg source for image to derive
     * @param threshold the threshold for the corner strength
     * @param hcr a raw bit stream of the harris corner response
     * @param cornerStrength the thresholded corner strength
     */
    vector<HarrisCornerPoint> performHarris(float **hcr);

    void normalize(float *data, int n, float newMax = 1.0f);
    vector<HarrisCornerPoint> treshold(float *data, int n, float threshold);
    vector<HarrisCornerPoint> normalizeAndThreshold(float *data, int n, float newMax, float threshold);
};

#endif /* HARRISCORNERDETECTOR_H_ */
