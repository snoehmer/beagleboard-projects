/*
 * HarrisCornerDetector.h
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

#ifndef HARRISCORNERDETECTOR_DSP_H_
#define HARRISCORNERDETECTOR_DSP_H_

#include "../../pure_arm/HarrisCornerDetector.h"
#include "../../pure_arm/ImageBitstream.h"
#include "../../util/HarrisCornerPoint.h"
#include <vector>
#include "../../pure_arm/FixedArithmetic.h"
#include "dspbridge/Dsp.h"

using namespace std;

class HarrisCornerDetectorDSP : public HarrisCornerDetector
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
    HarrisCornerDetectorDSP(float threshold = 0.8f, float k = 0.06f, float dSigma = 1.0f, int dKernelSize = 3, float gSigma = 2.0f, int gKernelSize = 5);

    virtual ~HarrisCornerDetectorDSP();

    /**
     * does all the initialization stuff (generating kernels etc)
     */
    void init();


private:

    Dsp *dspInstance_;
    DspNode *dspNode_;

    // kernels for convolution on DSP
    short *devKernel_gauss_;
    short *devKernel_gauss_der_;
    short *kernel_gauss_;
    short *nonMaxKernel_;
    short *nonMaxKernel1_;
    static const unsigned int nonMaxKernelSize_ = 3;

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
    vector<HarrisCornerPoint> performHarris(Fixed **hcr);

    void normalize(float *data, int n, float newMax = 1.0f);
    vector<HarrisCornerPoint> treshold(float *data, int n, float threshold);
    vector<HarrisCornerPoint> normalizeAndThreshold(float *data, int n, float newMax, float threshold);
    vector<HarrisCornerPoint> normalizeAndThreshold(Fixed *data, int n, Fixed newMax, Fixed threshold);
};

#endif /* HARRISCORNERDETECTOR_DSP_H_ */
