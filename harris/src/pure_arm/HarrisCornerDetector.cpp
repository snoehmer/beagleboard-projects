/*
 * HarrisCornerDetector.cpp
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

#include "HarrisCornerDetector.h"
#include "../util/HarrisCornerPoint.h"
#include "NonMaxSuppressor.h"
#include <cmath>
#include <Magick++.h>

using namespace std;

HarrisCornerDetector::HarrisCornerDetector(float threshold, float dSigma, int dKernelSize, float gSigma, int gKernelSize, float k)
{
	devSigma_ = dSigma;
	devKernelSize_ = dKernelSize;
	gaussSigma_ = gSigma;
	gaussKernelSize_ = gKernelSize;
	harrisK_ = k;
	threshold_ = threshold;

	devKernelX_ = 0;
	devKernelY_ = 0;
	gaussKernel_ = 0;
}

HarrisCornerDetector::~HarrisCornerDetector()
{
	if(devKernelX_)
		delete[] devKernelX_;

	if(devKernelY_)
		delete[] devKernelY_;

	if(gaussKernel_)
		delete[] gaussKernel_;
}

void HarrisCornerDetector::init()
{
    int row;
    int col;
    int center;
    int xc2; // = (x - center)^2
    int yc2;
    float sigma2 = devSigma_ * devSigma_;
    float sigma2g = gaussSigma_ * gaussSigma_;

    float sumX = 0; // needed for normalization
    float sumY = 0;
    float sumGauss = 0;

    devKernelX_ = new float[devKernelSize_ * devKernelSize_];
    devKernelY_ = new float[devKernelSize_ * devKernelSize_];
    gaussKernel_ = new float[gaussKernelSize_ * gaussKernelSize_];


    // step 1: calculate derived Gauss function for dev kernels
    center = (devKernelSize_ - 1) / 2;

    for(row = 0; row < devKernelSize_; row++)
    {
        for(col = 0; col < devKernelSize_; col++)
        {
            xc2 = (col - center) * (col - center);
            yc2 = (row - center) * (row - center);

            devKernelX_[row * devKernelSize_ + col] = -((float) col - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));
            devKernelY_[row * devKernelSize_ + col] = -((float) row - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));

            sumX += abs(devKernelX_[row * devKernelSize_ + col]);
            sumY += abs(devKernelY_[row * devKernelSize_ + col]);
        }
    }

    // step 2: normalize kernels
    for(row = 0; row < devKernelSize_; row++)
    {
        for(col = 0; col < devKernelSize_; col++)
        {
            devKernelX_[row * devKernelSize_ + col] = devKernelX_[row * devKernelSize_ + col] / sumX;
            devKernelY_[row * devKernelSize_ + col] = devKernelY_[row * devKernelSize_ + col] / sumY;
        }
    }


    // step 3: calculate Gauss function for gauss kernels
    center = (gaussKernelSize_ - 1) / 2;

    for(row = 0; row < gaussKernelSize_; row++)
    {
        for(col = 0; col < gaussKernelSize_; col++)
        {
            xc2 = (col - center) * (col - center);
            yc2 = (row - center) * (row - center);

            gaussKernel_[row * gaussKernelSize_ + col] = exp(((float) -(xc2 + yc2)) / (2 * sigma2g));

            sumGauss += gaussKernel_[row * gaussKernelSize_ + col];
        }
    }

    // step 2: normalize kernels
    for(row = 0; row < gaussKernelSize_; row++)
    {
        for(col = 0; col < gaussKernelSize_; col++)
        {
            gaussKernel_[row * gaussKernelSize_ + col] = gaussKernel_[row * gaussKernelSize_ + col] / sumGauss;
        }
    }
}


void HarrisCornerDetector::inputImage(ImageBitstream img)
{
    input_ = img;
    width_ = img.getWidth();
    height_ = img.getHeight();
}

vector<HarrisCornerPoint> HarrisCornerDetector::detectCorners(ImageBitstream img, float **hcr)
{
	if(!devKernelX_ || !devKernelY_ || !gaussKernel_)
		init();

	inputImage(img);

	return performHarris(hcr);
}

vector<HarrisCornerPoint> HarrisCornerDetector::performHarris(float **hcr)
{
	int imgrow;  // current row and col in the image where the filter is calculated
	int imgcol;
	int krow;  // current row and col in the kernel for the convolution sum
	int kcol;
	int row;  // row and col of image pixel for current kernel position
	int col;

	int offset;
	int extWidth;
	int extHeight;


	// step 1: convolve the image with the derives of Gaussians
	offset = (devKernelSize_ - 1) / 2;
	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	ImageBitstream extendedImg = input_.extend(offset);

	float *diffXX = new float[width_ * height_];
	float *diffYY = new float[width_ * height_];
	float *diffXY = new float[width_ * height_];

	float sumX;
	float sumY;
	float sumXY;

	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < devKernelSize_; krow++)
			{
				for(kcol = 0; kcol < devKernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extendedImg.pixel(row, col) * devKernelX_[krow * devKernelSize_ + kcol];
					sumY += extendedImg.pixel(row, col) * devKernelY_[krow * devKernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY * sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumY;
		}
	}


	// step 2: apply Gaussian filters to convolved image
	offset = (gaussKernelSize_ - 1) / 2;
	extWidth = width_ + 2 * offset;
	extHeight = height_ + 2 * offset;

	float *extDiffXX = ImageBitstream::extend(diffXX, width_, height_, offset);
	float *extDiffYY = ImageBitstream::extend(diffYY, width_, height_, offset);
	float *extDiffXY = ImageBitstream::extend(diffXY, width_, height_, offset);

	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;
			sumXY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < gaussKernelSize_; krow++)
			{
				for(kcol = 0; kcol < gaussKernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extDiffXX[row * extWidth + col] * gaussKernel_[krow * gaussKernelSize_ + kcol];
					sumY += extDiffYY[row * extWidth + col] * gaussKernel_[krow * gaussKernelSize_ + kcol];
					sumXY += extDiffXY[row * extWidth + col] * gaussKernel_[krow * gaussKernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumXY;
		}
	}

	delete[] extDiffXX;
	delete[] extDiffYY;
	delete[] extDiffXY;


	// step 3: calculate Harris corner response
	float *hcrIntern = new float[width_ * height_];
	float Ixx;
	float Iyy;
	float Ixy;

	for(row = 0; row < height_; row++)
	{
		for(col = 0; col < width_; col++)
		{
			Ixx = diffXX[row * width_ + col];
			Iyy = diffYY[row * width_ + col];
			Ixy = diffXY[row * width_ + col];

			hcrIntern[row * width_ + col] = Ixx * Iyy - Ixy * Ixy - harrisK_ * (Ixx + Iyy) * (Ixx + Iyy);
		}
	}

	delete[] diffXX;
	delete[] diffYY;
	delete[] diffXY;

/*
	// step 4: do non-maximum suppression
	float *diffX = new float[width_ * height_];
	float *diffY = new float[width_ * height_];
	float *magnitude = new float[width_ * height_];

	float *extHcr = extendImage(hcrIntern, offset);

	// again, convolve HCR with derived Gaussian to get edges
	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < kernelSize_; krow++)
			{
				for(kcol = 0; kcol < kernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extHcr[row * extWidth + col] * devKernelX_[krow * kernelSize_ + kcol];
					sumY += extHcr[row * extWidth + col] * devKernelY_[krow * kernelSize_ + kcol];
				}
			}

			diffX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX;
			diffY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY;
			magnitude[(imgrow - offset) * width_ + (imgcol - offset)] = sqrt(sumX * sumX + sumY * sumY);
		}
	}

	delete[] extHcr;


	// now find maxima
	bool sameSign;  // dX and dY have same sign
	int delta;
	float dX, dY, a1, a2, A, b1, b2, B, P;

	for(row = 1; row < height_ - 1; row++)
	{
		for(col = 1; col < width_ - 1; col++)
		{
			dX = diffX[row * width_ + col];
			dY = diffY[row * width_ + col];

			sameSign = ((dX > 0) && (dY > 0)) || ((dX < 0) && (dY<0));

			// set increments for different quadrants
			if(sameSign || dY == 0) delta = 1;
			else delta = -1;  // !sameSign || dX == 0

			if((abs(dX) > abs(dY)) || ((abs(dX) == abs(dY) && (!sameSign || dX == 0))))
			{
				a1 = magnitude[(row - 1) * width_ + (col)];
				a2 = magnitude[(row - 1) * width_ + (col + delta)];
				b1 = magnitude[(row + 1) * width_ + (col)];
				b2 = magnitude[(row + 1) * width_ + (col - delta)];

				A = (abs(dX) - abs(dY)) * a1 + abs(dY) * a2;
				B = (abs(dX) - abs(dY)) * b1 + abs(dY) * b2;

				P = magnitude[row * width_ + col] * abs(dX);
			}
			else  // abs(dX) < abs(dY) || (abs(dX) == abs(dY) && (sameSign || dY == 0))
			{
				a1 = magnitude[(row) * width_ + (col - 1)];
				a2 = magnitude[(row + delta) * width_ + (col - 1)];
				b1 = magnitude[(row) * width_ + (col + 1)];
				b2 = magnitude[(row - delta) * width_ + (col + 1)];

				A = (abs(dY) - abs(dX)) * a1 + abs(dX) * a2;
				B = (abs(dY) - abs(dX)) * b1 + abs(dX) * b2;

				P = magnitude[row * width_ + col] * abs(dY);
			}

			if(!(P > A && P > B))
			{
				hcrIntern[row * width_ + col] = 0;
			}
		}
	}
*/
/*
	// step 4: do non-maximum suppression
	float *diffX = new float[width_ * height_];
	float *diffY = new float[width_ * height_];
	float *magnitude = new float[width_ * height_];

	float *extHcr = ImageBitstream::extend(hcrIntern, width_, height_, offset);

	// again, convolve HCR with derived Gaussian to get edges
	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < kernelSize_; krow++)
			{
				for(kcol = 0; kcol < kernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extHcr[row * extWidth + col] * devKernelX_[krow * kernelSize_ + kcol];
					sumY += extHcr[row * extWidth + col] * devKernelY_[krow * kernelSize_ + kcol];
				}
			}

			diffX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX;
			diffY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY;
			magnitude[(imgrow - offset) * width_ + (imgcol - offset)] = sqrt(sumX * sumX + sumY * sumY);
		}
	}

	delete[] extHcr;

	// now find maxima
	int irow, icol;
	float dX, dY, a1, a2, A, b1, b2, B, P;

	for(row = 1; row < height_ - 1; row++)
	{
		for(col = 1; col < width_ - 1; col++)
		{
			dX = diffX[row * width_ + col];
			dY = diffY[row * width_ + col];

			// set increments for different quadrants
			if(dX > 0) irow = 1;
			else irow = -1;

			if(dY > 0) icol = 1;
			else icol = -1;

			if(abs(dX) > abs(dY))
			{
				a1 = magnitude[(row) * width_ + (col + icol)];
				a2 = magnitude[(row - irow) * width_ + (col + icol)];
				b1 = magnitude[(row) * width_ + (col - icol)];
				b2 = magnitude[(row + irow) * width_ + (col - icol)];

				A = (abs(dX) - abs(dY)) * a1 + abs(dY) * a2;
				B = (abs(dX) - abs(dY)) * b1 + abs(dY) * b2;

				P = magnitude[row * width_ + col] * abs(dX);

				if(P >= A && P > B)
				{
					hcrIntern[row * width_ + col] = abs(dX); //magnitude[row * width_ + col];
				}
				else
					hcrIntern[row * width_ + col] = 0;
			}
			else
			{
				a1 = magnitude[(row - irow) * width_ + (col)];
				a2 = magnitude[(row - irow) * width_ + (col + icol)];
				b1 = magnitude[(row + irow) * width_ + (col)];
				b2 = magnitude[(row + irow) * width_ + (col - icol)];

				A = (abs(dY) - abs(dX)) * a1 + abs(dX) * a2;
				B = (abs(dY) - abs(dX)) * b1 + abs(dX) * b2;

				P = magnitude[row * width_ + col] * abs(dY);

				if(P >= A && P > B)
				{
					hcrIntern[row * width_ + col] = abs(dY); //magnitude[row * width_ + col];
				}
				else
				{
					hcrIntern[row * width_ + col] = 0;
				}
			}
		}
	}
*/

	// step 4: perform non-maximum-suppression
	NonMaxSuppressor nonMax;
	float *hcrNonMax;

	hcrNonMax = nonMax.performNonMax(hcrIntern, width_, height_);

	delete[] hcrIntern;

	// step 5: normalize the image to a range 0...1 and threshold
	vector<HarrisCornerPoint> cornerPoints = normalizeAndThreshold(hcrNonMax, width_ * height_, 1.0f, threshold_);


	// return HCR if user wants to, delete it otherwise
	if(*hcr)
		*hcr = hcrNonMax;
	else
		delete[] hcrNonMax;

	return cornerPoints;
}


void HarrisCornerDetector::normalize(float *data, int n, float newMax)
{
	int i;
	float min, max;

	min = max = data[0];

	for(i = 0; i < n; i++)
	{
		if(data[i] > max) max = data[i];
		if(data[i] < min) min = data[i];
	}

	for(i = 0; i < n; i++)
	{
		data[i] = (data[i] - min) * newMax / (max - min);
	}
}

vector<HarrisCornerPoint> HarrisCornerDetector::treshold(float *data, int n, float threshold)
{
	int i;
	vector<HarrisCornerPoint> cornerPoints;

	for(i = 0; i < n; i++)
	{
		if(data[i] < threshold)
			data[i] = 0;
		else
			cornerPoints.push_back(HarrisCornerPoint(i / width_, i % width_, data[i]));
	}

	return cornerPoints;
}

vector<HarrisCornerPoint> HarrisCornerDetector::normalizeAndThreshold(float *data, int n, float newMax, float threshold)
{
	int i;
	float min, max;
	vector<HarrisCornerPoint> cornerPoints;

	min = max = data[0];

	for(i = 0; i < n; i++)
	{
		if(data[i] > max) max = data[i];
		if(data[i] < min) min = data[i];
	}

	for(i = 0; i < n; i++)
	{
		data[i] = (data[i] - min) * newMax / (max - min);

		if(data[i] < threshold)
			data[i] = 0;
		else
			cornerPoints.push_back(HarrisCornerPoint(i / width_, i % width_, data[i]));
	}

	return cornerPoints;
}
