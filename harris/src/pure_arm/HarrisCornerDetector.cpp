/*
 * HarrisCornerDetector.cpp
 *
 *  Created on: 18.08.2011
 *      Author: sn
 */

#include "HarrisCornerDetector.h"
#include "../util/HarrisCornerPoint.h"
#include <cmath>

using namespace std;

HarrisCornerDetector::HarrisCornerDetector(float threshold, float dSigma, int kernelSize, float gSigma, float k)
{
	devSigma_ = dSigma;
	kernelSize_ = kernelSize;
	gaussSigma_ = gSigma;
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
    int center = (kernelSize_ - 1) / 2;
    int xc2; // = (x - center)^2
    int yc2;
    float sigma2 = devSigma_ * devSigma_;
    float sigma2g = gaussSigma_ * gaussSigma_;

    float sumX = 0; // needed for normalization
    float sumY = 0;
    float sumGauss = 0;

    devKernelX_ = new float[kernelSize_ * kernelSize_];
    devKernelY_ = new float[kernelSize_ * kernelSize_];
    gaussKernel_ = new float[kernelSize_ * kernelSize_];


    // step 1: calculate derived Gauss function for kernels and the Gauss filter kernel
    for(row = 0; row < kernelSize_; row++)
    {
        for(col = 0; col < kernelSize_; col++)
        {
            xc2 = (col - center) * (col - center);
            yc2 = (row - center) * (row - center);

            devKernelX_[row * kernelSize_ + col] = -((float) col - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));

            devKernelY_[row * kernelSize_ + col] = -((float) row - center) * exp(((float) -(xc2 + yc2)) / (2 * sigma2));

            gaussKernel_[row * kernelSize_ + col] = exp(((float) -(xc2 + yc2)) / (2 * sigma2g));

            sumX += abs(devKernelX_[row * kernelSize_ + col]);
            sumY += abs(devKernelY_[row * kernelSize_ + col]);
            sumGauss += gaussKernel_[row * kernelSize_ + col];
        }
    }

    // step 2: normalize kernels
    for(row = 0; row < kernelSize_; row++)
    {
        for(col = 0; col < kernelSize_; col++)
        {
            devKernelX_[row * kernelSize_ + col] = devKernelX_[row * kernelSize_ + col] / sumX;
            devKernelY_[row * kernelSize_ + col] = devKernelY_[row * kernelSize_ + col] / sumY;
            gaussKernel_[row * kernelSize_ + col] = gaussKernel_[row * kernelSize_ + col] / sumGauss;
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

	int offset = (kernelSize_ - 1) / 2;

	int extWidth = width_ + 2 * offset;
	int extHeight = height_ + 2 * offset;

	ImageBitstream extendedImg = input_.extend((kernelSize_ - 1)/ 2);


	// step 1: convolve the image with the derives of Gaussians
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
			for(krow = 0; krow < kernelSize_; krow++)
			{
				for(kcol = 0; kcol < kernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extendedImg.pixel(row, col) * devKernelX_[krow * kernelSize_ + kcol];
					sumY += extendedImg.pixel(row, col) * devKernelY_[krow * kernelSize_ + kcol];
				}
			}

			diffXX[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumX;
			diffYY[(imgrow - offset) * width_ + (imgcol - offset)] = sumY * sumY;
			diffXY[(imgrow - offset) * width_ + (imgcol - offset)] = sumX * sumY;
		}
	}


	// step 2: apply Gaussian filters to convolved image
	float *extDiffXX = extendImage(diffXX, offset);
	float *extDiffYY = extendImage(diffYY, offset);
	float *extDiffXY = extendImage(diffXY, offset);

	for(imgrow = offset; imgrow < extHeight - offset; imgrow++)
	{
		for(imgcol = offset; imgcol < extWidth - offset; imgcol++)
		{
			sumX = 0;
			sumY = 0;
			sumXY = 0;

			// calculate weighted sum over kernel (convolution)
			for(krow = 0; krow < kernelSize_; krow++)
			{
				for(kcol = 0; kcol < kernelSize_; kcol++)
				{
					row = imgrow + krow - offset;
					col = imgcol + kcol - offset;

					sumX += extDiffXX[row * extWidth + col] * gaussKernel_[krow * kernelSize_ + kcol];
					sumY += extDiffYY[row * extWidth + col] * gaussKernel_[krow * kernelSize_ + kcol];
					sumXY += extDiffXY[row * extWidth + col] * gaussKernel_[krow * kernelSize_ + kcol];
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

	// step 5: normalize the image to a range 0...1 and threshold
	vector<HarrisCornerPoint> cornerPoints = normalizeAndThreshold(hcrIntern, width_ * height_, 1.0f, threshold_);


	// return HCR if user wants to, delete it otherwise
	if(*hcr)
		*hcr = hcrIntern;
	else
		delete[] hcrIntern;

	return cornerPoints;
}

float* HarrisCornerDetector::extendImage(float *input, int borderSize)
{
	int row;
	int col;
	float *extendedImg;

	int offset = borderSize;

	int extWidth = width_ + 2 * offset;
	int extHeight = height_ + 2 * offset;

	extendedImg = new float[extWidth * extHeight];

	// step 0: copy image
	for(row = 0; row < height_; row++)
			for(col = 0; col < width_; col++)
					extendedImg[(row + offset) * extWidth + (col + offset)] = input[row * width_ + col];

	// step 1a: copy upper border
	for(row = 0; row < offset; row++)
			for(col = 0; col < width_; col++)
					extendedImg[row * extWidth + (col + offset)] = input[0 * width_ + col];

	// step 1b: copy lower border
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = 0; col < width_; col++)
					extendedImg[row * extWidth + (col + offset)] = input[(height_ - 1) * width_ + col];

	// step 1c: copy left border
	for(col = 0; col < offset; col++)
			for(row = 0; row < height_; row++)
					extendedImg[(row + offset) * extWidth + col] = input[row * width_ + 0];

	// step 1d: copy right border
	for(col = offset + width_; col < width_ + 2*offset; col++)
			for(row = 0; row < height_; row++)
					extendedImg[(row + offset) * extWidth + col] = input[row * width_ + (width_ - 1)];

	// step 2a: copy upper left corner
	for(row = 0; row < offset; row++)
			for(col = 0; col < offset; col++)
					extendedImg[row * extWidth + col] = input[0 * width_ + 0];

	// step 2b: copy upper right corner
	for(row = 0; row < offset; row++)
			for(col = offset + width_; col < width_ + 2*offset; col++)
					extendedImg[row * extWidth + col] = input[0 * width_ + (width_ - 1)];

	// step 2c: copy lower left corner
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = 0; col < offset; col++)
					extendedImg[row * extWidth + col] = input[(height_ - 1) * width_ + 0];

	// step 2d: copy lower right corner
	for(row = offset + height_; row < height_ + 2*offset; row++)
			for(col = offset + width_; col < width_ + 2*offset; col++)
					extendedImg[row * extWidth + col] = input[(height_ - 1) * width_ + (width_ - 1)];

	return extendedImg;
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
