/*
 * NonMaxSuppressor.cpp
 *
 *  Created on: 27.08.2011
 *      Author: sn
 */

#include "NonMaxSuppressor.h"
#include "ImageBitstream.h"
#include <cmath>


NonMaxSuppressor::NonMaxSuppressor()
{
	devKernelX_[0] = -1; devKernelX_[1] =  0; devKernelX_[2] =  1;
	devKernelX_[3] = -1; devKernelX_[4] =  0; devKernelX_[5] =  1;
	devKernelX_[6] = -1; devKernelX_[7] =  0; devKernelX_[8] =  1;

	devKernelY_[0] = -1; devKernelY_[1] = -1; devKernelY_[2] = -1;
	devKernelY_[3] =  0; devKernelY_[4] =  0; devKernelY_[5] =  0;
	devKernelY_[6] =  1; devKernelY_[7] =  1; devKernelY_[8] =  1;
}

NonMaxSuppressor::~NonMaxSuppressor()
{
}

float* NonMaxSuppressor::performNonMax(float *input, int width, int height)
{
	float sumX, sumY;
	float *diffX = new float[width * height];
	float *diffY = new float[width * height];
	float *magnitude = new float[width * height];
	float *output = new float[width * height];

	int offset = (devKernelSize_ - 1) / 2;
	int extWidth = width + 2 * offset;
	int extHeight = height + 2 * offset;

	int row, col, krow, kcol, imgrow, imgcol;

	float *extInput = ImageBitstream::extend(input, width, height, offset);

	// again, convolve HCR with derive to get edges
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

					sumX += extInput[row * extWidth + col] * devKernelX_[krow * devKernelSize_ + kcol];
					sumY += extInput[row * extWidth + col] * devKernelY_[krow * devKernelSize_ + kcol];
				}
			}

			diffX[(imgrow - offset) * width + (imgcol - offset)] = sumX;
			diffY[(imgrow - offset) * width + (imgcol - offset)] = sumY;
			magnitude[(imgrow - offset) * width + (imgcol - offset)] = sqrt(sumX * sumX + sumY * sumY);
		}
	}

	delete[] extInput;

	// now find maxima
	int irow, icol;
	float dX, dY, a1, a2, A, b1, b2, B, P;

	for(row = 1; row < height - 1; row++)
	{
		for(col = 1; col < width - 1; col++)
		{
			dX = diffX[row * width + col];
			dY = diffY[row * width + col];

			// set increments for different quadrants
			if(dX > 0) irow = 1;
			else irow = -1;

			if(dY > 0) icol = 1;
			else icol = -1;

			if(abs(dX) > abs(dY))
			{
				a1 = magnitude[(row) * width + (col + icol)];
				a2 = magnitude[(row - irow) * width + (col + icol)];
				b1 = magnitude[(row) * width + (col - icol)];
				b2 = magnitude[(row + irow) * width + (col - icol)];

				A = (abs(dX) - abs(dY)) * a1 + abs(dY) * a2;
				B = (abs(dX) - abs(dY)) * b1 + abs(dY) * b2;

				P = magnitude[row * width + col] * abs(dX);

				if(P >= A && P > B)
				{
					output[row * width + col] = abs(dX); //magnitude[row * width + col];
				}
				else
					output[row * width + col] = 0;
			}
			else
			{
				a1 = magnitude[(row - irow) * width + (col)];
				a2 = magnitude[(row - irow) * width + (col + icol)];
				b1 = magnitude[(row + irow) * width + (col)];
				b2 = magnitude[(row + irow) * width + (col - icol)];

				A = (abs(dY) - abs(dX)) * a1 + abs(dX) * a2;
				B = (abs(dY) - abs(dX)) * b1 + abs(dX) * b2;

				P = magnitude[row * width + col] * abs(dY);

				if(P >= A && P > B)
				{
					output[row * width + col] = abs(dY); //magnitude[row * width + col];
				}
				else
				{
					output[row * width + col] = 0;
				}
			}
		}
	}

	return output;
}
