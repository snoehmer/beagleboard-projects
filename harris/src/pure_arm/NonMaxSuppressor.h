/*
 * NonMaxSuppressor.h
 *
 *  Created on: 27.08.2011
 *      Author: sn
 */

#ifndef NONMAXSUPPRESSOR_H_
#define NONMAXSUPPRESSOR_H_

class NonMaxSuppressor
{
public:
	static const int devKernelSize_ = 3;

	NonMaxSuppressor();
	virtual ~NonMaxSuppressor();

	float* performNonMax(float *input, int width, int height);

private:
	int devKernelX_[devKernelSize_ * devKernelSize_];
	int devKernelY_[devKernelSize_ * devKernelSize_];

};

#endif /* NONMAXSUPPRESSOR_H_ */
