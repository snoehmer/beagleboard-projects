/*
 * GaussFilter.h
 *
 *  Created on: 16.08.2011
 *      Author: sn
 */

#ifndef GAUSSFILTER_H_
#define GAUSSFILTER_H_

#include "ImageBitstream.h"

class GaussFilter
{
public:

	/**
	 * constructor for the Gaussian filter
	 * @param kernelSize size of the filter kernel (should be odd)
	 * @param sigma the sigma parameter of the Gauss filter
	 */
	GaussFilter(int kernelSize = 3, float sigma = 0.5f);

	/**
	 * constructor for the Gaussian filter
	 * @param original the original image
	 * @param width the width of the image
	 * @param height the height of the image
	 * @param kernelSize size of the filter kernel (should be odd)
	 * @param sigma the sigma parameter of the Gauss filter
	 */
	GaussFilter(ImageBitstream original, int kernelSize = 3, float sigma = 0.2f);

	virtual ~GaussFilter();

	/**
	 * generates the Gaussian kernel for the specified kernel size and sigma
	 * has to be called only once!
	 */
	void generateKernel();

	/**
	 * reads a new image for filtering
	 * the old image is deleted
	 * @param image the new image as raw array
	 * @param width the width of the image
	 * @param height the height of the image
	 */
	void inputImage(ImageBitstream input);

	/**
	 * applies the Gaussian filter and returns the filtered image
	 * first extends the image for convolution, then performs convolution
	 * with Gauss kernel, then crops the unneeded borders from the image
	 * stores the result internally until the user calls getFilteredImage!
	 */
	ImageBitstream calculate();

	/**
	 * performs a full gaussian filtering on the image and generates the
	 * resulting image
	 * this function just calls all the other functions (except generateKernel!)
	 * @param image the image to be filtered
	 * @param output the buffer for the resulting image (is newly allocated)
	 * @param width the width of the image
	 * @param height the height of the image
	 */
	ImageBitstream filterImage(ImageBitstream input);

private:
	ImageBitstream input_;
	float *kernel_;
	float sigma_;
	int kernelSize_;

};

#endif /* GAUSSFILTER_H_ */
