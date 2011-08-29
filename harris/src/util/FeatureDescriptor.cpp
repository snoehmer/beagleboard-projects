/*
 * FeatureDescriptor.cpp
 *
 *  Created on: 27.08.2011
 *      Author: sn
 */

#include "FeatureDescriptor.h"
#include "../pure_arm/ImageBitstream.h"
#include "HarrisCornerPoint.h"
#include <cstring>


FeatureDescriptor::FeatureDescriptor()
{
}


FeatureDescriptor::FeatureDescriptor(unsigned char *f)
{
	memcpy(&patch_, f, sizeof(patchSize_ * patchSize_ * sizeof(unsigned char)));
}


FeatureDescriptor::FeatureDescriptor(unsigned char *bitstream, int centerrow, int centercol, int width, int height)
{
	init(bitstream, centerrow, centercol, width, height);
}


FeatureDescriptor::FeatureDescriptor(ImageBitstream source, int centerrow, int centercol)
{
	init(source.getBitstream(), centerrow, centercol, source.getWidth(), source.getHeight());
}


FeatureDescriptor::FeatureDescriptor(ImageBitstream source, HarrisCornerPoint center)
{
	init(source.getBitstream(), center.getRow(), center.getCol(), source.getWidth(), source.getHeight());
}


FeatureDescriptor::FeatureDescriptor(unsigned char *bitstream, HarrisCornerPoint center, int width, int height)
{
	init(bitstream, center.getRow(), center.getCol(), width, height);
}


unsigned char* FeatureDescriptor::get()
{
	unsigned char *patch;

	memcpy(&patch, &patch_, sizeof(patchSize_ * patchSize_ * sizeof(unsigned char)));

	return patch;
}

void FeatureDescriptor::init(unsigned char *bitstream, int centerrow, int centercol, int width, int height)
{
	int row, col;
	int imagerow, imagecol;

	unsigned char *extBitstream = ImageBitstream::extend(bitstream, width, height, patchSize_/2);

	for(row = 0; row < patchSize_; row++)
	{
		for(col = 0; col < patchSize_; col++)
		{
			imagerow = centerrow - (patchSize_ - 1)/2 + row + patchSize_/2;
			imagecol = centercol - (patchSize_ - 1)/2 + col + patchSize_/2;

			patch_[row * patchSize_ + col] = extBitstream[imagerow * (width + patchSize_) + imagecol];
		}
	}

	delete[] extBitstream;
}
