/*
 * FeatureDescriptor.h
 *
 *  Created on: 25.08.2011
 *      Author: sn
 */

#ifndef FEATUREDESCRIPTOR_H_
#define FEATUREDESCRIPTOR_H_

class ImageBitstream;

class FeatureDescriptor
{
public:
	static const int patchSize_ = 16;

	FeatureDescriptor(unsigned char *f);
	FeatureDescriptor(unsigned char *bitstream, int centerrow, int centercol, int width, int height);
	FeatureDescriptor(ImageBitstream source, int centerrow, int centercol);

	unsigned char* get();


private:
	unsigned char patch_[patchSize_];

	void init(unsigned char *bitstream, int centerrow, int centercol, int width, int height);
};

#endif /* FEATUREDESCRIPTOR_H_ */
