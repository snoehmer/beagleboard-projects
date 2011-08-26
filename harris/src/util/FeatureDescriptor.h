/*
 * FeatureDescriptor.h
 *
 *  Created on: 25.08.2011
 *      Author: sn
 */

#ifndef FEATUREDESCRIPTOR_H_
#define FEATUREDESCRIPTOR_H_

#include <vector>

class FeatureDescriptor
{
public:
	static const int patchSize = 16;

	FeatureDescriptor();
	FeatureDescriptor(unsigned char *f);
	FeatureDescriptor(unsigned char *bitstream, int row, int col);
	FeatureDescriptor(ImageBitstream source, int row, int col);
	~FeatureDescriptor();

	unsigned char* get();


private:
	unsigned char patch[patchSize];
};

#endif /* FEATUREDESCRIPTOR_H_ */
