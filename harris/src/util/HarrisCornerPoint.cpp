/*
 * HarrisCornerPoint.cpp
 *
 *  Created on: 19.08.2011
 *      Author: sn
 */

#include "HarrisCornerPoint.h"

void HarrisCornerPoint::setStrength(float strength)
{
	strength_ = strength;
}

HarrisCornerPoint::HarrisCornerPoint(int row, int col, float strength)
{
	row_ = row;
	col_ = col;
	strength_ = strength;
}

void HarrisCornerPoint::setCoordinates(int row, int col)
{
	row_ = row;
	col_ = col;
}

float HarrisCornerPoint::getStrength()
{
	return strength_;
}

HarrisCornerPoint::HarrisCornerPoint()
{
	row_ = -1;
	col_ = -1;
	strength_ = 0.0f;
}

void HarrisCornerPoint::getCoordinates(int & row, int & col)
{
	row = row_;
	col = col_;
}

int HarrisCornerPoint::getCol()
{
	return col_;
}

int HarrisCornerPoint::getRow()
{
	return row_;
}
