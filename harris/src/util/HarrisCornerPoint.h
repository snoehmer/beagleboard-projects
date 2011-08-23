/*
 * HarrisCornerPoint.h
 *
 *  Created on: 19.08.2011
 *      Author: sn
 */

#ifndef HARRISCORNERPOINT_H_
#define HARRISCORNERPOINT_H_

class HarrisCornerPoint
{
public:

	HarrisCornerPoint();
	HarrisCornerPoint(int row, int col, float strength);

	void setCoordinates(int row, int col);
	void getCoordinates(int &row, int &col);
	void setStrength(float strength);
	float getStrength();
	int getCol();
	int getRow();

private:

	int row_;
	int col_;
	float strength_;

};
#endif /* HARRISCORNERPOINT_H_ */
