/***********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  -----------anyone
 *
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 **********************************************************************/

#ifndef __MRCORE__RANGEDATA__H
#define __MRCORE__RANGEDATA__H

#include <iostream>
#include <vector>
#include "../geometry/point.h"
#include "sensordata.h"
using namespace std;

namespace mr{

/*** This class corresponds to a single range reading **/

class GenericRangeReading : public Object{
	 DECLARE_MRL_OBJECT(GenericRangeReading)

public:
		GenericRangeReading(){};
		GenericRangeReading(string n, double r, Pose p = Pose(0,0,0), double maxR=5, double minR=0 ,  double f=0, double s=0){
			type=n; range=r; pose=p; fov=f; maxRange=maxR; minRange=minR; sigma=s;
			}
		virtual ~GenericRangeReading(){}

		string type;
		double range;
		double fov;
		double maxRange;//<the maximum range of the
		double minRange;
		double sigma; //< standard deviation of the range measurement
		Pose pose;

		virtual void drawGL2D() const;

};

/*** The range data is referenced to the following coordinates:
The origin is in the visual center of the sensor, righ handed system
The X axis is pointing front
The Y axis is in the scan plane
The Z axis points upwards
The bearings are rotations in the Z axis. **/


class GenericRangeData : public SensorData
{
	 DECLARE_MRL_OBJECT(GenericRangeData)
	///Text stream serializers for files and cout, cerr (std)
	friend ostream& operator << (ostream& os,const GenericRangeData& scan);

public:
	GenericRangeData();
	virtual ~GenericRangeData(){}

	/***
	 * Pushes a full set of GenericRangeData into the GenericRangeData
	**/
		void push(const GenericRangeData& r);
	/***
	 * Push a single reading into the GenericRange Data
	**/
	void pushReading(const GenericRangeReading& r){
		readings.push_back(r);
	}
	/***
	 * Gets a copy of a particular reading
	**/
	GenericRangeReading getReading(int i){return readings[i];}

	/***
	 * Clear all Readings
	**/

	void clear(){readings.clear();}

	/*** Drawing mode. 0=points, 1=solid, 2=contour **/
	int drawGLMode;
	virtual void drawGL2D() const;//<OpenGL draw

	//the data itself
	vector<GenericRangeReading> readings;

};


}



#endif
