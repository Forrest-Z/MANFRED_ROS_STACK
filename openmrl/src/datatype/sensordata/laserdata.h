/**********************************************************************
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

#ifndef __MRL__LASERDATA__H
#define __MRL__LASERDATA__H

#include "sensordata.h"
#include "../geometry/angle.h"
#include "../geometry/vector2d.h"


#include <iostream>
#include <vector>

using namespace std;

namespace mr{

/** The laser data is referenced to the following coordinates:
The origin is in the visual center of the sensor, righ handed system
The X axis is pointing front
The Y axis is in the scan plane
The Z axis points upwards
The bearings are rotations in the Z axis. A typical 180º - 181 measurements 1 per degree 
will have bearings from -PI/2 to + PI/2.
The parametrization is the startAngle (-PI/2 in this case), the angular step (PI/180 in this example)
and the number of measurements **/

class LaserData : public SensorData
{
	 DECLARE_MRL_OBJECT(LaserData)
	///Text stream serializers for files and cout, cerr (std)
	friend ostream& operator << (ostream& os,const LaserData& scan);

public:
	LaserData();
	virtual ~LaserData(){}

	/** method to set the values of the scan
	\param ranges vector of range values (in m)
	\param start the start angle (in RAD)
	\param step the interval (in RAD)**/
	void setRanges(const vector<double>& ranges);
	
		
	/** method to set the properties of the scan**/
	void setProperties(double _startangle, double _step, int _numSteps, double _maxrange=10.0, double sigma=0);
	bool setRange(unsigned int index, double range);
	double getRange(unsigned int index);
	/** gets vector of angles **/
	vector<Angle>	getAngles();
	/** gets vector of ranges **/
	vector<double>	getRanges() const{return range;}
	/** gets the start Angle of the readings **/
	double getStartAngle() const{return startAngle;}
	/** gets the step */
	double getStep() const{return stepAngle;}
	void clear(){range.clear();}	
	int size()const {return range.size();}
	/** get a vector of 2D points references to the laser sensor
	it only computes the points once and keep them internally to avoid excessive computation (sines and cosines)*/
	vector<Vector2D> getPoints();

	/** Drawing mode. 0=points, 1=rays, 2=contour **/
	int drawGLMode;
	//int drawGLColor;
	virtual void drawGL2D() const;//<OpenGL draw
		
protected:
	//the data basic parametrization, compulsory. The angle evolves ccw (positive direction)
	double startAngle;//<the initial angle, in RAD
	double stepAngle;//<the interval, in RAD
	unsigned int numSteps;
	//the data itself
	vector<double> range;

	/** this vector is computed only the first time getPoints() is invoked**/
	vector<Vector2D> points;//<to avoid recomputing them again and again (for drawing for example)
	//Configuration parameters, could be factorized to a LaserConf class
	double maxRange;//<the maximum range of the
	double sigma; //< standard deviation of the range measurement

};


}



#endif
