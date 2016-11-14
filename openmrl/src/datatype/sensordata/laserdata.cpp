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

#include "laserdata.h"

#include <math.h>
#include <fstream>
#include <string>
#include <GL/glut.h>



namespace mr{

IMPLEMENT_MRL_OBJECT(LaserData)

LaserData::LaserData()
{
	drawGLMode=0;
	//drawGLColor=YELLOW;
	numSteps=0;
}

void LaserData::setRanges(const vector<double>& rang)
{
	if(numSteps!=rang.size())return;
	range=rang;
	points.clear();
}


void  LaserData::setProperties(double _startangle, double _step, int _numSteps, double _maxrange, double _sigma)
{
if(_numSteps<=0)_numSteps=1;
startAngle=_startangle;
stepAngle=_step; 
numSteps=_numSteps; 
maxRange=_maxrange; 
sigma=_sigma;
range.resize(numSteps);
}
bool LaserData::setRange(unsigned int index, double _range)
{
//if(index<0)return false;
if(index>=numSteps)return false;
if(_range>maxRange)_range=maxRange;
range[index]=_range;
points.clear();
return true;
}
double LaserData::getRange(unsigned int index)
{
if(index>=numSteps)return maxRange;
return range[index];
}

vector<Angle>	LaserData::getAngles()
{
	vector<Angle> angles;
	angles.resize(range.size());
	for(unsigned int i=0;i<numSteps;i++)
		angles[i]=Angle(startAngle+i*stepAngle);
	return angles;
}

vector<Vector2D> LaserData::getPoints()
{
	if(points.size()!=numSteps)
	{
		points.resize(numSteps);
		for(unsigned int i=0;i<numSteps;i++)
		{
			double ang=startAngle+i*stepAngle;
			points[i].x=range[i]*cos(ang);
			points[i].y=range[i]*sin(ang);
		}
	}	
	return points;
}


void LaserData::drawGL2D() const
{
	cout << "LaserData DrawGL2D not yet implemented" << endl;
	glColor3f(rgb[0],rgb[1],rgb[2]);
}

ostream& operator << (ostream& os,const  LaserData& scan)
{
	int size=(int)scan.numSteps;
	os<<"LASER "<<size<<" ";
	for(int i=0;i<size;i++)
	{
		os<<scan.range[i]<<" ";
	}
	os<<endl;
	return os;
}


}
