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

#include "rangedata.h"

#include <math.h>
#include <fstream>
#include <string>

#include <GL/glut.h>

namespace mr{


IMPLEMENT_MRL_OBJECT(GenericRangeReading)
IMPLEMENT_MRL_OBJECT(GenericRangeData)

void GenericRangeReading::drawGL2D() const{

	 //compute coordinates of the sensor cone
	 Point2 vertex0, vertex1, vertex2;
	 vertex0.x=pose.x; vertex0.y=pose.y;

	 Vector2D mid_vector;
	 mid_vector.setPolarCoords(range,pose.theta);

	 vertex1 = vertex0 + mid_vector.rot(-fov/2);
	 vertex2 = vertex0 + mid_vector.rot(fov/2);

	 glEnable (GL_BLEND);
	 glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	 glPushMatrix();

	 glBegin (GL_TRIANGLES);
			 glColor4f(rgb[0],rgb[1], rgb[2], 0.5);
			 glVertex2f(vertex0.x, vertex0.y);
			 glVertex2f(vertex1.x,vertex1.y);
			 glVertex2f(vertex2.x, vertex2.y);
		 glEnd();

	 glPopMatrix();

	 glDisable(GL_BLEND);
}

GenericRangeData::GenericRangeData()
{
	drawGLMode=0;
	readings.resize(0);
}


void GenericRangeData::push(const GenericRangeData& r){
	for (unsigned int i=0; i<r.readings.size(); i++){
		pushReading(r.readings[i]);
	}
}


//TODO
void GenericRangeData::drawGL2D() const
{
	 glPushMatrix();
	 for (int i=0; i< readings.size(); i++){
		  //readings[i].setRGB(rgb[0],rgb[1],rgb[2]);
		  readings[i].drawGL2D();
	 }
	 glPopMatrix();
}

ostream& operator << (ostream& os,const  GenericRangeData& scan)
{
	int size= (int)scan.readings.size();
	os<<"SONAR "<<size<<" ";
	for(int i=0;i<size;i++)
	{
		os <<scan.readings[i].type<<" "<<scan.readings[i].fov<<" "<<scan.readings[i].maxRange<<" "<<scan.readings[i].minRange<<" "<<scan.readings[i].sigma<<" "<<scan.readings[i].range<<" "<<scan.readings[i].pose.x<<" "<<scan.readings[i].pose.y<<" "<<scan.readings[i].pose.theta<<" ";
		os<<endl;
	}

	return os;
}

} //end namespace
