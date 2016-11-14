#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include "datatype/sensordata/rangedata.h"
#include "datatype/geometry/point.h"



class Robot{
	public:
		Robot(){robPose.x=0; robPose.y=0; robPose.theta=0; rangeData = new mr::GenericRangeData;}
		~Robot(){};
		
		void setPose(double x, double y, double th){robPose=mr::Pose(x,y,th);}
		
		mr::GenericRangeData * rangeData;
		mr::Pose robPose;
		
		virtual bool updateRangeData() = 0;
};


#endif
