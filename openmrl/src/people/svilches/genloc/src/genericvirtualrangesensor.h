#ifndef GENERICVIRTUALRANGESENSOR_H
#define GENERICVIRTUALRANGESENSOR_H

#include <vector>
//#include <mrcore.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include <SerialPort.h>
#include <datatype/gridmap/labeledgridmap.h>
#include <datatype/sensordata/rangedata.h>
 
#include "genericrangesensor.h"

using namespace std;
using namespace mr;

class GenericVirtualRangeSensor : public GenericRangeSensor {
/***************************************************************************************************
 * GenericVirtualRangeSensor													
 * In this class we are emulating a range sensor working in a virtual world (described by a the map),
 * and updating its readings.					
 **************************************************************************************************/


	public:
		GenericVirtualRangeSensor(mr::LabeledGridMap * Map_ptr, mr::Pose rPose);
					
		~GenericVirtualRangeSensor(){};
		mr::LabeledGridMap * savedMap_ptr; // Map where the obstacles are detailed
		mr::Pose robPose;	// Where we are placing our virtual robot
};


#endif
