#ifndef RANGESENSOR_H
#define	RANGESENSOR_H

#include <iostream>
#include <vector>
#include "datatype/sensordata/rangedata.h"
#include <datatype/gridmap/labeledgridmap.h>
#include "datatype/geometry/point.h"
#include <string>
namespace mr{

class RangeSensor{
	public:
		RangeSensor(string typ, mr::Pose sPose, double fv, double minR, double maxR);
		~RangeSensor(){};
				
		mr::Pose sensPose;	// Relative position and pointing angle of the sensor w.r.t. the center of the robot
		string type;
		float fov;			// Sensor Field Of View (Angle of detection) IN RADIANS!!
		float minRange;		// Sensor minimum distance to detect an obstacle reliably (m)
		float maxRange;		// Sensor maximum distance to detect an obstacle reliably (m)
		
		mr::GenericRangeReading measure(mr::LabeledGridMap * environmentMap, mr::Pose robPose);
};

}
#endif
