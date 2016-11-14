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
#include "datatype/object.h"

namespace mr 
{

class Robot: public Object {
	//DECLARE_MRL_OBJECT(Robot)
	
	public:
		Robot();
		virtual ~Robot(){};
				
		mr::GenericRangeData * rangeData;
		
		mr::Pose pose;
		
		// FIXME!!!
		virtual bool updateRangeData()=0;

		//virtual bool updateRangeData(){};
		
		void drawGL2D() const;
};

}
#endif
