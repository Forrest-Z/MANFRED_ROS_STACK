#ifndef SIMSKYBOT_H
#define SIMSKYBOT_H

#include "robot.h"
#include "rangesensor.h"
#include <datatype/gridmap/labeledgridmap.h>

class SimSkyBot: public Robot{
	public:
		SimSkyBot(mr::LabeledGridMap * environmentMap){map = environmentMap;};
		SimSkyBot(){};
		~SimSkyBot(){};
		
		vector <mr::RangeSensor *> sensors;
		
		mr::LabeledGridMap * map;	// The map is used to create the random individuals	

		bool updateRangeData();
};


#endif
