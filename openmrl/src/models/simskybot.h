#ifndef SIMSKYBOT_H
#define SIMSKYBOT_H

#include "models/robot.h"
#include "models/rangesensor.h"
#include <datatype/gridmap/labeledgridmap.h>

namespace mr 
{

class SimSkyBot: public Robot{
	DECLARE_MRL_OBJECT(SimSkyBot)
	public:
		SimSkyBot(){};
		SimSkyBot(mr::LabeledGridMap * environmentMap):Robot(){map = environmentMap;}

		virtual ~SimSkyBot(){}
		
		vector <RangeSensor> * sensors;
		
		mr::LabeledGridMap * map;	// The map is used to create the random individuals	
		bool updateRangeData();

};

}
#endif
