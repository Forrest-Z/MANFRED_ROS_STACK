#include "simskybot.h"

namespace mr 
{

	IMPLEMENT_MRL_OBJECT(SimSkyBot)


bool SimSkyBot::updateRangeData(){
	rangeData->clear();
	cout.flush();
	for (int i = 0; i < sensors->size(); i++)
	{
		rangeData->pushReading((*sensors)[i].measure(map,pose));
	}
}
}
