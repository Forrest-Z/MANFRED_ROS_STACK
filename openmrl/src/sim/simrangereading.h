#ifndef SIMRANGEREADING_H
#define SIMRANGEREADING_H

#include "datatype/object.h"
#include "datatype/sensordata_includes.h"
#include "datatype/gridmap_includes.h"

namespace mr{

class SimRangeReading : public GenericRangeReading
{
	 DECLARE_MRL_OBJECT(SimRangeReading)
public:
SimRangeReading(){};
	 SimRangeReading(LabeledGridMap *map, string name, Pose pose, double maxRange=10, double minRange=0, double fov=deg2rad(30), double sigma=0);

	 virtual ~SimRangeReading(){}
};

} // end namespace

#endif // SIMRANGEREADING_H
