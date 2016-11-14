#ifndef SIMRANGEDATA_H
#define SIMRANGEDATA_H

#include "datatype/sensordata_includes.h"
#include "datatype/gridmap_includes.h"


namespace mr{
class SimRangeData : public GenericRangeData
{
public:
    SimRangeData();
	 SimRangeData(LabeledGridMap* map, Pose pose, string name,
					  int number_readings, double fov, double init_angle=-deg2rad(90),
					  double final_angle=deg2rad(90), double minRange=0, double maxRange=10, double sigma=0 );
	 SimRangeData(LabeledGridMap* map, vector<Pose> poses, string name,
					  double fov,double minRange=0, double maxRange=10, double sigma=0 );
};

}//end namespace

#endif // SIMRANGEDATA_H
