#include "simrangedata.h"
#include "simrangereading.h"

namespace mr{

SimRangeData::SimRangeData()
{
}

SimRangeData::SimRangeData(LabeledGridMap *map, Pose pose, string name, int number_readings, double fov, double init_angle, double final_angle, double minRange, double maxRange, double sigma){
	double total_fov = final_angle - init_angle;
	double angle_step = total_fov / (number_readings-1);

	for (int i=0;i<number_readings;i++){
		 double angle = init_angle + i*angle_step;
		 Pose p(pose.x, pose.y, pose.theta+angle);
		 SimRangeReading rd(map,"generic",p,maxRange,minRange,fov,sigma);
		 readings.push_back(rd);
	}
}

SimRangeData::SimRangeData(LabeledGridMap *map, vector<Pose>poses, string name, double fov, double minRange, double maxRange, double sigma){
	 for (int i=0;i<poses.size();i++){
		  SimRangeReading rd(map,"generic",poses[i],maxRange,minRange,fov,sigma);
		  readings.push_back(rd);
	 }
}

} // end namespace




