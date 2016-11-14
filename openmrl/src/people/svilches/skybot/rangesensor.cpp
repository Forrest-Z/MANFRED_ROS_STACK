#include "rangesensor.h"
namespace mr{

RangeSensor::RangeSensor(string typ, mr::Pose sPose, double fv=mr::deg2rad(8), double minR=0.1, double maxR=0.8){
	type = typ;
	sensPose = sPose;
	fov = fv;
	maxRange = maxR;
	minRange = minR;
}

GenericRangeReading RangeSensor::measure(mr::LabeledGridMap * map, mr::Pose robPose){
	// TODO: This could be done only once in constructor
	// compute the number of rays to trace inside the fov cone
	double map_res = map->getResolution();
	double arc_length = maxRange * fov ;

	int num_rays = int (arc_length/map_res);
	if (num_rays < 1) num_rays=1;

	double angle_step = fov/num_rays;

	double range = maxRange;

	LabeledGridMap::RayPoint ray_point;

	Point2o pose = robPose + sensPose;

	for (double angle= -fov/2 ; angle <= fov/2 ; angle+=angle_step){
		// isThereCellValueInRay (Point2o point, T value, double maxdistance, RayPoint &rayPoint)
		if (map->isThereCellValueInRay(pose,angle,LabeledGridMap::OBSTACLE,maxRange,ray_point)){
			Pose obst_pose(ray_point.x, ray_point.y);
			float aux = mrl_distance(pose,obst_pose);
			if (aux<range) range=aux;
		}
	}
	
	GenericRangeReading measurement(type, range, pose, maxRange, minRange, fov, 0);
	
	return measurement;
	
}

}
