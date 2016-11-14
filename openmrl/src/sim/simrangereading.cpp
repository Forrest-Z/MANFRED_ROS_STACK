#include "simrangereading.h"

namespace mr{

IMPLEMENT_MRL_OBJECT(SimRangeReading)

SimRangeReading::SimRangeReading(LabeledGridMap *map, string name, Pose pose, double maxRange, double minRange, double fov, double sigma)
	 :GenericRangeReading( name, maxRange, pose, maxRange, minRange, fov, sigma){

	 //compute the number of rays to trace inside the fov cone
	 double map_res = map->getResolution();
	 double arc_length = maxRange * fov ;

	 int num_rays = int (arc_length/map_res);
	 if (num_rays < 1) num_rays=1;

	 double angle_step = fov/num_rays;

	 range = maxRange;

	 LabeledGridMap::RayPoint ray_point;

	 for (double angle= -fov/2 ; angle <= fov/2 ; angle+=angle_step){
		  if (map->isThereCellValueInRay(pose,angle,LabeledGridMap::OBSTACLE,maxRange,ray_point)){
				Pose obst_pose(ray_point.x, ray_point.y);
				float aux = mrl_distance(pose,obst_pose);
				if (aux<range) range=aux;
		  }
	 }
}

} // end namespace


