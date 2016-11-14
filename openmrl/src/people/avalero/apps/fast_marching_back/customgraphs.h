#include <mrcore.h>

namespace mr{

//!< Speed Graphs Typedefs
class PoseSpeed : public Pose {
	public:
	PoseSpeed(){}
	PoseSpeed(Pose p, double s);
	double speed;
	inline double timeTo(PoseSpeed &dest){
		double dist = distance(Pose(x,y),Pose(dest.x, dest.y));
		double mean_speed = (speed + dest.speed ) / 2 ;
		if (mean_speed>100) mean_speed=100;
		double time = dist / mean_speed;
		return time;
	}
};

typedef Graph<PoseSpeed,double> PoseSpeedGraph;
typedef Node<PoseSpeed,double> PoseSpeedNode;
typedef Edge<PoseSpeed,double> PoseSpeedEdge;
typedef Searcher<PoseSpeed, double> PoseSpeedGraphSearcher;
typedef UninformedSearcher<PoseSpeed,double> UninformedPoseSpeedSearcher;
typedef DijkstraSearcher<PoseSpeed,double> DijkstraPoseSpeedSearcher;
typedef AstarSearcher<PoseSpeed,double> AstarPoseSpeedSearcher;

}
