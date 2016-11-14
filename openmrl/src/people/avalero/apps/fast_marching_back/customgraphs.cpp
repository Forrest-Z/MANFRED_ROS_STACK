#include "customgraphs.h"

namespace mr{

PoseSpeed::PoseSpeed(Pose p, double s):Pose(p),speed(s){}

/********************************************
 ***********  POSESPEED GRAPH FUNCS ************
 ********************************************/

//! dist among two Pose nodes
template<> double PoseSpeedNode::costTo(PoseSpeedNode* dest){
	double time = value.timeTo(dest->value);
	//cout << time << endl;
	return time;
}

} //end namespace
