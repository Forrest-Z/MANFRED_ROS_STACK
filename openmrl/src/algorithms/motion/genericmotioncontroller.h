/***********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Alberto Valero
 *
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 **********************************************************************/

#ifndef GENERIC_CONTROL_H
#define GENERIC_CONTROL_H

#include "datatype/sensordata_includes.h"

namespace mr{
/*** The Generice Motion Controller is a virtual class for defining the interface
 * of a generic reactive controller.
 * It takes as inputs the range data, robot pose, and target pose.
 * It outputs the control Speed and Jog **/

class GenericMotionController{

public:
	GenericMotionController(){controlActionComputed=false;speed=jog=0;}
	virtual ~GenericMotionController(){}

//methods
	virtual void controlWithTarget() = 0;
	virtual void controlWithAngleDistance() = 0;
	/*** method to set the range data
	\param GenericRangeData, a vector of general range sensor data**/
	inline void setRangeData(const GenericRangeData& rg){rangeData=rg; controlActionComputed=false;}
	/*** method to set the robot actual and target poses
	\param rP The robot current pose
	\param tP The robot targetPose
	**/
	inline void setPoses(const Pose& rP, const Pose& tP){robotPose=rP;targetPose=tP; controlActionComputed=false; hasDestinationPoint=true;}
	inline void setControlAngleDistance(double angle,double distance=1){controlAngle=angle; controlDistance= distance; controlActionComputed=false; hasDestinationPoint=false;}
	inline void setRelativeTarget(const Pose& tP){robotPose=Pose(0,0,0);targetPose=tP; controlActionComputed=false; hasDestinationPoint=true;}
	inline double getControlSpeed(){if (!controlActionComputed) computeControlAction(); return speed;}
	inline double getControlJog(){if (!controlActionComputed) computeControlAction(); return jog;}
	inline bool isArrived(){return ( mr::mrl_distance(robotPose, targetPose) < arrivalDistance);}
protected:
	GenericRangeData rangeData;
	bool controlActionComputed;
	bool hasDestinationPoint;
	double speed, jog;
	double arrivalDistance;
	Pose robotPose;
	Pose targetPose;
	double controlAngle, controlDistance;
	
	inline void computeControlAction(){
		if (hasDestinationPoint) controlWithTarget();
		else controlWithAngleDistance();
		controlActionComputed=true;
	}

};

} //endnamespace

#endif
