/***********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Alberto Valero.
 * Mail: alberto.valero.gomez@gmail.com
 *
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, controlDistanceribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may controlDistanceribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is controlDistanceributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 **********************************************************************/

#include "springreactivecontrol.h"
#include "datatype/geometry/geometryfunctions.h"

namespace mr{

void SpRController::controlWithTarget(){

	forces.clear();

	Vector2D directionVector = vectorFromTo(robotPose,targetPose);
	controlAngle = directionVector.argument()-robotPose.theta.getValue();
	controlDistance = mrl_distance(robotPose,targetPose);
	
	controlWithAngleDistance();
}

void SpRController::controlWithAngleDistance(){

	//Compute the attractive force relative to robot (also orientation)
	Vector2D force;
	force.setPolarCoords(attrForceModule,controlAngle);
	//cout<<"Robot Pose: "<<robotPose.x<<" "<<robotPose.y<<endl;
	//cout<<"Target Pose: "<<targetPose.x<<" "<<targetPose.y<<endl;
	//cout<<"force attr: "<<attrForceModule<<" "<<rad2deg(controlAngle)<<endl;
    //cout<<"force attr 2: "<< force.module() <<" "<<rad2deg(force.theta())<<endl;

	forces.push_back(force); //add to the vector of forces acting on the robot.

	//The proportional repulsion dependes on the number of readings and the attractive force.
	double k2=attrForceModule/rangeData.readings.size();

	//Compute the repulsive forces relative to robot. Consider only front obstacles closer than target

	for (unsigned int i=0; i<rangeData.readings.size(); i++){
	//Check if rangeDevices are facing forward
		if ( ( ( (rangeData.readings[i].pose.theta >= -deg2rad(90))
					&& (rangeData.readings[i].pose.theta <= deg2rad(90)) )
				||
				( (rangeData.readings[i].pose.theta >= controlAngle-deg2rad(45))
					&& (rangeData.readings[i].pose.theta <= controlAngle + deg2rad(90)) )

			)&&(
				( rangeData.readings[i].range <= controlDistance ) ) )
		{
			Vector2D force;
			force.setPolarCoords(k2/rangeData.readings[i].range, rangeData.readings[i].pose.theta + deg2rad(180));
			forces.push_back(force); //add the force
		}
	}




	//Compute the resulting force
	resultingForce.x=resultingForce.y=0;

	for (unsigned int i=0; i<forces.size(); i++){
	    //std::cout<<"Force "<<i<<" "<<forces[i].module()<<" "<<rad2deg(forces[i].theta())<<endl;
		resultingForce=resultingForce+forces[i];
	}

	//set control speed and jog

	double angleError=resultingForce.argument();
	//std::cout<<"Angle Error " << rad2deg(angleError) << endl;

	//force frontal angles to be smaller than 90.
	if (angleError < deg2rad(-270) ) angleError = angleError + deg2rad(360);
	else if (angleError > deg2rad(270) ) angleError = angleError - deg2rad(360);

	//Control jog and speed

	speed = fabs(speedGain/angleError); //the robot must not go backwards
	jog = angleError*jogGain;

	if (isArrived()){ //when arrived stop
		speed=0;
		jog=0;
	}

	//std::cout<<"Speed: "<<speed<<" Jog: "<<rad2deg(jog)<<endl;

}

}//end namespace
