/***********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Victor Perez
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

#include "propcontroller.h"

namespace mr{
	
void PropController::controlWithTarget(){
	Vector2D directionVector = vectorFromTo(robotPose,targetPose);
	controlAngle = directionVector.argument() - robotPose.theta.getValue();
	controlDistance = mrl_distance(robotPose,targetPose);
	
	controlWithAngleDistance();
}

void PropController::controlWithAngleDistance(){
	
	speed=1/(ks*controlAngle);
	jog=kj*controlAngle;
	
	if (controlDistance < 1) speed=0.1;
	
	if (speed>maxSpeed) speed=maxSpeed;
	if (fabs(jog)>maxJog){
		if (jog>0) jog=maxJog;
		if (jog<0) jog=-maxJog;
	}
}

}//end namespace
