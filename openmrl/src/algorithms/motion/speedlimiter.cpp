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


#include "speedlimiter.h"
#include <float.h>

namespace mr{

SpeedLimiter::SpeedLimiter()
	: GenericMotionController()
{

}

void SpeedLimiter::controlWithTarget(){
	setMaxJog();
	setMaxSpeed();
	parseJog();
	parseSpeed();
	controlActionComputed=true;

}

void SpeedLimiter::controlWithAngleDistance(){
	//TODO
	cerr << "Control With Angle and Distance not yet implemented" << endl;
}

void SpeedLimiter::setMaxJog(){

		int numOfPoints=rangeData.readings.size();

		double initAngle=safeAngle/2;
		double finalAngle=Angle::makePositive(-safeAngle/2);

		double angle=0;
		double rightLateralRange=0;
		double leftLateralRange=0;

		int countLeft, countRight;
		countLeft=countRight=0;

		for (int i=0; i<rangeData.readings.size(); i++){
			angle=rangeData.readings[numOfPoints-i].pose.theta.getValue();
			if ( (Angle::makePositive(angle) > initAngle )
				&& (Angle::makePositive(angle) < (PI + initAngle)) ) //left
			{
				leftLateralRange+=fabs(rangeData.readings[i].range*cos(angle)); //add all the ranges for making afterwards the mean
				countLeft++;
			}
			else if ((Angle::makePositive(angle) < finalAngle )
				&& (Angle::makePositive(angle) > (2*PI/2 - finalAngle)) )
			{
				rightLateralRange+=fabs(rangeData.readings[i].range*cos(angle)); //add all the ranges for making afterwards the mean
				countRight++;
			}

		//	RDK_DEBUG_PRINTF("Range: %f, Angle: %f, leftLateralRange %f",rangeData.readings[numOfPoints-i].reading, angle, leftLateralRange);
		}
		//mean value of the lateral range.
		leftLateralRange=leftLateralRange/countLeft-safeWidth;
		rightLateralRange=rightLateralRange/countRight-safeWidth;

		if (leftLateralRange<0) leftLateralRange=0; //just in case, but it should not happen
		if (rightLateralRange<0) rightLateralRange=0; //just in case, but it should not happen


		maxLeftJog=lateralPropValue*leftLateralRange; if (maxLeftJog<deg2rad(1)) maxLeftJog=deg2rad(1);
		maxRightJog=lateralPropValue*rightLateralRange; if (maxRightJog<deg2rad(1)) maxRightJog=deg2rad(1);

}

void SpeedLimiter::setMaxSpeed(){

	double minFrontalRange, minBackwardsRange;

	minBackwardsRange=minFrontalRange=DBL_MAX;

	//MIN FRONTAL AND BACKWARDS RANGE MEASURED WITH RANGE DATA

	for (unsigned int i=0; i<rangeData.readings.size(); i++)
	{
		if ( fabs(rangeData.readings[i].range * sin(rangeData.readings[i].pose.theta.getValue()) ) <= safeWidth / 2 )
		{
			if (  ( Angle::getQuadrant(rangeData.readings[i].pose.theta.getValue()) == 1) ||
			      ( Angle::getQuadrant(rangeData.readings[i].pose.theta.getValue()) == 4) ) //frontal obstacle
			{
				if ( fabs(rangeData.readings[i].range * cos(rangeData.readings[i].pose.theta.getValue()) ) <= minFrontalRange )
					minFrontalRange = fabs(rangeData.readings[i].range * cos(rangeData.readings[i].pose.theta.getValue()));
			}
			else if (  ( Angle::getQuadrant(rangeData.readings[i].pose.theta.getValue()) == 2) ||
			      ( Angle::getQuadrant(rangeData.readings[i].pose.theta.getValue()) == 3) ) //back obstacle
			{
				if ( fabs(rangeData.readings[i].range * cos(rangeData.readings[i].pose.theta) ) <= minBackwardsRange )
					minBackwardsRange = fabs(rangeData.readings[i].range * cos(rangeData.readings[i].pose.theta));
			}
		}
	}

	maxFrontalSpeed=(minFrontalRange-frontalSafeDistance)*frontalPropValue;
	maxBackwardsSpeed=(minBackwardsRange-frontalSafeDistance)*frontalPropValue;

	if (!allowBackWards){
		if (maxFrontalSpeed < 0) maxFrontalSpeed=0;
		if (maxBackwardsSpeed > 0) maxBackwardsSpeed = 0;
	}
}

void SpeedLimiter::parseSpeed()
{

	if ( fabs(desiredSpeed) < 0.05 ){
		speed=0;
		return;
	}

	if (desiredSpeed < -maxBackwardsSpeed) desiredSpeed = -maxBackwardsSpeed; //max computed speed
	if (desiredSpeed > maxFrontalSpeed) desiredSpeed = maxFrontalSpeed;


	if (desiredSpeed > speedLimit) desiredSpeed=speedLimit; //speed limit set by parameter
	if (desiredSpeed < -speedLimit)	desiredSpeed=-speedLimit;

	if ( (desiredSpeed < 0) && (!allowBackWards) ) desiredSpeed=0;

	speed=desiredSpeed;
}

void SpeedLimiter::parseJog()
{

	if (desiredJog > maxLeftJog) desiredJog=maxLeftJog; //max computed jog
	if (desiredJog < -maxRightJog) desiredJog=-maxRightJog;

	jog=desiredJog;
}


}//end namespace
