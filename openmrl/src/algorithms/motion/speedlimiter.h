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




#ifndef SPEEDLIMITER_H
#define SPEEDLIMITER_H

#include "genericmotioncontroller.h"

namespace mr{

class SpeedLimiter: public GenericMotionController
{
	public:
		SpeedLimiter();

	//! Function to configure the controller
    /**!
      \param sL Speed Limit
      \param jL Jog Limit
      \param frontD Safe Front Distance
      \param safeA Safe Angle for Jog Limit
      \param frontPV Proportional value for speed limit according to the distance to obstacles
      \param lateralPV Proportional value for jog limit according to the distance to obstacles
      \param allowBack Allow backwards movements (speed < 0)
    */
	void virtual configure(double sL=1, double jL=deg2rad(30), double frontD=0.5, double safeA=deg2rad(60), double frontPV=1, double lateralPV=1, bool allowBack=false){
		controlActionComputed=false;speedLimit=sL;jogLimit=jL; frontalSafeDistance=frontD; allowBackWards=allowBack;
		frontalPropValue=frontPV; lateralPropValue=lateralPV; safeAngle=safeA;
	}
	//! Set the desired Speed and Jog. The speed limiter will compute these value and return a limitted speed and jog
	/**!
	 * \param des_speed Desired linear speed
	 * \param des_jog Desired Jog (angular speed)
	 */
	void setDesiredSpeed(double des_speed, double des_jog){controlActionComputed=false; desiredSpeed=des_speed; desiredJog=des_jog;}

protected:
	void virtual controlWithTarget();
	void virtual controlWithAngleDistance();

private:
	double speedLimit, jogLimit, desiredSpeed, desiredJog;
	double lateralPropValue, frontalPropValue;
	double safeWidth, frontalSafeDistance, safeAngle;
	bool allowBackWards;
	void setMaxSpeed();
	void setMaxJog();
	void parseSpeed();
	void parseJog();


	double maxLeftJog, maxRightJog, maxFrontalSpeed, maxBackwardsSpeed;

};

} //end namesapce

#endif /** SPEEDLIMITER_H */
