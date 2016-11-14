/***********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Victor Perez
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

#ifndef PROP_REACTIVE_CONTROL_H
#define PROP_REACTIVE_CONTROL_H

#include "genericmotioncontroller.h"
#include "datatype/geometry/geometryfunctions.h"

namespace mr{


/************************************************
 * PropController. This class implements
 * a non-reactive controller that takes robot pose and
 * target pose and tries to reach the target using a 
 * propotional controller.
 * The controller does not plan a path. So the target
 * point must be under sight.
 ***********************************************/
class PropController : public GenericMotionController{
	public:
	PropController(){}
	~PropController(){}
	/**
	 * Configure the params of the controller
	 * \param ks Gain of the speed
	 * \param kj Gain of the jog
	 * \param maxSpeed Maximum linear speed
	 * \param maxJog Maximum angular speed
	 */
	inline void configure(double ks, double kj, double maxSpeed=1, double maxJog=M_PI/6)
	{
		this->ks=ks; this->kj=kj; this->maxSpeed=maxSpeed; this->maxJog=maxJog;
		controlActionComputed=false;
	} 
	
	protected:
	void virtual controlWithTarget();
	void virtual controlWithAngleDistance();

	private:
	double ks;
	double kj;
	double maxSpeed;
	double maxJog;
};

}//end namespace

#endif
