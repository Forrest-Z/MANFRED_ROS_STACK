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

#ifndef SPRING_REACTIVE_CONTROL_H
#define SPRING_REACTIVE_CONTROL_H

#include "genericmotioncontroller.h"


namespace mr{

/************************************************
 *  SpringBasedController. This class implements
 *  a reactive controller that takes generic range
 *  data and sets speed in order to avoid obstacles
 *  in the way to a target point.
 *  The controller does not plan a path. So the target
 *  point must be under sight.
 ***********************************************/
class SpRController: public GenericMotionController{

public:
	SpRController(): GenericMotionController() {speedGain=0.1;jogGain=1;attrForceModule=2;}
	virtual ~SpRController(){}

//methods

	//! Function to configure the controller
    /**!
      \param sG Speed Gain.
      \param jG Jog Gain
      \param attr Attractive Force to the target Point
    */
	void configure(double sG=0.01, double jG=1, double attr=2, double arrDist=1){
		controlActionComputed=false;speedGain=sG;jogGain=jG;attrForceModule=attr; arrivalDistance=arrDist;
	}
	//! gets the resulting virtual force after applying the control action
    /**!
      \return virtual resulting force
    */
	Vector2D getResultingForce(){return resultingForce;}
	//! gets the resulting virtual forces (attractive target, repulsive obstacles)
	/**!
      \return virtual forces
    */
	vector<Vector2D> getForces(){return forces;}

protected:
	void virtual controlWithTarget();
	void virtual controlWithAngleDistance();
	Vector2D resultingForce;
	vector<Vector2D>forces;

private:
	double speedGain;
	double jogGain;
	double attrForceModule;

};

} //endnamespace

#endif
