#include "genericvirtualrangesensor.h"
extern int debug;

/***************************************************************************************************
 *
 * GenericVirtualRangeSensor (constructor):
 * 
 **************************************************************************************************/
GenericVirtualRangeSensor::GenericVirtualRangeSensor(	mr::LabeledGridMap * Map_ptr, 
														mr::Pose rPose)
														:GenericRangeSensor()
{
	savedMap_ptr = Map_ptr; // Map where the obstacles are detailed
	mr::Pose robPose = rPose;	// Where we are placing our virtual robot
	
	/***************************************************************************************************
	 * virtualScan:
	 * In this function we are emulating a range sensor working in a virtual world (described by a the map),
	 * and updating virtualReadings.
	 **************************************************************************************************/
	
	sensorData.clear();
	// Relative to servo movement
	double totalFov = endAngle - startAngle;
	double servoStep = totalFov / (servoSteps-1);
	// Relative to ray tracing
	float mapRes = savedMap_ptr->getResolution();
	int raysInCone = nRays(maxRange , mapRes);
	double rayStep = fov/raysInCone;	
	
	
	
	// We are placing a virtual robot somewhere in the map and it will start scanning from -90 to 90 deg
	
	for (int servoPos = 0; servoPos < servoSteps; servoPos++){
		
		// servoAngle is relative to the direction of the robot
		double servoAngle = startAngle + servoStep * servoPos ; // Servo orientation
		
		// Now that the servo is pointing to a certain position, let's see it it finds something in its FOV
		double range = maxRange;
		LabeledGridMap::RayPoint ray_point;
		
		for (double rayAngle = -fov/2.0 ; rayAngle <= fov/2.0 ; rayAngle+=rayStep){
			Point2 robotPoint(robPose.x,robPose.y);
			
			double rayDir = robPose.angle() + servoAngle + rayAngle;
			
			if (savedMap_ptr->isThereCellValueInRay(robotPoint, rayDir,LabeledGridMap::OBSTACLE,maxRange,ray_point)){
				Pose obst_pose(ray_point.x, ray_point.y);
				float aux = mrl_distance(robPose,obst_pose);
				if (aux<range) range=aux;
			}
		}
		pushAngleDistance(servoAngle, range);
	} // Gone through all the servo steps
}
