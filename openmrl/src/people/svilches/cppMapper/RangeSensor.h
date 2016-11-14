#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <vector>
#include <mrcore.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include <SerialPort.h>

using namespace std;
using namespace mr;

class sensorData{
/***************************************************************************************************
 * Class to store the raw information from the sensor coming from the 
 * serial port.
 * 
 * Usage:
 * 	sensordata lastLine(-0.14, 0.98, i);
 * 	lastAngle = lastLine.angle;
 **************************************************************************************************/
	
   	public:
		sensorData(float a, float d, char sT)
		{
		 angle = a; distance = d; sensorType = sT;
		}
		float angle;
		float distance;
		char sensorType;
};


#endif

class RangeSensor{
/***************************************************************************************************
 * RangeSensor													
 * A set of specific data containers and functions for working with	
 * range finder sensors (ie InfraRed, ultrasonic...)							
 * 
 * Member functions:	
 * 	public:											
 * 		receiveSerialData -- Get the readings from the sensor and store 
 * 							 them in sensorBuffer.
 * 		updateMap -- Use the last sensor data to update the map, given 
 * 					 the pose of the robot.
 * 	private:
 * 		nRays -- Returns how many rays are necessary to fill uniformly 
 * 				 all the cone where the obstacle is.
 * 		getStandardDeviation -- Dependent on sensor type and distance 
 * 								to the obstacle.
 * 		updateCell -- Returns the cell value modified according to the mode (No obstacle / Obstacle)
 * 					  and the area that the cone is spanning.
 **************************************************************************************************/


	public:
		RangeSensor(char sensorID, float defaultSigma, float fov, float minRange, float maxRange, float sensorOffset, float sensorDiphase);
		~RangeSensor(){};
		int receiveSerialData(); 
		int updateMap(mr::OccupancyGridMap255 *, mr::Pose robPose);
		int saveSensorBuffer(mr::Pose robPose);
		
	   /*
	    * Sensor characteristics:
	    */
		char sensorID; 		// Letter located at the beginning of each serial sensor log to discern from which sensor the data comes
		float uncertainty; 	// Sensor uncertainty in m (measurement +- uncertainty) TODO 
		float fov;			// Sensor Field Of View (Angle of detection) IN RADIANS!!
		float minRange;		// Sensor minimum distance to detect an obstacle reliably (m)
		float maxRange;		// Sensor maximum distance to detect an obstacle reliably (m)
		float sensorOffset; // distance from the center of rotation of the servo and the sensor (m)
		float sensorDiphase;// angle between the direction the servo is pointing and the line joining the
  						    //   center of the servo with the sensor.
	private:
		vector <sensorData> sensorBuffer;	// Here we will dump in a nice way all the info coming from the serial port
		int nRays(float maxDistance, float pixWidth);
		float getStandardDeviation(float distance);
		unsigned int updateCell(int prevCellValue, char mode, float area, float pixWidth);


};
