#ifndef GENERICRANGESENSOR_H
#define GENERICRANGESENSOR_H

#include <vector>
//#include <mrcore.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include <SerialPort.h>
#include <datatype/gridmap/labeledgridmap.h>
#include <datatype/sensordata/rangedata.h>
 
using namespace std;
using namespace mr;



// From rev 1059, RangeSensor uses GenericRangeReading and GenericRangeData
//  GenericRangeReading(string type, double range, Pose pose = Pose(0,0,0), double maxR=5, double minR=0 ,  double fov=0, double sigma=0){
//  GenericRangeData();

class GenericRangeSensor{
/***************************************************************************************************
 * GenericRangeSensor													
 * Defines the hardware of a range sensor					
 **************************************************************************************************/


	public:
		GenericRangeSensor (string typ = "sensor", 
										float dSigma = 0.01, 
										float f = deg2rad(8),
										float minR = 0.1, 
										float maxR = 0.8, 
										mr::Pose sPos =mr::Pose(0,0,0), 
										Vector2D sCenter = mr::Vector2D(0,0), 
										double sAngle = -M_PI_2, 
										double eAngle = M_PI_2, 
										int sSteps = 181);
		~GenericRangeSensor(){};
		
	   /*
	    * Sensor characteristics:
	    */
		string type; 		// Letter located at the beginning of each serial sensor log to discern from which sensor the data comes
		float defaultSigma;	// Sensor uncertainty in m (measurement +- uncertainty) TODO 
		float fov;			// Sensor Field Of View (Angle of detection) IN RADIANS!!
		float minRange;		// Sensor minimum distance to detect an obstacle reliably (m)
		float maxRange;		// Sensor maximum distance to detect an obstacle reliably (m)
		Pose sensorPose;	// Relative position and pointing angle of the sensor w.r.t. the axis of the servo
  		Vector2D servoCenter;// Relative position of the axis of the servo w.r.t. the body of the robot
  		float startAngle;	// Maximum angle (wrt the x axis of the robot) that the servo can point
  		float endAngle;		// Minimum .......
  		int servoSteps;		// Number of different positions to where the servo can point
  		GenericRangeData sensorData;	// The data of a full scan of the robot
  		
  		void pushAngleDistance(double angle, double distance);
		int nRays(float maxDistance, float pixWidth);
};

// Here we create an easy and flexible way to configure the sensor and save data from a file or robot
istream & operator >> (istream & is, GenericRangeSensor & sensor);



#endif
