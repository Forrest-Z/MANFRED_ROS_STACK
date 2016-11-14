#include "genericrangesensor.h"
extern int debug;

istream & operator >> (istream& is, GenericRangeSensor & sensor){
	// Configuring the sensor with a stream
	// The stream must start with something like:
	// Config: IR 0.01 0.15 0.1 0.6 0 0 0 0 0 1.5708 -1.5708 181
	
	//
	if (debug > 0) cout << "(Debug 1)       " <<"Writing data from a stream to a GenericRangeSensor "<< endl;
	//
	
	string header;
	is >> header;
	//
	if (debug > 0) cout << "(Debug 1)       " <<"  The first word of the stream is: " << header << endl;
	//
	
	if ((is.fail() == false) && (header == ("Config:")))
	{
		//
		if (debug > 0) cout << "(Debug 1)       " <<"  Configuring the sensor"<< endl;
		//
		is>>sensor.type;
		is>>sensor.defaultSigma;
		is>>sensor.fov;
		is>>sensor.minRange;
		is>>sensor.maxRange;
		// Reading point2o sensorPose:
		is>>sensor.sensorPose.x;
		is>>sensor.sensorPose.y;
		double sensorPoseTheta;
		is>>sensorPoseTheta;
		sensor.sensorPose.theta.setValue(sensorPoseTheta);
		is>>sensor.servoCenter.x;
		is>>sensor.servoCenter.y;
		is>>sensor.startAngle;
		is>>sensor.endAngle;
		is>>sensor.servoSteps;
	}
	// Now we parse the rest of the log
	
	//
	if (debug > 0) cout << "(Debug 1)       " <<"  Saving data from the stream "<< endl;
	//
	
	// Deleting all previous data from sensorData:
	
	
	sensor.sensorData.clear();
		
	string line;
	getline (is,line); // Quick hack to avoid reading strange characters in the first line
	
	while(1){ // break inside the loop
		
		getline (is,line);
		//if (debug > 2) cout << "      (Debug 3)   " << "Parsing line: " << line << endl;
		istringstream streamLine(line);
		// We are looping until we reach the end of the log
		if (line == "EOF") break;
		
		double angle;
		streamLine >> angle;
		double distance;
		streamLine >> distance;
		
		if (debug > 2) {
			cout << "      (Debug 3)   ";
			cout << "Angle: " << angle;
			cout <<";\tDistance: " << distance << "\n";
		}
		
		// Now, include this data in the sensor's sensorData
		sensor.pushAngleDistance(angle, distance);
	}
}


/***************************************************************************************************
 * GenericRangeSensor (constructor):
 * 
 * Specifies the sensor and servo characteristics
 **************************************************************************************************/
GenericRangeSensor::GenericRangeSensor (string typ, 
							float dSigma,
							float f,
							float minR,
							float maxR,
							mr::Pose sPos,
							Vector2D sCenter,
							double sAngle,
							double eAngle,
							int sSteps)
{
	type = typ;
	defaultSigma = dSigma;	// Sensor uncertainty in m (measurement +- uncertainty) TODO 
	fov = f;			// Sensor Field Of View (Angle of detection) IN RADIANS!!
	minRange = minR;		// Sensor minimum distance to detect an obstacle reliably (m)
	maxRange = maxR;		// Sensor maximum distance to detect an obstacle reliably (m)
	sensorPose = sPos;	// Relative position and pointing angle of the sensor w.r.t. the axis of the servo
	servoCenter = sCenter;// Relative position of the axis of the servo w.r.t. the body of the robot
	startAngle = sAngle;	// Maximum angle (wrt the x axis of the robot) that the servo can point
	endAngle = eAngle;		// Minimum .......
	servoSteps = sSteps;		// Number of different positions to where the servo can point
}


void GenericRangeSensor::pushAngleDistance(double angle, double distance)
{
	GenericRangeReading thisMeasurement(type, distance, Pose(0,0,angle), maxRange, minRange, fov, defaultSigma);
	sensorData.pushReading(thisMeasurement);
}


/***************************************************************************************************
 * nRays:
 * 
 * Returns how many rays are necessary to fill uniformly all the cone 
 * where the obstacle is.
 * 
 * Parameters:
 * 		maxDistance -- Maximum expected distance from sensor to the 
 * 					   obstacle.
 *		pixWidth -- real pixel width (in m)
 * Usage:
 * 		nRays(obstacleDist+uncertainty, 0.01);
 **************************************************************************************************/
int GenericRangeSensor::nRays(float maxDistance, float pixWidth){
	float minRays;
	minRays = fov * maxDistance / pixWidth;
	return ceil(minRays);
}
