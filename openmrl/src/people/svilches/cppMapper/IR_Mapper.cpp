#include <iostream>
#include <mrcore/navigation2d/occupancygridmap255.h>
#include <mrcore.h>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include <SerialPort.h>
#include "RangeSensor.h"
#include "mapviewer.h"

using namespace std;

int main(int argc, char* argv[]){
	mr::Pose robotPose(0,0.3,0);

	/*
	 * Create an object of the sensor we are using (InfraRed) and receive its data
	 */
	float IR_fov = mr::deg2rad(8.0);
	RangeSensor IR_Sensor( 'i', 0.02, IR_fov, 0.1, 0.6, 0.025, M_PI/6 ); 	//char sensorID, float uncertainty, float fov, float minRange, float maxRange, float sensorOffset, float sensorDiphase

	// Creating the map:
	// usage: mr::OccupancyGridMap255 (double real_width, double real_height, double resolution, unsigned char default_value=127, double mapX=0, double mapY=0);
	mr::OccupancyGridMap255 *map = new mr::OccupancyGridMap255(1.20, 1.10, 0.005, 127, -0.1 , -0.1); // (1.20, 1.10, 0.01, 127, -0.1 , -0.1);
	
	cout << "Starting Mapping.\n";
	while(1){
		cout << "Insert the robot position:\n X (meters): ";
		cin >> robotPose.x;
		cout << "Y (meters): ";
		cin >> robotPose.y;
		float robAngle;
		cout << "Angle (deg): ";
		cin >> robAngle;
		robotPose.theta = mr::deg2rad(robAngle);
		int times;
		cout << "How many measurements do you want to make? ";
		cin >> times;
		
		// Take readings
		for (int i = 0; i < times; i++)
		{
			IR_Sensor.receiveSerialData(); // Data is stored in the object
			// Make map
			IR_Sensor.updateMap(map, robotPose); 
			// Create Log
			//IR_Sensor.saveSensorBuffer(robotPose);
		}
		
		mr::Image *bitMap_ptr;
		bitMap_ptr = map->convertToImage();
		cout << "Saving map.bmp" << endl;
		bitMap_ptr->save("map.bmp");
		delete bitMap_ptr;
		
		// Ask for next
		
		char option;
		cout << "Take more readings (y/n)? ";
		cin >> option;
		if (option == 'n') break;
	}
	
	// Show map
	//GLViewMap(map, &argc, argv);
	
	// Save map
	mr::Image *bitMap_ptr;
	bitMap_ptr = map->convertToImage();
	cout << "Saving map.bmp" << endl;
	bitMap_ptr->save("map.bmp");
	delete map;
	delete bitMap_ptr;

	return 0;
}
