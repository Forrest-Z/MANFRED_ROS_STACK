#include "rangesensor.h"

/***************************************************************************************************
 * RangeSensor (constructor):
 * 
 * Specifies the sensor's characteristics	
 **************************************************************************************************/
RangeSensor::RangeSensor(char sID, float ucert, float fv, float mR, float MR, float sensOffset, float sensDiphase){
	sensorID = sID;
	uncertainty = ucert;
	fov = fv;	
	minRange = mR;
	maxRange = MR;
	sensorOffset = sensOffset;
	sensorDiphase = sensDiphase;
	
}


/***************************************************************************************************
 * receiveSerialData:
 *
 * Get the readings from the sensor and store them in sensorBuffer
 **************************************************************************************************/
int RangeSensor::receiveSerialData(){
	// Here we will store each incoming line of serial data
	string serialLine; 
	
	// Clear the data from previous measurements
	sensorBuffer.clear();
	
	// Initializing the serial port
	::SerialPort serial_port("/dev/ttyUSB0");
	serial_port.Open(::SerialPort::BAUD_9600,
			         ::SerialPort::CHAR_SIZE_8,
			         ::SerialPort::PARITY_NONE,
			         ::SerialPort::STOP_BITS_1,
			         ::SerialPort::FLOW_CONTROL_NONE);
	bool endFlag = 0;
	// Read all the lines coming from the serial port until "End"
	while(1){ // break is inside the loop
		// serial_port.ReadLine(max waiting time, new line char)
		serialLine = serial_port.ReadLine(10000,'\n'); 
		//cout << "### " << serialLine;
		// We convert the current line into a stream, which contains 
		// 	 the sensor type, angle and distance.
		istringstream stream(serialLine); 
		
		char sensorType;
		stream >> sensorType;
		float angle;
		stream >> angle;
		float distance;
		stream >> distance;
		// if (distance > maxRange) distance = maxRange; // maxRange test is performed behind
		
		cout << "Sensor type: " << sensorType;
		cout << ";\tAngle: " << angle;
		cout <<";\tDistance: " << distance << "\n";
		
		
		// Now, include this data in the vector sensorBuffer
		sensorData lineData(angle,distance,sensorType); 
		sensorBuffer.push_back(lineData);
		if (angle > 1.53){
			cout << "=== Finished reading serial port data ===" << "\n";
			break;
		}
	} // End while
	
	/*
	 * If the data comes from the IR sensor, the distance will be an integer with the result from
	 * the ADC. Therefore, we must convert it to real distance with a formula:
	 * 	ADC_Units = 2.176 / (0.018424*dist + 0.00048)
	 */
	if (sensorID == 'i'){	// If we are working with IR sensor
		for (int i = 0; i < sensorBuffer.size(); i++)	// Convert all the entries of the log
		{
			float ADC_Units = sensorBuffer[i].distance;
			float dist;
			dist = 2.176 / (0.018424 * ADC_Units) - 0.00048 / 0.018424;
			//cout << ADC_Units << "->" <<dist << endl;
			if (dist > maxRange){
				dist = maxRange;
			}
			sensorBuffer[i].distance = dist;
			//cout << "### Distance: " << dist << endl;
		}
	}
	
	return 0;
} // End receiveSerialData



/***************************************************************************************************
 * saveSensorBuffer:
 * Saves in buffer.dat the readings from the sensor
 **************************************************************************************************/
int RangeSensor::saveSensorBuffer(mr::Pose robPose){
	
	ofstream outfile;
	outfile.open ("buffer.dat");
	outfile << robPose.x << " " << robPose.y << " " << robPose.theta << endl;
	for (int i = 0; i < sensorBuffer.size() ; i++)
	{
		outfile << sensorBuffer[i].angle << " " << sensorBuffer[i].distance << endl;
	}
	
	outfile.close();
	return 0;
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
int RangeSensor::nRays(float maxDistance, float pixWidth){
	float minRays;
	minRays = fov * maxDistance / pixWidth;
	return ceil(minRays);
}

/***************************************************************************************************
 * getStandardDeviation:
 * 
 * Returns the standard deviation associated to a certain measurement, depending on its distance.
 * 
 * Parameters:
 * 		distance -- distance from sensor to obstacle
 * Usage:
 * 		getStandardDeviation(sensorBuffer[i].distance);
 * Notes:
 * 		By now, this returns a fixed, constant value.
 * Improvements:
 * 		Create a program to measure the standardDeviation in function of the distance, save it in 
 * 		a file and access it in this function
 **************************************************************************************************/
float RangeSensor::getStandardDeviation(float distance){
	return (uncertainty);
}


/***************************************************************************************************
 * updateCell:
 * 
 * Returns the cell value modified according to the mode (No obstacle / Obstacle) and the area that 
 * the cone is spanning.
 * 
 * Parameters:
 * 		prevCellValue -- ...
 * 		mode -- 'w' if you are setting a hollow space (between the sensor minRange and the obstacle)
 * 				'b' if you are setting the area where the obstacle is
 * 		area -- The area of the circular sector you are painting with these rays
 * 				(considering that the probability that an obstacle is in a certain pixel is 
 * 				inversely proportional to the area of the circular sector we are covering with the 
 * 				sensor (Area = pi*(R^2-r^2)*fov/2pi).
 * 		pixWidth -- Resolution of the map
 * 
 **************************************************************************************************/
unsigned int RangeSensor::updateCell(int prevCellValue, char mode, float sectorArea, float pixWidth){
	// If the obstacle possibility area == minimum detectable area: 100% sure it's there
	float certainty = 250 * pixWidth*pixWidth / sectorArea;  // FIXME Remove '2'
	if (certainty > 1.0) certainty = 1.0;
	float newCellVal;
	
	switch (mode){
		case 'w': 	// Setting a hollow space
			//newCellVal = 255.0 - (1.0 - certainty) * (255.0 - prevCellValue);
			newCellVal = 255.0 - (0.95) * (255.0 - prevCellValue);
			break;
			
		case 'b':	// Setting the area where the obstacle is
			newCellVal = prevCellValue * (1.0 - certainty);
			//newCellVal = prevCellValue * (0.6);
			//cout << "### Certainty = " << certainty << endl;
			break;
	}
	return (unsigned int) newCellVal;
}

/***************************************************************************************************
 * updateMap:
 *
 * Use the last sensor data to update the map, given the pose of the 
 * robot.
 * 
 * Usage:
 * 		mysensor::updateMap(map_ptr, pose)
 **************************************************************************************************/
int RangeSensor::updateMap(	mr::OccupancyGridMap255 * map, 
							mr::Pose robPose	  )	
{
	// Take each sensor reading (one line of the log)
	for	(int obs_i = 0; obs_i <= (sensorBuffer.size() - 1); obs_i ++ ){
		//cout << "Starting processing obstacle " << obs_i;
		//cout << " of " << sensorBuffer.size() << ".\n";
		
		// Make sure that the line corresponds to the same sensor type as the class (using the var 
		//   sensorType)
		if (sensorBuffer[obs_i].sensorType != sensorBuffer[obs_i].sensorType){
			cout << "Obstacle " << obs_i << " is not from sensor ";
			cout << sensorID << ".\n";
			continue;
		}
		
		/*
		 * === We are now processing obs_i ===
		 */
		// We have to know how many rays do we have to draw to cover the cone where the obstacle is.
		//   This depends on how far the obstacle is and the resolution of the map.
		//   By doing this, we avoid leaving spaces between the different rays
		float mapRes = map->getResolution();
		int raysInCone = nRays(sensorBuffer[obs_i].distance , mapRes);
		if (raysInCone == 0) continue;
		
		for (int ray_i = 0; ray_i < raysInCone; ray_i++) // ray Index
		{
			/*
			 * We are now processing each one of the rays that compose
			 * the cone sector where the obstacle is.
			 */
			 
			// Relative angle between the ray we are tracing and the
			//	 direction where the sensor is pointing:
			float scanAngle = -fov / 2.0  +  fov * ray_i / raysInCone;
			
			// World (absolute) angle of the ray we are tracing:
			float rayDir = robPose.angle() + scanAngle + sensorBuffer[obs_i].angle;
						
			/*
			 * For each ray, we can distinguish 4 different zones:
			 * 	a) From the sensor position to minRange: Unknown terrain
			 * 	b) From minRangePoint to (obstacle - sigma): White (no obstacles)
			 * 	c) From (obstacle - sigma) to(obstacle + sigma): Black (Obstacle)
			 * 	d) From (obstacle + sigma) until maxRangePoint: Unknown terrain
			 * Therefore, we have to modify the map only in zones b and c.
			 */

			/*
			 * Calculating all the points where the rays start and end
			 */
			 
			// Coordinates of the sensor:
			// Vector2D sensorPos(robPose.x,robPose.y);
			Vector2D sensorPos;
			sensorPos.setPolarCoords(sensorOffset, rayDir + sensorDiphase);
			Vector2D robotCenter(robPose.x,robPose.y);
			sensorPos = sensorPos + robotCenter;
			
			
			// Relative position of minRangePoint with respect to the sensor:
			Vector2D rel_minRangePoint;
			rel_minRangePoint.setPolarCoords(minRange, rayDir);
			// Absolute position of minRangePoint:
			Vector2D minRangePoint = sensorPos + rel_minRangePoint;
			
			// Relative position of obstStart with respect to the sensor:
			Vector2D rel_obstStart;
			float sigma = getStandardDeviation(sensorBuffer[obs_i].distance);
			rel_obstStart.setPolarCoords(sensorBuffer[obs_i].distance - sigma, rayDir);
			// Absolute position of obst(acle)Start:
			Vector2D obstStart = sensorPos + rel_obstStart;
			
			// Relative position of obstEnd with respect to the sensor:
			Vector2D rel_obstEnd;
			rel_obstEnd.setPolarCoords(sensorBuffer[obs_i].distance + sigma, rayDir);
			// Absolute position of obst(acle)Start:
			Vector2D obstEnd = sensorPos + rel_obstEnd;
			
			// Relative position of maxRangePoint with respect to the sensor:
			//   (This is only needed if the obstacle is beyond maxRange)
			Vector2D rel_maxRangePoint;
			rel_maxRangePoint.setPolarCoords(maxRange, rayDir);
			// Absolute position of minRangePoint:
			Vector2D maxRangePoint = sensorPos + rel_maxRangePoint;
			
			/*
			 * Now that we have all the points, we have to trace the rays between them.
			 */
			
			// === Processing White ray ===
			
			// FIXME SegFault when the ray gets
			mr::OccupancyGridMap255::Ray ray = map->getRay(minRangePoint, obstStart);
			if (ray.size()==0) continue;
			for (int j=0; j < ray.size()-1; j++){
				// Painting each pixel of the white Ray
				unsigned char cellValue;
				// To avoid segFaults, we test if the cell we are acessing is in the map:
				if ( map->getSafeCellValue(ray[j].pixelX, ray[j].pixelY, cellValue) ){
					double area = (pow((sensorBuffer[obs_i].distance - sigma),2.0) - pow(minRange,2)*fov / 2.0);
					cellValue = updateCell( cellValue, 'w', area, map->getResolution() );
					map->setCellValue(ray[j].pixelX, ray[j].pixelY, cellValue);
				}	// Ended setting pixel
			} 	// Ended processing white part
			
			
			// === Processing Black ray ===
			 
			// If there is no obstacle in the fov of the sensor, the log will show a distance 
			//   greater than maxRange. In that case, no obstacle should be painted:
			if (sensorBuffer[obs_i].distance >= maxRange) continue;
			
			ray = map->getRay(obstStart, obstEnd);
			if (ray.size()==0) continue;
			for (int k=0; k < ray.size(); k++){
				// Painting each pixel of the black Ray
				unsigned char cellValue;
				// To avoid segFaults, we test if the cell we are acessing is in the map:
				if ( map->getSafeCellValue(ray[k].pixelX, ray[k].pixelY, cellValue) ){
					// Update cell value, considering that the probability that an obstacle is in a
					//   certain pixel is inversely proportional to the area of the circular sector
					//   we are covering with the sensor (Area = pi*(R^2-r^2)*fov/2pi)
					double area = (pow((sensorBuffer[obs_i].distance + sigma),2.0) - pow((sensorBuffer[obs_i].distance - sigma),2)*fov / 2.0);
					cellValue = updateCell( cellValue, 'b', area, map->getResolution() );
					map->setCellValue(ray[k].pixelX, ray[k].pixelY, cellValue);
				}
			} 	// Ended processing black part: ray finished
			
		} 	// Ended complete cone	
	}	// Ended whole log
	
	return 0;
}
