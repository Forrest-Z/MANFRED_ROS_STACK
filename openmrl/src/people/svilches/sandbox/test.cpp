#include <iostream>
#include <mrcore/navigation2d/occupancygridmap255.h>
#include <mrcore.h>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include "rangesensor.h"
#include <vector>


using namespace std;


vector <sensorData> loadLog(){
	string line;
	ifstream logFile ("sensor_log.dat");
	
	vector <sensorData> sensorReadings;
	
	if (logFile.is_open())
	{
		while ( logFile.good() )
		{
			getline (logFile,line);
			cout << "###" << line << endl;
			istringstream stream(line);
			float angle;
			stream >> angle;
			float distance;
			stream >> distance;
			
			cout << "Angle: " << angle;
			cout <<";\tDistance: " << distance << "\n";
			
			// Now, include this data in the vector sensorBuffer
			sensorData lineData(angle,distance,'i'); 
			sensorReadings.push_back(lineData);
		}
		logFile.close();
		
	}

  else cout << "Unable to open file";

  return sensorReadings;
}

int main () {
	vector <sensorData> sensorReadings = loadLog();
}

	


 



