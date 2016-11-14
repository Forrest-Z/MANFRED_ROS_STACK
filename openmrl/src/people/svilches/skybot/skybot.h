#ifndef SKYBOT_H
#define SKYBOT_H

#include "robot.h"
#include <SerialStream.h>
#include <stdlib.h>

class SkyBot: public Robot{
	public:
		SkyBot(string sPort);
		~SkyBot(){};
		
		LibSerial::SerialStream robotStream;		
		string serialPort;

		bool connectToRobot();
		bool updateRangeData();
		
};


#endif
