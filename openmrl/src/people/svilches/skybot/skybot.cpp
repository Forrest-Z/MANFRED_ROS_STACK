#include "skybot.h"

SkyBot::SkyBot(string sPort){
	serialPort = sPort;
	robPose.x=0; robPose.y=0; robPose.theta=0;
	connectToRobot();
}

bool SkyBot::connectToRobot(){
	std::cout << "Reading info from the robot\n";
	cout.flush();
	
	// Initializing the serial port
	
	robotStream.Open(serialPort.c_str());
	robotStream.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
	robotStream.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
	robotStream.SetNumOfStopBits(1);
	robotStream.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
	robotStream.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);
	
	string rawLine;

	if(robotStream.IsOpen()) cout << "Serial port is OPEN\n";
	else exit(1);
	cout.flush();
	sleep(2);
	
	// Asking the robot for its configuration info:
	robotStream << 'i';
	
	// Reading the name of the robot
	getline (robotStream,rawLine);
	cout << "Connected to robot: " <<rawLine << endl;
	
}

bool SkyBot::updateRangeData(){
}
