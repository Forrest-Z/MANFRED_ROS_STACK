#include <iostream>
#include <SerialStream.h>
#include <stdlib.h>
#include "third-party/lmcurve.h"
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <string>

using namespace std;

/* model function: a quadratic function p0 + p1 * (t-p2)^2 */
double f( double mV, const double *p )
{
    return p[0] * pow(mV, -p[1]);
}

int main(){
	LibSerial::SerialStream robotStream;	
	string serialPort = "/dev/ttyUSB0";

	cout << "Reading info from the robot\n";
	
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
	
	cout << "Handshaking..." << endl;
	
	// Asking the robot for its configuration info:
	robotStream << 'i';
	
	// Reading the name of the robot
	getline (robotStream,rawLine);
	cout << "Connected to robot: " <<rawLine << endl;
	
	double distances[] = {0.10, 0.12, 0.15, 0.18, 0.25, 0.30, 0.40, 0.50, 0.60, 0.80};
	double readings[10];
	const int distSize = 10;	// = sizeof(distances) / sizeof(distances[0]);
	

	cout << "Place an obstacle at " << distances[0] << "m and enter 'y' when you are ready\n";
	string blah;
	cin >> blah;

	for (int i=0; i< distSize; i++){
		bool repeat;
		do{
			repeat = 0;
			robotStream << 'm';
			getline (robotStream,rawLine);
		
			istringstream iss (rawLine,istringstream::in);
			iss >> readings[i];
		
			cout << "Sensor output: " << readings[i] << "mV." << endl;
			cout << "Choose an option: " << endl;
			if (i < (distSize-1)){
				cout << "\t(n) Make the next measurement at " << distances[i+1] << "m.\n";
				}
			else cout << "\t(n) FINISH\n";
			cout << "\t(r) Repeat this last measurement\n";
			
			char option;
			cin >> option;
			cout << endl;
			
			switch(option){
				case 'n':{
					break;
				}
				case 'r':{
					repeat = 1;
				}
			}
		}
		while(repeat);
		
		
	}

    /* parameter vector */

    int n_par = 2; // number of parameters in model function f
    double par[2] = { 1, 1}; // relatively bad starting value

    int m_dat = distSize; // number of data pairs
    int i;

    /* auxiliary parameters */
    lm_status_struct status;
    lm_control_struct control = lm_control_double;
    control.printflags = 0; // monitor status (+1) and parameters (+2)

    /* perform the fit */

    printf( "Fitting...\n" );
    lmcurve_fit( n_par, par, m_dat, readings, distances, f, &control, &status );
    printf("Obtained parameters:\n");
    cout << "\talpha = " << par[0] << endl;  
    cout << "\tbeta = " << par[1] << endl;
    
    cout << "Fitted equation:" << endl;
    cout << "\tDistance(m) = " << par[0] << " * mV ^ -" << par[1] << endl;
	
	char option;
	cout << "Do you want to make some test measurements to test the fitting?" << endl;
	cout << "(y) Yes" << endl;
	cout << "(n) No, exit the program"<< endl;
	cin >> option;
	
	while (option == 'y'){
		double mV;
		robotStream << 'm';
		getline (robotStream,rawLine);
		
		istringstream iss (rawLine,istringstream::in);
		iss >> mV;
		double dist = par[0] * pow(mV,-par[1]);
		cout << "Distance to the object: " << dist << "m.\n";
		cout << "Another one?\n";
		cin >> option;
	}	
	

	return 0;
	
}

