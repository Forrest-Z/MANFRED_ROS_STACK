/**
 * This file contains methods and other stuff belonging to the Joystick class
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <sstream>
#include <cstring>
#include <unistd.h>

#include "joystick.h"

using namespace std;

const char *Joystick::defaultDevice = "/dev/input/js0";

void Joystick::initDevice() throw (string)
{
	int a;
	unsigned char c;
	char s[255];
	ostringstream oss;
	fd = open(devName.c_str(), O_RDONLY);
	if (fd < 0) {
		throw string("Unable to open device ") + devName + ": " +
			strerror(errno);
	}
	ioctl(fd, JSIOCGVERSION, &a);
	oss << (a >> 16) << "." <<  ((a >> 8) & 0xff) << "." << (a & 0xff);
	driverVersion = oss.str();
	ioctl(fd, JSIOCGAXES, &c);
	axesNumber = c;
	axes = new int[axesNumber];
	ioctl(fd, JSIOCGBUTTONS, &c);
	buttonsNumber = c;
	buttons = new char[buttonsNumber];
	ioctl(fd, JSIOCGNAME(255), &s);
	identifierString = s;
	ioctl(fd, JSIOCGAXMAP, axisMapping);
	ioctl(fd, JSIOCGBTNMAP, buttonMapping);
}

bool Joystick::dataReady() throw (string)
{
	struct timeval timeout;
	int a;
	bool retval = true;
	fd_set readfds;
	// Arrigo hates select()
	FD_ZERO(&readfds);
	FD_SET(fd, &readfds);
	// Timeout 0 means "don't wait"
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;
	a = select(fd + 1, &readfds, NULL, NULL, &timeout);
	if (a == 0) {
		retval = false;
	} else if (a < 0) {
		throw string("error in select(): ") + strerror(errno);
	}
	return retval;
}

void Joystick::refresh() throw (string)
{
	struct js_event event;
	ostringstream oss;
	//int dataRead = 0, dataToRead = getButtonsNumber() + getAxesNumber();
	// We can just read one event for each button or axis. 
	// But some of them may be blocking.
	while (dataReady()) {
		if (read(fd, &event, sizeof(event)) != sizeof(event)) {
			throw string("Unable to read events: ") + strerror(errno);
		}
		//dataRead++;
		// JS_EVENT_INIT could be superimposed to event information
		switch (event.type & ~JS_EVENT_INIT) {
		case JS_EVENT_BUTTON:
			buttons[event.number] = event.value;
			break;
		case JS_EVENT_AXIS:
			axes[event.number] = event.value;
			break;
		default: // Should happen only if driver changes
			oss << "Unknown event type: " << event.type;
			throw oss.str();
		}
	}
	//else printf("No data to be read\n");
}

int Joystick::getAxis(unsigned int axis) throw (string)
{
	if (axis >= axesNumber) {
		ostringstream oss;
		oss << "Wrong axis number specified: " << axis;
		throw oss.str();
	}
	return axes[axis];
}

bool Joystick::getButton(unsigned int button) throw (string)
{
	if (button >= buttonsNumber) {
		ostringstream oss;
		oss << "Wrong button number specified: " << button;
		throw oss.str();
	}
	return buttons[button];
}

void Joystick::getAxes(vector<int> &axesData)
{
	unsigned int i;
	axesData.resize(axesNumber);
	for (i = 0; i < axesNumber; i++) {
		axesData[i] = axes[i];
	}
}

void Joystick::getButtons(vector<bool> &buttonsData)
{
	unsigned int i;
	buttonsData.resize(buttonsNumber);
	for (i = 0; i < buttonsNumber; i++) {
		buttonsData[i] = buttons[i];
	}
}

Joystick::Joystick() throw (string): devName(defaultDevice)
{
	initDevice();
}

Joystick::Joystick(const string &device) throw(string): devName(device)
{
	initDevice();
}

Joystick::~Joystick() throw(string)
{
	int a;
	a = close(fd);
	if (a < 0) {
		throw string("Unable to close device ") + devName + ": " +
			strerror(errno);
	}
	delete [] buttons;
	delete [] axes;
}
