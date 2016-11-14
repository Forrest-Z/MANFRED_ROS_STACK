#include "robot.h"
namespace mr 
{
//IMPLEMENT_MRL_OBJECT(Robot)

Robot::Robot(){
	pose.x=0; 
	pose.y=0; 
	pose.theta=0; 
	rangeData = new mr::GenericRangeData;
}

void Robot::drawGL2D() const{
	pose.drawGL2D();
}

}
