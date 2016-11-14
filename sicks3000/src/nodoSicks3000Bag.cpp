#include <cerrno>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "sicks3000/sicks3000.h"

ros::Publisher *p_publicadorSicks3000 = NULL;

void procesarBarrido(const sensor_msgs::LaserScan::ConstPtr& rp_barrido_laser_rx)
{

  sensor_msgs::LaserScan barrido_laser_sicks3000 = *rp_barrido_laser_rx;
  barrido_laser_sicks3000.header.stamp = ros::Time::now();
  barrido_laser_sicks3000.header.frame_id = "link_laser";
  p_publicadorSicks3000->publish(barrido_laser_sicks3000);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nodoSicks3000");
  ros::NodeHandle nodoManejador;
  ros::Subscriber suscriptorSicks3000 = nodoManejador.subscribe("topic_sicks3000", 1, procesarBarrido);
  ros::Publisher publicadorSicks3000 = nodoManejador.advertise<sensor_msgs::LaserScan>("topic_sicks3000_OK", 1, true);
  p_publicadorSicks3000 = &publicadorSicks3000;
  ros::spin();
  return 0;
}
