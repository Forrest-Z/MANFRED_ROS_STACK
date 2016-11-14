#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include "pmac.h"
#include "odometria/servicioOdometria.h"

using std::cout;

using std::endl;

ros::Time tiempoAnterior;

ros::Time tiempoActual;

bool odometriaRX = false;

double x_mbase_modom_ant = 0.0f;

double y_mbase_modom_ant = 0.0f;

double t_mbase_modom_ant = 0.0f;

double gradRad = 180.0f / M_PI;

void mostrarOdometria(const nav_msgs::Odometry::ConstPtr& rp_msj_odometria_rx)
{

  odometriaRX = true;

  double x_mbase_modom = rp_msj_odometria_rx->pose.pose.position.x;

  double y_mbase_modom = rp_msj_odometria_rx->pose.pose.position.y;

  double t_mbase_modom = tf::getYaw(rp_msj_odometria_rx->pose.pose.orientation);

  // Coordenadas del Marco Base en el Marco Odometria original.
  cout << "x_mbase_modom: " << x_mbase_modom << " m" << endl;

  cout << "y_mbase_modom: " << y_mbase_modom << " m" << endl;

  cout << "z_mbase_modom: " << gradRad * t_mbase_modom << " grad" << endl;

  tiempoActual = ros::Time::now();

  double deltaTiempo = (tiempoActual - tiempoAnterior).toSec();

  double vx = (x_mbase_modom - x_mbase_modom_ant) / deltaTiempo;

  double vy = (y_mbase_modom - y_mbase_modom_ant) / deltaTiempo;

  double vt = (t_mbase_modom - t_mbase_modom_ant) / deltaTiempo;

  tiempoAnterior = tiempoActual;

  x_mbase_modom_ant = x_mbase_modom;

  y_mbase_modom_ant = y_mbase_modom;

  t_mbase_modom_ant = t_mbase_modom;

  cout << "vx: " << vx << " m/s" << endl;

  cout << "vy: " << vy << " m/s" << endl;

  cout << "vt: " << gradRad * vt << " grad/s" << endl;

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "nodoOdometriaPrueba");

  ros::NodeHandle nodoManejador;

  ros::Subscriber suscriptorTopicOdometria = nodoManejador.subscribe("topic_odometria", 1, mostrarOdometria);

  int descriptorDPRAM = 0;

  if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1)
  {

    printf("No se puede abrir %s.\n", DPRAM);

    return -1;

  }

  resetearOdometriaPMAC(descriptorDPRAM);

  ros::Rate frec(1);

  tiempoAnterior = ros::Time::now();

  while (1)
  {

    do
    {

      ros::spinOnce();

    } while (!odometriaRX);

    odometriaRX = false;

    frec.sleep();

  }

  return 0;
}
