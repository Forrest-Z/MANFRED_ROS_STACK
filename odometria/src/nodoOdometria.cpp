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

#define DEBUG

int descriptorDPRAM = 0;

const double gradRad = 180.0f / M_PI;

float x_mbase_modom_ant = 0.0; // En m
float y_mbase_modom_ant = 0.0; // En m
float t_mbase_modom_ant = 0.0; // En rad

ros::Time tiempoAnterior;

void resetearOdometria(const std_msgs::Bool::ConstPtr& pBool)
{

  // El vector de coordenadas del Marco Base en el Marco odometria original en el
  // momento de ejecutar la funcion 'resetearOdometria' se convierte en
  // el vector de coordenadas del Marco odometria a guardar (que es otro Marco diferente del
  // Marco odometria original). Asi a partir del instante en que la funcion resetearOdometria
  // termina se puede calcular el vector de coordenadas del Marco Base con respecto
  // al Marco odometria guardado.
  // P.D: Cuando la funcion resetearOdometria termina el vector de coordenadas
  // del Marco Base en el Marco Odometria Guardado sera nulo o prácticamente nulo (si
  // la base se ma movido ligeramente con respecto a este nuevo Marco).
#ifdef DEBUG
  printf("\n");
  printf("\x1B[31;1m");
  printf("Resetear odometria \n");
  printf("\x1B[0m");
#endif

  tiempoAnterior = ros::Time::now();
  resetearOdometriaPMAC(descriptorDPRAM);
  // Coordenadas del Marco Base en el Marco odometria guardado.
  x_mbase_modom_ant = 0;
  y_mbase_modom_ant = 0;
  t_mbase_modom_ant = 0;
}

int main(int argc, char** argv)
{

  if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1)
  {
    printf("No se puede abrir %s.\n", DPRAM);
    return -1;
  }
  ros::init(argc, argv, "nodoOdometria");
  ros::NodeHandle nodoManejador;
  ros::Subscriber suscriptorResetOdometria = nodoManejador.subscribe("topic_resetOdometria", 1, resetearOdometria);
  ros::Publisher publicadorTopicOdometria = nodoManejador.advertise<nav_msgs::Odometry>("topic_odometria", 1, true);
  nav_msgs::Odometry msjOdometria;
  tf::TransformBroadcaster transfBroadcaster;
  geometry_msgs::TransformStamped transfStampedOdomBase;

  resetearOdometriaPMAC(descriptorDPRAM);

  float x_mbase_modom = 0;
  float y_mbase_modom = 0;
  float t_mbase_modom = 0;

  double vx = 0.0;	// En m/seg
  double vy = 0.0;	// En m/seg
  double vth = 0.0;	// En rad/seg

  double deltaTiempo = 0;	// En seg

  ros::Time tiempoActual;

  // The pose in this message should be specified in the coordinate frame given by header.frame_id
  transfStampedOdomBase.header.frame_id = "link_odom";
  // The twist in this message should be specified in the coordinate frame given by the child_frame_id
  // We'll set the child_frame_id of the message to be the "base_link" frame since that's the coordinate
  // frame we're sending our velocity information in.
  transfStampedOdomBase.child_frame_id = "link_base";

  msjOdometria.header.frame_id = "link_odom";
  msjOdometria.child_frame_id = "link_base";

  std::string parametro;
  if (!nodoManejador.hasParam("/VelocidadRuedaMaximaMpS"))
  {
    return -1;
  }
  nodoManejador.getParam("/VelocidadRuedaMaximaMpS", parametro);

  // Frecuencia:
  // VelocidadRuedaMaximaMpS en m/s ==> 100 * VelocidadRuedaMaximaMpS en cm/s
  // 1 cm = (100*VelocidadRuedaMaximaMpS) * t
  // t = 1 cm / (100*VelocidadRuedaMaximaMpS);
  // f = 1 / t = (100*VelocidadRuedaMaximaMpS) / 1 cm = 100 * VelocidadRuedaMaximaMpS;
  ros::Rate tasa(100 * atof(parametro.c_str()));

#ifdef DEBUG

  std::cout << "Frecuencia " << 100 * atof(parametro.c_str()) << " Hz" << std::endl;

#endif

  tiempoAnterior = ros::Time::now();

  while (nodoManejador.ok())
  {
    if (obtenerOdometria(descriptorDPRAM, &x_mbase_modom, &y_mbase_modom, &t_mbase_modom))
    {
      printf("\x1B[31;1m");
      printf("Error leyendo datos de odometria.\n");
      printf("\x1B[0m");

    }
    tiempoActual = ros::Time::now();
    deltaTiempo = (tiempoActual - tiempoAnterior).toSec();

#ifdef DEBUG
    printf("\n");
    printf("Tiempo anterior: %f \n", tiempoAnterior.toSec());
    printf("Tiempo actual: %f \n", tiempoActual.toSec());
    printf("deltaTiempo: %f \n", deltaTiempo);
    printf("x_mbase_modom: %f m \n", x_mbase_modom);
    printf("y_mbase_modom: %f m \n", y_mbase_modom);
    printf("t_mbase_modom: %f grad \n", gradRad * t_mbase_modom);
#endif

    vx = (x_mbase_modom - x_mbase_modom_ant) / deltaTiempo;
    vy = (y_mbase_modom - y_mbase_modom_ant) / deltaTiempo;
    vth = (t_mbase_modom - t_mbase_modom_ant) / deltaTiempo;

#ifdef DEBUG
    printf("vx: %f m/s \n", vx);
    printf("vy: %f m/s \n", vy);
    printf("vth: %f grad/s \n", gradRad * vth);
#endif

    // Copia de los valores actuales.
    tiempoAnterior = tiempoActual;
    x_mbase_modom_ant = x_mbase_modom;
    y_mbase_modom_ant = y_mbase_modom;
    t_mbase_modom_ant = t_mbase_modom;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(t_mbase_modom);
    // This expresses a transform from coordinate frame 'header.frame_id'  to the coordinate
    // frame 'child_frame_id'
    transfStampedOdomBase.header.stamp = tiempoActual;
    // La odometria esta calculada en 'centimetros' y en ROS las distancias
    // deben estar expresdas en 'metros' y los angulos en radianes.
    transfStampedOdomBase.transform.translation.x = x_mbase_modom;
    transfStampedOdomBase.transform.translation.y = y_mbase_modom;
    // Altura desde el Marco Odometria hasta el Marco Base.
    // Radio de la rueda de motriz de Manfred 75 mm.
    transfStampedOdomBase.transform.translation.z = 0.075;
    transfStampedOdomBase.transform.rotation = odom_quat;
    transfBroadcaster.sendTransform(transfStampedOdomBase);
    msjOdometria.header.stamp = tiempoActual;
    msjOdometria.pose.pose.position.x = x_mbase_modom;
    msjOdometria.pose.pose.position.y = y_mbase_modom;
    msjOdometria.pose.pose.position.z = 0.075;
    msjOdometria.pose.pose.orientation = odom_quat;
    msjOdometria.twist.twist.linear.x = vx;
    msjOdometria.twist.twist.linear.y = vy;
    msjOdometria.twist.twist.angular.z = vth;
    publicadorTopicOdometria.publish(msjOdometria);
    ros::spinOnce();
    tasa.sleep();
  }
  return 0;
}
