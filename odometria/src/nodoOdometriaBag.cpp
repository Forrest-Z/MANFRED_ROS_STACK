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

ros::Time tiempoAnterior;
ros::Time tiempoActual;
double deltaTiempo = 0.0f;

tf::TransformBroadcaster *p_transfBroadcaster;
geometry_msgs::TransformStamped *p_transfStampedOdomBase;
ros::Publisher *p_publicadorTopicOdometria;
nav_msgs::Odometry *p_msj_odometria_bag;

double xRobotSRO = 0.0; // En m
double yRobotSRO = 0.0; // En m
double tRobotSRO = 0.0; // En rad

double xRobotSROAnt = 0.0; // En m
double yRobotSROAnt = 0.0; // En m
double tRobotSROAnt = 0.0; // En rad

double xRobotSRO_ = 0.0; // En m
double yRobotSRO_ = 0.0; // En m
double tRobotSRO_ = 0.0; // En rad

double tRef = 0.0; // En rad.

double R11 = 1;
double R12 = 0;
double R13 = 0;

double R21 = 0;
double R22 = 1;
double R23 = 0;

double R31 = 0;
double R32 = 0;
double R33 = 1;

double T11 = 0;
double T21 = 0;
double T31 = 0;

double vx = 0.0;	// En m/seg
double vy = 0.0;	// En m/seg
double vth = 0.0;	// En rad/seg

double d_ = 0.0f;
double gamma_ = 0.0f;

double gradRad = 180.0f / M_PI;

void obtenerOdometria(
		const nav_msgs::Odometry::ConstPtr& rp_msj_odometria_rx) {

	//  We'll assume that the robot starts at the origin of the "sr_odom" coordinate frame initially.
	// SRO: Sistema de referencia odometrico.

	// Coordenadas del Marco Base en el Marco Odometria original.
	xRobotSRO_ = rp_msj_odometria_rx->pose.pose.position.x;
	yRobotSRO_ = rp_msj_odometria_rx->pose.pose.position.y;
	tRobotSRO_ = tf::getYaw(rp_msj_odometria_rx->pose.pose.orientation);

	// Coordenadas del Marco Base en el Marco odometria guardado.
	xRobotSRO = (R11 * xRobotSRO_) + (R12 * yRobotSRO_) + T11;
	yRobotSRO = (R21 * xRobotSRO_) + (R22 * yRobotSRO_) + T21;
	tRobotSRO = tRobotSRO_ - tRef;

	tiempoActual = ros::Time::now();
	deltaTiempo = (tiempoActual - tiempoAnterior).toSec();

	vx = (xRobotSRO - xRobotSROAnt) / deltaTiempo;
	vy = (yRobotSRO - yRobotSROAnt) / deltaTiempo;
	vth = (tRobotSRO - tRobotSROAnt) / deltaTiempo;

	printf("xRobotSRO_: %f m \n", xRobotSRO_);
	printf("yRobotSRO_: %f m \n", yRobotSRO_);
	printf("tRobotSRO_: %f grad \n", gradRad * tRobotSRO_);

	printf("xRobotSRO: %f m \n", xRobotSRO);
	printf("yRobotSRO: %f m \n", yRobotSRO);
	printf("tRobotSRO: %f grad \n", gradRad * tRobotSRO);

	printf("vx: %f m/s \n", vx);
	printf("vy: %f m/s \n", vy);
	printf("vth: %f grad/s \n", gradRad * vth);

	xRobotSROAnt = xRobotSRO;
	yRobotSROAnt = yRobotSRO;
	tRobotSROAnt = tRobotSRO;
	tiempoAnterior = tiempoActual;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
			tRobotSRO);

	p_transfStampedOdomBase->header.stamp = tiempoActual;

	// La odometria esta calculada en 'centimetros' y en ROS las distancias
	// deben estar expresdas en 'metros' y los angulos en radianes.
	p_transfStampedOdomBase->transform.translation.x = xRobotSRO;
	p_transfStampedOdomBase->transform.translation.y = yRobotSRO;
	// Altura desde el sist.ref.odom (odom, en el suelo) hasta el sist.ref.base (base).
	// Radio de la rueda de motriz de Manfred 75 mm.
	p_transfStampedOdomBase->transform.translation.z = 0.075;
	p_transfStampedOdomBase->transform.rotation = odom_quat;

	p_transfBroadcaster->sendTransform(*p_transfStampedOdomBase);

	p_msj_odometria_bag->header.stamp = tiempoActual;
	p_msj_odometria_bag->pose.pose.position.x = xRobotSRO;
	p_msj_odometria_bag->pose.pose.position.y = yRobotSRO;
	p_msj_odometria_bag->pose.pose.position.z = 0.075;
	p_msj_odometria_bag->pose.pose.orientation = odom_quat;

	p_msj_odometria_bag->twist.twist.linear.x = vx;
	p_msj_odometria_bag->twist.twist.linear.y = vy;
	p_msj_odometria_bag->twist.twist.angular.z = vth;

	p_publicadorTopicOdometria->publish(*p_msj_odometria_bag);

}

void resetearOdometria(const std_msgs::Bool::ConstPtr& pBool) {

	// El vector de coordenadas del Marco Base en el Marco odometria original en el
	// momento de ejecutar la funcion 'resetearOdometria' se convierte en
	// el vector de coordenadas del Marco odometria a guardar (que es otro Marco diferente del
	// Marco odometria original). Asi a partir del instante en que la funcion resetearOdometria
	// termina se puede calcular el vector de coordenadas del Marco Base con respecto
	// al Marco odometria guardado.
	// P.D: Cuando la funcion resetearOdometria termina el vector de coordenadas
	// del Marco Base en el Marco Odometria Guardado sera nulo o prácticamente nulo (si
	// la base se ma movido ligeramente con respecto a este nuevo Marco).
	printf("\n");
	printf("\x1B[31;1m");
	printf("Resetear odometria \n");
	printf("\x1B[0m");

	R11 = cos(-tRobotSRO_);
	R12 = -sin(-tRobotSRO_);

	R21 = -R12;
	R22 = R11;

	d_ = hypot(xRobotSRO_, yRobotSRO_);
	gamma_ = -tRobotSRO_ + atan2(-yRobotSRO_, -xRobotSRO_);

	T11 = d_ * cos(gamma_);
	T21 = d_ * sin(gamma_);

	tRef = tRobotSRO_;

	// Coordenadas del Marco Base en el Marco odometria guardado.
	xRobotSROAnt = (R11 * xRobotSRO_) + (R12 * yRobotSRO_) + T11;
	yRobotSROAnt = (R21 * xRobotSRO_) + (R22 * yRobotSRO_) + T21;
	tRobotSROAnt = tRobotSRO_ - tRef;

	tiempoAnterior = ros::Time::now();

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "nodoOdometriaProcesarBag");
	ros::NodeHandle nodoManejador;
	ros::Subscriber suscriptorResetOdometria = nodoManejador.subscribe(
			"topic_resetOdometria", 1, resetearOdometria);
	ros::Subscriber suscriptorTopicOdometria = nodoManejador.subscribe(
			"topic_odometria", 1, obtenerOdometria);
	ros::Publisher publicadorTopicOdometria = nodoManejador.advertise<
			nav_msgs::Odometry>("topic_odometria_OK", 1, true);
	p_publicadorTopicOdometria = &publicadorTopicOdometria;
	nav_msgs::Odometry msj_odometria_bag;
	msj_odometria_bag.header.frame_id = "link_odom";
	msj_odometria_bag.child_frame_id = "link_base";
	p_msj_odometria_bag = &msj_odometria_bag;
	tf::TransformBroadcaster transfBroadcaster;
	p_transfBroadcaster = &transfBroadcaster;
	geometry_msgs::TransformStamped transfStampedOdomBase;
	// The pose in this message should be specified in the coordinate frame given by header.frame_id
	transfStampedOdomBase.header.frame_id = "link_odom";
	// The twist in this message should be specified in the coordinate frame given by the child_frame_id
	// We'll set the child_frame_id of the message to be the "base_link" frame since that's the coordinate
	// frame we're sending our velocity information in.
	transfStampedOdomBase.child_frame_id = "link_base";
	p_transfStampedOdomBase = &transfStampedOdomBase;
	ros::spin();
	return 0;
}
