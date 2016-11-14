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

//#define REPRODUCIR_BAG
#define SIMULACION

ros::Time tiempoAnterior;
nav_msgs::Odometry msjOdometria;
float xRobotSROAnt = 0.0; // En m
float yRobotSROAnt = 0.0; // En m
float tRobotSROAnt = 0.0; // En rad
float xRobotSRO = 0.0; // En m
float yRobotSRO = 0.0; // En m
float tRobotSRO = 0.0; // En rad

double gradRad = 180.0f / M_PI;

#ifdef SIMULACION
double radio = 5.00; // En  m.
// Angulo inicial de la base en el sist.ref.odom. En rad.
double anguloInicial = 0.0;
double anguloRecorrido = anguloInicial;
double velocidadAngular = M_PI / 18.0f; //En rad/s. 10 grad/s ==> M_PI/18
#endif

#ifdef REPRODUCIR_BAG
void escucharPruebaOffline(
		const nav_msgs::Odometry::ConstPtr& refPtrMsjOdometriaRX) {
	msjOdometria = *refPtrMsjOdometriaRX;
	std::cout << "Recibo topic odometria" << std::endl;
}
#endif

bool servirOdometria(odometria::servicioOdometria::Request& req,
		odometria::servicioOdometria::Response& res) {
	res.msjOdometria = msjOdometria;
	std::cout << "Sirvo odometria" << std::endl;
	return true;
}

void resetearOdometria(const std_msgs::Bool::ConstPtr& pBool) {

	printf("\n");
	printf("\x1B[31;1m");
	printf("Resetear odometria \n");
	printf("\x1B[0m");

	tiempoAnterior = ros::Time::now();

#ifdef SIMULACION
	anguloRecorrido = anguloInicial;
	xRobotSROAnt = radio;
	yRobotSROAnt = 0.0; // En m
	tRobotSROAnt = anguloRecorrido + M_PI / 2; // En rad

	printf("xRobotSROAnt: %f m\n", xRobotSROAnt);
	printf("yRobotSROAnt: %f m\n", yRobotSROAnt);
	printf("tRobotSROAnt: %f grad\n", gradRad * tRobotSROAnt);

#endif

}

int main(int argc, char** argv) {
#ifndef REPRODUCIR_BAG
#ifndef SIMULACION
	int descriptorDPRAM = 0;
	if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1) {
		printf("No se puede abrir %s.\n", DPRAM);
		return -1;
	}
#endif
#endif
	ros::init(argc, argv, "nodoOdometria");
	ros::NodeHandle nodoManejador;
	ros::ServiceServer servidorServicioOdometria =
			nodoManejador.advertiseService("servicioOdometria",
					servirOdometria);

	ros::Subscriber suscriptorResetOdometria = nodoManejador.subscribe(
			"topic_resetOdometria", 1, resetearOdometria);

#ifdef REPRODUCIR_BAG
	std::cout << "Reproduciendo datos de un bag" << std::endl;
	// TOPIC_ODOMETRY PORQUE ESTOY HACIENDO PRUEBAS CON BAGS EN LOS QUE EL TOPIC DEL NODO
	// ODOMETRIA HA SIDO GRABADO CON EL NOMBRE 'TOPIC_ODOMETRY'
	ros::Subscriber suscriptorOdometria = nodoManejador.subscribe(
			"topic_odometria", 1, escucharPruebaOffline);
	ros::spin();
#else

	// The advertise() function is how you tell ROS that you want to
	// publish on a given topic name. This invokes a call to the ROS
	// master node, which keeps a registry of who is publishing and who
	// is subscribing. After this advertise() call is made, the master
	// node will notify anyone who is trying to subscribe to this topic name,
	// and they will in turn negotiate a peer-to-peer connection with this
	// node.  advertise() returns a Publisher object which allows you to
	// publish messages on that topic through a call to publish().  Once
	// all copies of the returned Publisher object are destroyed, the topic
	// will be automatically unadvertised.
	// The second parameter to advertise() is the size of the message queue
	// used for publishing messages.  If messages are published more quickly
	// than we can send them, the number here specifies how many messages to
	// buffer up before throwing some away.

	ros::Publisher publicadorTopicOdometria = nodoManejador.advertise<
			nav_msgs::Odometry>("topic_odometria", 1);
	// We need to create both a ros::Publisher and a tf::TransformBroadcaster to be able to send messages
	// out using ROS and tf respectively.
	tf::TransformBroadcaster transfBroadcaster;
	geometry_msgs::TransformStamped transfStampedOdomBase;

	//  We'll assume that the robot starts at the origin of the "sr_odom" coordinate frame initially.
	// SRO: Sistema de referencia odometrico.

	float deltaX = 0;	// En m
	float deltaY = 0;	// En m
	float deltaTheta = 0;	// En rad

	double deltaTiempo = 0;	// En seg
	double vx = 0.0;	// En m/seg
	double vy = 0.0;	// En m/seg
	double vth = 0.0;	// En rad/seg

	ros::Time tiempoActual;

	// Frecuencia: Ejecucion cada 1 s (1 Hz)
	ros::Rate tasa(1);

#ifndef SIMULACION
	if (leerUbicacionRobotSRO(descriptorDPRAM, &xRobotSROAnt, &yRobotSROAnt,
					&tRobotSROAnt)) {
		ROS_ERROR("Error al leer la ubicación del robot en el SRO.\n");
	}
#else
	xRobotSROAnt = radio;
	yRobotSROAnt = 0;
	tRobotSROAnt = anguloRecorrido + M_PI / 2;
#endif

	tiempoAnterior = ros::Time::now();
	// The pose in this message should be specified in the coordinate frame given by header.frame_id
	transfStampedOdomBase.header.frame_id = "link_odom";
	// The twist in this message should be specified in the coordinate frame given by the child_frame_id
	// We'll set the child_frame_id of the message to be the "base_link" frame since that's the coordinate
	// frame we're sending our velocity information in.
	transfStampedOdomBase.child_frame_id = "link_base";

	msjOdometria.header.frame_id = "link_odom";
	msjOdometria.child_frame_id = "link_base";

	while (nodoManejador.ok()) {

#ifndef SIMULACION  
		if (leerUbicacionRobotSRO(descriptorDPRAM, &xRobotSRO, &yRobotSRO,
						&tRobotSRO)) {
			ROS_ERROR("Error al leer la ubicación del robot en el SRO.\n");
		}
#endif

		tiempoActual = ros::Time::now();
		deltaTiempo = (tiempoActual - tiempoAnterior).toSec();
		printf("\n");
		printf("Tiempo anterior: %f \n", tiempoAnterior.toSec());
		printf("Tiempo actual: %f \n", tiempoActual.toSec());
		printf("deltaTiempo: %f \n", deltaTiempo);

#ifdef SIMULACION

		anguloRecorrido += velocidadAngular * deltaTiempo;
		xRobotSRO = radio * cos(anguloRecorrido);
		yRobotSRO = radio * sin(anguloRecorrido);
		tRobotSRO = anguloRecorrido + M_PI / 2;
		printf("anguloRecorrido: %f grad\n", gradRad * anguloRecorrido);
#endif

		deltaX = xRobotSRO - xRobotSROAnt;
		deltaY = yRobotSRO - yRobotSROAnt;
		deltaTheta = tRobotSRO - tRobotSROAnt;
		vx = deltaX / deltaTiempo;
		vy = deltaY / deltaTiempo;
		vth = deltaTheta / deltaTiempo;
		printf("deltaX: %f m/s \n", deltaX);
		printf("deltaY: %f m/s \n", deltaY);
		printf("deltaTheta: %f grad \n", gradRad * deltaTheta);
		printf("xRobotSRO: %f m \n", xRobotSRO);
		printf("yRobotSRO: %f m \n", yRobotSRO);
		printf("tRobotSRO: %f grad \n", gradRad * tRobotSRO);
		printf("VX: %f m/s \n", vx);
		printf("VY: %f m/s \n", vy);
		printf("VTHETA: %f grad/s \n", gradRad * vth);

		tiempoAnterior = tiempoActual;
		xRobotSROAnt = xRobotSRO;
		yRobotSROAnt = yRobotSRO;
		tRobotSROAnt = tRobotSRO;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
				tRobotSRO);
		// This expresses a transform from coordinate frame 'header.frame_id'  to the coordinate
		// frame 'child_frame_id'
		transfStampedOdomBase.header.stamp = tiempoActual;

		// La odometria esta calculada en 'centimetros' y en ROS las distancias
		// deben estar expresdas en 'metros' y los angulos en radianes.
		transfStampedOdomBase.transform.translation.x = xRobotSRO;
		transfStampedOdomBase.transform.translation.y = yRobotSRO;
		// Altura desde el sist.ref.odom (odom, en el suelo) hasta el sist.ref.base (base).
		// Radio de la rueda de motriz de Manfred 75 mm.
		transfStampedOdomBase.transform.translation.z = 0.075;
		transfStampedOdomBase.transform.rotation = odom_quat;

		transfBroadcaster.sendTransform(transfStampedOdomBase);

		msjOdometria.header.stamp = tiempoActual;
		msjOdometria.pose.pose.position.x = xRobotSRO;
		msjOdometria.pose.pose.position.y = yRobotSRO;
		msjOdometria.pose.pose.position.z = 0.075;
		msjOdometria.pose.pose.orientation = odom_quat;

		msjOdometria.twist.twist.linear.x = vx;
		msjOdometria.twist.twist.linear.y = vy;
		msjOdometria.twist.twist.angular.z = vth;

		publicadorTopicOdometria.publish(msjOdometria);

		ros::spinOnce();
		tasa.sleep();
	}
#endif
	return 0;
}
