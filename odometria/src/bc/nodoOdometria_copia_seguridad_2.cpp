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

const double gradRad = 180.0f / M_PI;

float xRobotSROAnt = 0.0; // En m
float yRobotSROAnt = 0.0; // En m
float tRobotSROAnt = 0.0; // En rad

ros::Time tiempoAnterior;


/*

bool servirOdometria(odometria::servicioOdometria::Request& req,
		odometria::servicioOdometria::Response& res) {
	res.msjOdometria = *p_msjOdometriaBag;
	std::cout << "Sirvo odometria" << std::endl;
	return true;
}
*/

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

	// Coordenadas del Marco Base en el Marco odometria guardado.
	xRobotSROAnt = 0;
	yRobotSROAnt = 0;
	tRobotSROAnt = 0;

	tiempoAnterior = ros::Time::now();

}

/*
bool servirOdometria(odometria::servicioOdometria::Request& req,
		odometria::servicioOdometria::Response& res) {
	res.msjOdometria = msjOdometria;
	std::cout << "Sirvo odometria" << std::endl;
	return true;
}
*/

int main(int argc, char** argv) {


	int descriptorDPRAM = 0;
	if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1) {
		printf("No se puede abrir %s.\n", DPRAM);
		return -1;
	}

	ros::init(argc, argv, "nodoOdometria");
	ros::NodeHandle nodoManejador;
	/*
	ros::ServiceServer servidorServicioOdometria =
			nodoManejador.advertiseService("servicioOdometria",
					servirOdometria);
					*/

	ros::Subscriber suscriptorResetOdometria = nodoManejador.subscribe(
			"topic_resetOdometria", 1, resetearOdometria);

	ros::Publisher publicadorTopicOdometria = nodoManejador.advertise<
	nav_msgs::Odometry>("topic_odometria", 1, true);
	
	nav_msgs::Odometry msjOdometria;

	// We need to create both a ros::Publisher and a tf::TransformBroadcaster to be able to send messages
	// out using ROS and tf respectively.
	tf::TransformBroadcaster transfBroadcaster;
	geometry_msgs::TransformStamped transfStampedOdomBase;

	double radioConSigno = 0.0f;
	double cuerda = 0.0f;
	double phi = 0.0f;
	
	double deltaX = 0;// En m
	double deltaY = 0;// En m
	double deltaTheta = 0;// En rad
	
	double xRobotSRO = 0;
	double yRobotSRO = 0;
	double tRobotSRO = 0;

	double vx = 0.0;// En m/seg
	double vy = 0.0;// En m/seg
	double vth = 0.0;// En rad/seg
	
	double deltaTiempo = 0;// En seg

	ros::Time tiempoActual;

	// Frecuencia: Ejecucion cada 200 ms (50 Hz)
	ros::Rate tasa(50);

	// The pose in this message should be specified in the coordinate frame given by header.frame_id
	transfStampedOdomBase.header.frame_id = "link_odom";
	// The twist in this message should be specified in the coordinate frame given by the child_frame_id
	// We'll set the child_frame_id of the message to be the "base_link" frame since that's the coordinate
	// frame we're sending our velocity information in.
	transfStampedOdomBase.child_frame_id = "link_base";

	msjOdometria.header.frame_id = "link_odom";
	msjOdometria.child_frame_id = "link_base";

	tiempoAnterior = ros::Time::now();

	while (nodoManejador.ok()) {

		if (obtenerAnguloRecorridoRsig(descriptorDPRAM, &deltaTheta,
						&radioConSigno)) {
			printf("\x1B[31;1m");
			printf("Error leyendo datos de odometria.\n");
			printf("\x1B[0m");

		}

		tiempoActual = ros::Time::now();
		deltaTiempo = (tiempoActual - tiempoAnterior).toSec();

		printf("\n");
		printf("Tiempo anterior: %f \n", tiempoAnterior.toSec());
		printf("Tiempo actual: %f \n", tiempoActual.toSec());
		printf("deltaTiempo: %f \n", deltaTiempo);

		printf("Radio con signo: %f \n", radioConSigno);
		printf("deltaTheta: %f \n", deltaTheta);

		// Si deltaTheta = 0 el robot se ha movido en linea recta y en la variable
		// 'radioConSigno' se tiene la distancia recorrida en linea recta.
		// NOTA: Esta situacion probablemente no ocurra nunca pues aunque
		// visualmente el robot parece moverse en linea recta, el numero de
		// cuentas registradas por los encoders de las ruedas, en la PMAC, es
		// distinto, provocando que deltaTheta, aunque pequeño, no sea 0.
		if (!deltaTheta) {

			deltaX = radioConSigno * cos(tRobotSROAnt);
			deltaY = radioConSigno * sin(tRobotSROAnt);

		} else {

			cuerda = 2 * radioConSigno * sin(deltaTheta / 2);
			phi = tRobotSROAnt + (deltaTheta / 2);

			printf("Cuerda: %f \n", cuerda);
			printf("Phi: %f \n", phi);

			deltaX = cuerda * cos(phi);
			deltaY = cuerda * sin(phi);
		}

		printf("deltaX: %f m/s \n", deltaX);
		printf("deltaY: %f m/s \n", deltaY);

		xRobotSRO = xRobotSROAnt + deltaX;
		yRobotSRO = yRobotSROAnt + deltaY;
		tRobotSRO = tRobotSROAnt + deltaTheta;

		printf("xRobotSRO: %f m \n", xRobotSRO);
		printf("yRobotSRO: %f m \n", yRobotSRO);
		printf("tRobotSRO: %f grad \n", gradRad * tRobotSRO);

		vx = deltaX / deltaTiempo;
		vy = deltaY / deltaTiempo;
		vth = deltaTheta / deltaTiempo;

		printf("vx: %f m/s \n", vx);
		printf("vy: %f m/s \n", vy);
		printf("vth: %f grad/s \n", gradRad * vth);

		// Copia de los valores actuales.
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
		// Altura desde el Marco Odometria hasta el Marco Base.
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

	return 0;
}
