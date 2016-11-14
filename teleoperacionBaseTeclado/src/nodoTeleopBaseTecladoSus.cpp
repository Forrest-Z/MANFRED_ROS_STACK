/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "pmac.h"

int descriptorDPRAM = 0;
// En m.
float radioBase = 0.2425f;
// En m/s.
double velRuedaDerInicial = 0;
// En m/s.
double velRuedaIzqInicial = 0;
// En cm/s^2.
float acelRuedaMax = 10.0f;
// En ms.
int tiempoMediaCurvaSRuedaDer = 0;
int tiempoMediaCurvaSRuedaIzq = 0;

void func_escuchadora(const geometry_msgs::Twist::ConstPtr& cmdvel) {

	int flagActualizarVelocidades = 0;

	// Uso el PLC 3 que actualiza la velocidad de la rueda derecha, la velocidad de la rueda
	// izquierda, el tiempo de la media curva S de la rueda derecha y el tiempo de la media curva
	// S de la rueda izquierda.
	float velRuedaDerFinal = 100*(cmdvel->linear.x + (radioBase * cmdvel->angular.z)); // x100 para pasar de m a cm.
	float velRuedaIzqFinal = 100*(cmdvel->linear.x - (radioBase * cmdvel->angular.z));
	// x1000 para pasar a ms.
	tiempoMediaCurvaSRuedaDer = ceilf(
			1000 * fabs(velRuedaDerFinal - velRuedaDerInicial) / acelRuedaMax);
	tiempoMediaCurvaSRuedaIzq = ceilf(
			1000 * fabs(velRuedaIzqFinal - velRuedaIzqInicial) / acelRuedaMax);
	// La velocidad inicial del proximo tramo es igual a la velocidad final del tramo actual.
	velRuedaDerInicial = velRuedaDerFinal;
	velRuedaIzqInicial = velRuedaIzqFinal;

	printf("\n");	
	printf("vel.lineal: %g m/s\n", cmdvel->linear.x);
	printf("vel.angular: %g rad/s\n", cmdvel->angular.z);
	printf("\n");
	printf("velRuedaDerFinal: %g m/s\n", velRuedaDerFinal);
	printf("velRuedaIzqFinal: %g m/s\n", velRuedaIzqFinal);
	printf("\n");
	printf("tiempoMediaCurvaSRuedaDer: %d ms\n", tiempoMediaCurvaSRuedaDer);
	printf("tiempoMediaCurvaSRuedaIzq: %d ms\n", tiempoMediaCurvaSRuedaIzq);

	 // Averiguar si la PMAC esta lista para obtener nuevas velocidades para las ruedas.
	 do {
	 if (leerDPRAM(descriptorDPRAM, DIR_FLAG_ACTUALIZAR_VELOCIDADES,
	 (char *) &flagActualizarVelocidades, NBYTES)) {
	 return;
	 }
	 } while (flagActualizarVelocidades != 0);

	 // La PMAC esta lista para aceptar nuevos valores para las velocidade de las ruedas.
	 if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_DER_FINAL,
	 (char *) &velRuedaDerFinal, NBYTES)) {
	 return;
	 }
	 if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_IZQ_FINAL,
	 (char *) &velRuedaIzqFinal, NBYTES)) {
	 return;
	 }
	 if (escribirDPRAM(descriptorDPRAM, DIR_TIEMPO_MEDIA_CURVAS_RUEDA_DER,
	 (char *) &tiempoMediaCurvaSRuedaDer, NBYTES)) {
	 return;
	 }

	 if (escribirDPRAM(descriptorDPRAM, DIR_TIEMPO_MEDIA_CURVAS_RUEDA_IZQ,
	 (char *) &tiempoMediaCurvaSRuedaIzq, NBYTES)) {
	 return;
	 }

	 flagActualizarVelocidades = 1;

	 if (escribirDPRAM(descriptorDPRAM, DIR_FLAG_ACTUALIZAR_VELOCIDADES,
	 (char *) &flagActualizarVelocidades, NBYTES) == -1) {
	 // La funcion finaliza sin exito.
	 return;

	 }


	// FIN
	/*
	 // ME QUEDO AQUI;
	 double i122 = (((double) 20) / M_PI) * fabs(velRuedaDer);
	 double i222 = (((double) 20) / M_PI) * fabs(velRuedaIzq);

	 char comandoPMAC[NUM_MAX_CARAC_BUFFER_PC_DPRAM_1];
	 char buffer_pmac_pc[NUM_MAX_CARAC_BUFFER_DPRAM_PC];

	 memset(comandoPMAC, '\0', NUM_MAX_CARAC_BUFFER_PC_DPRAM_1 * sizeof(char));
	 memset(buffer_pmac_pc, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC);

	 char dird = '+';
	 char diri = '+';

	 if (velRuedaDer < 0) {
	 dird = '-';
	 } else if (velRuedaDer == 0) {
	 dird = '/';
	 }

	 if (velRuedaIzq < 0) {
	 diri = '-';
	 } else if (velRuedaIzq == 0) {
	 diri = '/';
	 }

	 ROS_INFO("\ni122: %g - i222: %g\n", i122, i222);

	 int error = 0;
	 sprintf(comandoPMAC, "i122=%g i222=%g", i122, i222);
	 ROS_INFO("\nComando PMAC: %s\n", comandoPMAC);
	 comunicacionASCII(descriptorDPRAM, comandoPMAC, buffer_pmac_pc, &error);

	 memset(comandoPMAC, '\0', NUM_MAX_CARAC_BUFFER_PC_DPRAM_1 * sizeof(char));

	 sprintf(comandoPMAC, "#1j%c#2j%c", dird, diri);
	 ROS_INFO("\nComando PMAC: %s\n", comandoPMAC);
	 comunicacionASCII(descriptorDPRAM, comandoPMAC, buffer_pmac_pc, &error);
	 ROS_INFO("\n***********************\n");
	 */
}

int main(int argc, char **argv) {
	if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1) {
	 printf("No se puede abrir %s.\n", DPRAM);
	 return -1;
	 }
	 else{
		 printf("Comunicacion establecida con DPRAM.\n");
	 }
	ros::init(argc, argv, "nodoTeleopBaseTecladoSus");
	ros::NodeHandle n_;
	ros::Subscriber sub_ = n_.subscribe("topic_teleop_base_teclado_pub", 1,
			func_escuchadora);
	ros::spin();
	return 0;
}
