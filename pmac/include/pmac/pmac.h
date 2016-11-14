#ifndef PMAC

#define PMAC

#include <ctype.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/sem.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <netinet/in.h>
#include <errno.h>   /* Correspondencia de errores */
#include <termios.h> /* Definitiones POSIX terminal control */

// Poner el mismo fichero de dispositivo que se pone en el script dpram_load.sh
// y que se puede ver en /dev/<fichero_dispositivo>
#define DPRAM "/dev/dpram_driver"

// Numero de bytes que se pueden escribir en un registro de memoria XY de la DPRAM.
// El registro X tiene 16 bits (2 bytes) mas significativos.
// El registro Y tiene 16 bits (2 bytes) menos significativos.
#define NBYTES 4 

#define NUM_MOTORES 8		    // numero de motores del robot
#define NUM_MOTORES_BRAZO 6		// numero de motores del brazo
#define MOV_RUNNING 131072

// Caracteres de control.
#define CTRL_A  1
#define CTRL_B  2
#define CTRL_C  3
#define CTRL_D  4
#define CTRL_E  5
#define CTRL_F  6
#define CTRL_G  7
#define CTRL_H  8
#define CTRL_I  9

#define CTRL_K 11 
#define CTRL_L 12
#define CTRL_M 13
#define CTRL_N 14
#define CTRL_O 15
#define CTRL_P 16
#define CTRL_Q 17
#define CTRL_R 18
#define CTRL_S 19
#define CTRL_T 20
#define CTRL_U 21
#define CTRL_V 22
#define CTRL_W 23
#define CTRL_X 24
#define CTRL_Y 25
#define CTRL_Z 26

// 159 caracteres de datos mas el caracter final '\0'.
#define NUM_MAX_CARAC_BUFFER_PC_DPRAM_1 159
#define NUM_MAX_CARAC_BUFFER_PC_DPRAM_2  41
#define NUM_MAX_CARAC_BUFFER_PC_DPRAM   200
// 255 caracteres de datos mas el caracter final '\0'.
#define NUM_MAX_CARAC_BUFFER_DPRAM_PC   256

#define DIR_ESTADO_MOTOR1                  0x0000001B

#define DIR_CTRL_BACKGROUND_FIXED_DATA     0X0000008A

#define DIR_REGXY_D18B                     0x0000018B
#define DIR_BUFFER_PC_A_DPRAM              0x0000018C
#define DIR_REGXY_D1B4                     0x000001B4
#define DIR_BUFFER_DPRAM_A_PC              0x000001B5
#define DIR_REGXY_D009                     0X00000009

#define DIR_POSICION_ORDENADA_MOTOR1       0x00000012

#define DIR_POSICION_ACTUAL_MOTOR1         0x00000014
#define DIR_POSICION_ACTUAL_MOTOR2         0x00000023
#define DIR_POSICION_ACTUAL_MOTOR3         0x00000032
#define DIR_POSICION_ACTUAL_MOTOR4         0x00000041
#define DIR_POSICION_ACTUAL_MOTOR5         0x00000050
#define DIR_POSICION_ACTUAL_MOTOR6         0x0000005F
#define DIR_POSICION_ACTUAL_MOTOR7         0x0000006E
#define DIR_POSICION_ACTUAL_MOTOR8         0x0000007D

#define DIR_ESTADO_MOTOR1                  0x0000001B
#define DIR_ESTADO_MOTOR2                  0x0000002A
#define DIR_ESTADO_MOTOR3                  0x00000039
#define DIR_ESTADO_MOTOR4                  0x00000048
#define DIR_ESTADO_MOTOR5                  0x00000057
#define DIR_ESTADO_MOTOR6                  0x00000066
#define DIR_ESTADO_MOTOR7                  0x00000075
#define DIR_ESTADO_MOTOR8                  0x00000084

#define DIR_RESET_ODOMETRIA                0X00000200
#define DIR_VEL_RUEDA_DER_FINAL            0X00000201
#define DIR_VEL_RUEDA_IZQ_FINAL            0X00000202
#define DIR_FLAG_ACTUALIZAR_VELOCIDADES    0X00000203
#define DIR_FLAG_ODOMETRIA_ACTUALIZADA     0X00000204
#define DIR_MP9_TIEMPO_CURVAS_TRAMO1       0X00000205
#define DIR_MP9_TIEMPO_CURVAS_TRAMO2       0X00000206
#define DIR_MP9_DESPLAZ_CON_SIGNO_TRAMO1   0X00000207
#define DIR_MP9_DESPLAZ_CON_SIGNO_TRAMO2   0X00000208
#define DIR_MP9_VEL_FINAL_TRAMO1           0X00000209
#define DIR_TIEMPO_MEDIA_CURVAS_RUEDA_DER  0X0000020A
#define DIR_TIEMPO_MEDIA_CURVAS_RUEDA_IZQ  0X0000020B
#define DIR_X_ROBOT_ODOM		   0X0000020C
#define DIR_Y_ROBOT_ODOM                   0X0000020D
#define DIR_T_ROBOT_ODOM                   0X0000020E

#define DIR_FLAG_PVE_ACTUALIZADOS          0X0000035E

#define COM_INC_DEC1 "#1j:"
#define COM_INC_DEC2 "#2j:"
#define COM_INC_DEC3 "#3j=="
#define COM_INC_DEC4 "#4j=="
#define COM_INC_DEC5 "#5j=="
#define COM_INC_DEC6 "#6j=="
#define COM_INC_DEC7 "#7j=="
#define COM_INC_DEC8 "#8j=="

#define STOP_1 "#1J/"
#define STOP_2 "#2J/"
#define STOP_3 "#3J/"
#define STOP_4 "#4J/"
#define STOP_5 "#5J/"
#define STOP_6 "#6J/"
#define STOP_7 "#7J/"
#define STOP_8 "#8J/"

// Si cambia el valor de alguna variable IX08 modificar el define correspondiente.
// IX08 = 96, con X = 1, 2, 3, 4, 5, 6, 7, 8
// Ix08 * 32 = 3072
#define I108_POR_32 3072
#define I208_POR_32 3072
#define I308_POR_32 3072
#define I408_POR_32 3072
#define I508_POR_32 3072
#define I608_POR_32 3072
#define I708_POR_32 3072
#define I808_POR_32 3072

// Si cambia el valor de alguna variable IX08 modificar el define correspondiente.
// IX09 = 96, con X = 1, 2, 3, 4, 5, 6, 7, 8
// Ix09 * 32 = 3072
#define I109_POR_32 3072
#define I209_POR_32 3072
#define I309_POR_32 3072
#define I409_POR_32 3072
#define I509_POR_32 3072
#define I609_POR_32 3072
#define I709_POR_32 3072
#define I809_POR_32 3072

//DEFINICION DE FUNCIONES

int abrirComunicacionDPRAM(void);
int apagarOEncenderMotores(int descriptorFich, int apagarOEncender, char * datosParaPC, int * ptrError);
int cerrarComunicacionDPRAM(int fd_dpram);
int comunicacionASCII(int descriptorFich, char * datosParaPMAC, char * datosParaPC, int * error);
int escribirDPRAM(int descriptorFich, unsigned long direccionDPRAM, char * datosParaPMAC, int numBytesAEscribir);
int escribirRotacionBaseDPRAM(int descriptorFich, float * comandosPDer, float * comandoVDer, int * comandosT,
                              char * datosParaPC, int * ptrError);
int finalizarTurnoLecturaDatosFijosServo(int descriptorFich);
int learn(int fd, int number_buffer);
int leerDPRAM(int descriptorFich, unsigned long direccionDPRAM, char * datosParaPC, int numBytesALeer);
int leerEstadoMotor(int descriptorFich, int numeroMotor, int * bitVelocidadCeroMotor);
int leerInfoMotores(int descriptorFich, float * posicion, float * velocidad, float * error);
int leerPosicionOrdenadaOActual(int descriptorFich, int numeroMotor, double * cuentas);
// SRO: Sistema de referencia odometrico.
int obtenerOdometria(int descriptorDPRAM, float *x_mbase_modom, float *y_mbase_modom, float *t_mbase_modom);
long long int obtenerNumeroEnteroLargo(char * datosParaPC);
int pedirTurnoLecturaDatosFijosServo(int descriptorFich);
int resetearOdometriaPMAC(int descriptorFich);
int teleoperacion(int fd, float vdesp, float vang);

#endif
