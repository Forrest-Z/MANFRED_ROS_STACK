#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/resource.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "pmac/pmac.h"

// #include <ros/ros.h>

// Los registros de memoria X e Y de la PMAC, excepto aquellos
// registros que pertenecen a la DPRAM, tienen 24 bits, cada uno.
// Los registros de memoria X e Y de la DPRAM solo tienen 16 bits, cada uno. 

// El formato fixed-point    de la PMAC proporciona numeros enteros.
// El formato floating-point de la PMAC proporciona numeros reales.

// Un numero de 24 bits en formato fixed-point guardado en un registro de memoria X o Y
// de una direccion que no sea de la DPRAM se amplia a 32 bits (extension de 8 bits para signo del numero) cuando se copia
// en una direccion de la DPRAM, ocupan los 16 bits del registro X y los 16 bits del registro Y
// de la misma direccion. Este numero se puede guardar en una variable int (32 bits) de un programa C/C++ directamente.

// Un numero de 32 bits en formato fixed-point, usado especificamente para la DPRAM, se debe
// guardar en una variable int de un programa C/C++ directamente. Signo incluido en los 32 bits.

// Un numero de 48 bits en formato fixed-point guardado en un registro de memoria X e Y
// de una direccion que no sea de la DPRAM se amplia a 64 bits cuando se copia
// en una direccion de la DPRAM. Los 24 bits guardados en el registro de memoria Y de la direccion que no
// pertenece a la DPRAM se expanden a 32 bits cuando se copian en una direccion de la DPRAM.
// En la DPRAM el numero ocupa 16 bits de un registro de memoria X y 16 bits de un registro de memoria Y,
// de la misma direccion. Los 24 bits guardados en el registro de memoria X de la direccion que no
// pertenece a la DPRAM se expanden a 32 bits cuando se copian en una direccion de la DPRAM.
// En la DPRAM el numero ocupa 16 bits de un registro de memoria X y 16 bits de un registro de memoria Y,
// de la misma direccion. Este numero de 48 bits se guarda en una variable long long int (64 bits) de un programa C/C++.
// Hay que aniadir explicitamente el signo en la variable long long int (funcion obtenerNumeroEnteroLargo).

// Un numero de 32 bits en formato floating-point, usado especificamente para la DPRAM, se
// debe guardar en una variable float de un programa C/C++ directamente. Signo incluido en los 32 bits.

// Un numero de 48 bits en formato floating-point -----

// Actualmente se usa en los computadores el estandar IEEE 754 para 
// presentar numeros reales en coma flotante.

// El estandar codificacion de numeros reales IEEE 754 32 bits usa coma flotante y precision simple (variable float).
// 		signo (1 bit) | exponente (8 bits) | mantisa (23 bits).

// El estandar codificacion de numeros reales IEEE 754 64 bits usa coma flotante y precision doble (varible double).
// 		signo (1 bit) | exponente (11 bits) | mantisa (52 bits)

// Para entender estos formatos:
// 		pagina web 1: http://www.carlospes.com/curso_representacion_datos/06_01_estandar_ieee_754.php

/** Esta funcion permite abrir la comunicacion con la DPRAM de la PMAC.
 * @param void Esta funcion no acepta parametros.
 * @return Entero que representa el descriptor de fichero para comunicarse con la DPRAM.
 */
int abrirComunicacionDPRAM()
{
  return open(DPRAM, O_RDWR);
}

int apagarOEncenderMotores(int descriptorFich, int apagarOEncender, char * datosParaPC, int * ptrError)
{
  // Por defecto apagar motores.
  char comando = CTRL_K;
  // apagarOEncender: 0 -> apagar motores.
  // apagarOEncender: 1 -> encender motores.
  printf("Apagar motores por defecto.\n");
  if (apagarOEncender)
  {
    comando = CTRL_A;
    printf("Encender motores.\n");
  }
  char datosParaPMAC[2] = {comando, '\0'};
  if (comunicacionASCII(descriptorFich, datosParaPMAC, datosParaPC, ptrError))
  {
    return -1;
  }
  return 0;
}

/** Funcion que permite cerrar la comunicacion con la DPRAM de la PMAC.
 * @param int fd_dpram Entero que representa el descriptor de fichero para comunicarse con la DPRAM de la PMAC.
 * @return Entero que indica si el cierre de la comunicacion 'PC - DPRAM' se ha llevado a cabo con exito. Si el valor
 * devuelto es '0' la funcion ha retornado con exito. En caso de que la funcion finalice sin exito se retorna un numero
 * negativo.
 */
int cerrarComunicacionDPRAM(int fd_dpram)
{
  return close(fd_dpram);
}

// El comando debe acabar en caracter nulo '\0'.
int comunicacionASCII(int descriptorFich, char * datosParaPMAC, char * datosParaPC, int * error)
{

  int flag = 0;
  int numBytesAEscribir = 0;
  int numBytesALeer = 0;
  int tipoRespuestaPMAC = 0;
  char registroXY[NBYTES] = {'\0', '\0', '\0', '\0'};
  // Ojo: +1 debido al caracter '\0' con el que termina el comando
  // enviado a la PMAC.
  numBytesAEscribir = strlen(datosParaPMAC) + 1;
  // Se limita el numero de caracteres del comando que el PC puede escribir
  // en la DPRAM a 159 caracteres (158 caracteres + '\0' final).
  // PC -> DPRAM
  if (numBytesAEscribir > NUM_MAX_CARAC_BUFFER_PC_DPRAM_1)
  {
    return -1;
  }

  char comandoLimpiar[2] = {CTRL_X, '\0'};
  if(escribirDPRAM(descriptorFich, DIR_BUFFER_PC_A_DPRAM, comandoLimpiar, 2*sizeof(char))){
    return -1;
  }

  // El PC pone el bit0 del registro Y:$D18B a 1 cuando ha terminado de cargar
  // un comando a partir de la direccion DIR_BUFFER_PC_DPRAM en la DPRAM.
  // Mientras la PMAC procesa un comando escrito por el PC en la DPRAM
  // a partir de la direccion DIR_BUFFER_PC_DPRAM el valor del bit0 en el
  // registro Y:$D18B es 1 (la PMAC esta ocupada).
  // La PMAC pone el bit0 del registro Y:$D18B a 0 cuando ha terminado de procesar
  // un comando y esta lista para procesar otro.
  // El PC escribe en la DPRAM el comando que quiere enviar a la PMAC.
  if (escribirDPRAM(descriptorFich, DIR_BUFFER_PC_A_DPRAM, datosParaPMAC, numBytesAEscribir))
  {
    return -1;
  }
  // printf("El PC ha concluido de escribir datos en la DPRAM: ");
  // int i = 0;  
  // for(; i < numBytesAEscribir; ++i){	  
  //	  printf("%#.2X ", datosParaPMAC[i]);	  	  
  // }
  // printf("\n");
  // El PC pone el bit 0 del registro Y:$D18B a 1, indicando que ha terminado
  // de escribir el comando en la DPRAM.
  flag = 1;
  if (escribirDPRAM(descriptorFich, DIR_REGXY_D18B, (char *)&flag, NBYTES))
  {
    return -1;
  }
  // printf("El PC escribe un 1 en el bit 0 del registro Y:$D18B.\n");
  // LEER de la respuesta de la PMAC en la DPRAM.
  do
  {
    // Mientras el bit 0 del registro Y:$D1B4 esta a 0 la PMAC no ha terminado de responder al PC
    // y el PC esta a la espera de que la PMAC finalice su respuesta.
    do
    {
      // Lectura del registro Y:$D1B4.
      if (leerDPRAM(descriptorFich, DIR_REGXY_D1B4, registroXY, NBYTES))
      {
        return -1;
      }
      // Tipo de respuesta que proporciona la PMAC al PC.
      tipoRespuestaPMAC = registroXY[1];
      tipoRespuestaPMAC <<= 8;
      tipoRespuestaPMAC += registroXY[0];
      // printf("tipoRespuestaPMAC: %#.8X.\n", tipoRespuestaPMAC);
    } while (tipoRespuestaPMAC == 0);

    // Comprobar si hubo un error
    if ((tipoRespuestaPMAC & 0x00008000) == 0x00008000)
    {
      *error = (tipoRespuestaPMAC & 0x00000FFF);
      printf("ERROR: %#.8X\n", *error);
      return -1;
    }
    // Aqui no hay errores ya. Obtener el numero de caracteres de la respuesta.
    numBytesALeer = (registroXY[3] << 8) + registroXY[2];
    // Leer la respuesta de la PMAC.
    if (leerDPRAM(descriptorFich, DIR_BUFFER_DPRAM_A_PC, datosParaPC, numBytesALeer))
    {
      return -1;
    }
    // printf("El PC ha concluido de leer datos en la DPRAM: ");    
    // int i = 0;      
    // for(; i < numBytesALeer; ++i){	  
    //  printf("%#.2X ", datosParaPC[i]);
    // }      
    // printf("\n");

    // El PC pone a 0 el bit 0 del registro Y:$D1B4, indicando que ha procesado
    // la respuesta actual.
    flag = 0;
    if (escribirDPRAM(descriptorFich, DIR_REGXY_D1B4, (char *)&flag, NBYTES))
    {
      return -1;
    }
    // printf("El PC escribe un 0 en el bit 0 del registro Y:$D1B4.\n");
  } while (tipoRespuestaPMAC == 0x0000000D);

  // Funcion finaliza con exito.
  return 0;
}
//escribirDPRAM
int escribirDPRAM(int descriptorFich, unsigned long direccionDPRAM, char * datosParaPMAC, int numBytesAEscribir)
{

  // Esto siempre se cumple: numBytesAEscribir = numBytesNoEscritos + numBytesSiEscritos.
  int numBytesNoEscritos = numBytesAEscribir;
  int numBytesSiEscritos = 0;
  // printf("numBytesAEscribir: %d.\n", numBytesAEscribir);
  lseek(descriptorFich, direccionDPRAM, SEEK_SET);
  do
  {
// write en este caso (debido a la forma de hacer el driver de la PMAC) retona el numero de
// bytes que no se han escrito.
    if ((numBytesNoEscritos = write(descriptorFich, (datosParaPMAC + numBytesSiEscritos), numBytesNoEscritos)) < 0)
    {
      return -1;
    }
    numBytesSiEscritos = numBytesAEscribir - numBytesNoEscritos;
    //printf("numBytesAEscribir: %d     numBytesSiEscritos: %d\n", numBytesAEscribir, numBytesSiEscritos);
  } while (numBytesSiEscritos != numBytesAEscribir);
  return 0;
}

int escribirRotacionBaseDPRAM(int descriptorFich, float * comandosPDer, float * comandoVDer, int * comandosT,
                              char * datosParaPC, int * ptrError)
{

  char datosParaPMAC[6] = {'&', '1', 'B', '8', 'R', '\0'};
  int tiempoMediaCurvaSTramo1 = 0;
  int tiempoMediaCurvaSTramo2 = 0;
  float desplazConSignoTramo1 = 0;
  float desplazConSignoTramo2 = 0;
  float velFinalTramo1 = 0;

  //char * comando = (char *)calloc(NUM_MAX_CARAC_BUFFER_PC_DPRAM_1, sizeof(char));
  //printf("comandosPDer: %f %f.\n", comandosPDer[0], comandosPDer[1]);
  //printf("comandosPDer: %f.\n", comandoVDer[0]);
  //printf("comandosT: %d %d.\n", comandosT[0], comandosT[1]);
  //sprintf(comando, "M82=%d M83=%d M84=%f M85=%f M86=%f", comandosT[0], comandosT[1], comandosPDer[0], comandosPDer[1], *comandoVDer);
  //printf(comando);
  //printf("\n");
  if (apagarOEncenderMotores(descriptorFich, 1, datosParaPC, ptrError))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorFich, DIR_MP8_TIEMPO_MEDIA_CURVAS_TRAMO1, (char *)comandosT, NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorFich, DIR_MP8_TIEMPO_MEDIA_CURVAS_TRAMO2, (char *)(comandosT + 1), NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorFich, DIR_MP8_DESPLAZ_CON_SIGNO_TRAMO1, (char *)comandosPDer, NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorFich, DIR_MP8_DESPLAZ_CON_SIGNO_TRAMO2, (char *)(comandosPDer + 1), NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorFich, DIR_MP8_VEL_FINAL_TRAMO1, (char *)comandoVDer, NBYTES))
  {
    return -1;
  }
  //if (comunicacionASCII(descriptorFich, comando, datosParaPC, ptrError))
  //{
    //return -1;
  //}
  if (leerDPRAM(descriptorFich, DIR_MP8_TIEMPO_MEDIA_CURVAS_TRAMO1, (char *)&tiempoMediaCurvaSTramo1, NBYTES))
  {
    return -1;
  }
  printf("tiempoMediaCurvaSTramo1: %d.\n", tiempoMediaCurvaSTramo1);
  if (leerDPRAM(descriptorFich, DIR_MP8_TIEMPO_MEDIA_CURVAS_TRAMO2, (char *)&tiempoMediaCurvaSTramo2, NBYTES))
  {
    return -1;
  }
  printf("tiempoMediaCurvaSTramo2: %d.\n", tiempoMediaCurvaSTramo2);
  if (leerDPRAM(descriptorFich, DIR_MP8_DESPLAZ_CON_SIGNO_TRAMO1, (char *)&desplazConSignoTramo1, NBYTES))
  {
    return -1;
  }
  printf("desplazConSignoTramo1: %f.\n", desplazConSignoTramo1);
  if (leerDPRAM(descriptorFich, DIR_MP8_DESPLAZ_CON_SIGNO_TRAMO2, (char *)&desplazConSignoTramo2, NBYTES))
  {
    return -1;
  }
  printf("desplazConSignoTramo2: %f.\n", desplazConSignoTramo2);
  if (leerDPRAM(descriptorFich, DIR_MP8_VEL_FINAL_TRAMO1, (char *)&velFinalTramo1, NBYTES))
  {
    return -1;
  }
  printf("velFinalTramo1: %f.\n", velFinalTramo1);
  if (comunicacionASCII(descriptorFich, datosParaPMAC, datosParaPC, ptrError))
  {
    return -1;
  }
  return 0;
}

int leerDPRAM(int descriptorFich, unsigned long direccionDPRAM, char * datosParaPC, int numBytesALeer)
{

  // Esto siempre se cumple: numBytesALeer = numBytesNoLeidos + numBytesSiLeidos.
  int numBytesNoLeidos = numBytesALeer;
  int numBytesSiLeidos = 0;
  // printf("numBytesALeer: %d.\n", numBytesALeer);
  lseek(descriptorFich, direccionDPRAM, SEEK_SET);
  do
  {
// read en este caso (debido a la forma de hacer el driver de la PMAC) retona el numero de
// bytes que no se han leido.
    if ((numBytesNoLeidos = read(descriptorFich, (datosParaPC + numBytesSiLeidos), numBytesNoLeidos)) < 0)
    {
      return -1;
    }
    numBytesSiLeidos = numBytesALeer - numBytesNoLeidos;
  } while (numBytesSiLeidos != numBytesALeer);
  return 0;
}

/*
 int leerDatosFijosSegundoPlano(int fd_dpram, unsigned long dir_dpram, long long int* datos, int *marca_temporal,
 int tamanio_4_8)
 {
 int estado_pmac = 0;
 int flag_estado = 0;
 int tamanio_leido = 0;
 // Cada palabra es de 32 bits.
 int palabra_alta = 0;
 int palabra_baja = 0;

 // Esperar mientras la PMAC no ha terminado de actualizar los datos de interes.
 // Cuando la PMAC haya terminado de actualizar los datos de interes en la DPRAM lo
 // indicara poniendo el bit 0 del registro Y:$D08A a 1.
 lseek(fd_dpram, DIR_CTRL_BACKGROUND_FIXED_DATA, SEEK_SET);
 do
 {
 tamanio_leido = read(fd_dpram, &estado_pmac, NBYTES);
 flag_estado = 0x00000001 & estado_pmac;
 } while (flag_estado == 0x00000000);

 // El pc ya puede leer.
 lseek(fd_dpram, dir_dpram, SEEK_SET);
 // Los datos de interes pueden tener un tamanio de 4 u 8 bytes.
 tamanio_leido = read(fd_dpram, (void *)&palabra_baja, NBYTES);
 // printf("pb: %#8X\n", palabra_baja);
 palabra_baja = 0x00FFFFFF & palabra_baja;
 //printf("pb 2: %#8X\n", palabra_baja);
 // Comprobar si el dato de interes tiene un tamanio de 8 bytes.
 if (tamanio_4_8)
 {
 lseek(fd_dpram, dir_dpram + 1, SEEK_SET);
 tamanio_leido = read(fd_dpram, (void *)&palabra_alta, NBYTES);
 // printf("pa: %#8X\n", palabra_alta);
 }
 *datos = (palabra_alta << 24) + palabra_baja;
 // Obtener la marca temporal asociada a los datos.
 *marca_temporal = (estado_pmac >> 16);
 // El pc indica que ha terminado de leer.
 lseek(fd_dpram, DIR_CTRL_BACKGROUND_FIXED_DATA, SEEK_SET);
 flag_estado = 0;
 // Los datos de servo tienen una tama�o de 64 bits como mucho. (8 bytes).
 tamanio_leido = write(fd_dpram, (void *)&flag_estado, 1);
 // La funcion finaliza con exito.
 return 0;
 }
 */

// posicionOrdenadaOActual: 0 para posicion ordenada
// posicionOrdenadaOActual: 1 para posicion actual
int leerPosicionOrdenadaOActual(int descriptorFich, int numeroMotor, int posicionOrdenadaOActual, double * cuentas)
{
  unsigned long direccionDPRAM = 0;
  char datosParaPC[8] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
  double denominador = 0;
  if (numeroMotor < 0 || numeroMotor > 8 || posicionOrdenadaOActual < 0 || posicionOrdenadaOActual > 1)
  {
    printf("NUMERO DE MOTOR ERRONEO U ORDEN ERRONEA");
    return -1;
  }
  // Actualmente todas las variables IX08, con X = 1, 2, ...., 8, valen 96. Asi que 96 * 32 = 3072
  // El numero decimal 15 en demical es F
  direccionDPRAM = DIR_POSICION_ORDENADA_MOTOR1 + 2 * posicionOrdenadaOActual + 15 * (numeroMotor - 1);
  denominador = I108_POR_32;
  /*
   switch (numeroMotor)
   {
   // Motor #1 (Rueda derecha, la que esta en el lado del brazo)
   case 1:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR1;
   denominador = I108_POR_32;
   break;
   // Motor #2 (Base)
   case 2:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR2;
   denominador = I208_POR_32;
   break;
   // Motor #3 (Brazo-Munieca)
   case 3:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR3;
   denominador = I308_POR_32;
   break;
   // Motor #4 (Brazo-Hombro)
   case 4:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR4;
   denominador = I408_POR_32;
   break;
   // Motor #5 (Brazo-Munieca)
   case 5:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR5;
   denominador = I508_POR_32;
   break;
   // Motor #6 (Brazo-Hombro)
   case 6:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR6;
   denominador = I608_POR_32;
   break;
   // Motor #7 (Brazo-Munieca)
   case 7:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR7;
   denominador = I708_POR_32;
   break;
   // Motor #8 (Brazo-Codo)
   case 8:
   direccionDPRAM = DIR_POSICION_ACTUAL_MOTOR8;
   denominador = I808_POR_32;
   break;
   default:
   printf("ERROR: NUMERO DE MOTOR ERRONEO.\n");
   return -1;
   }
   */
  if (pedirTurnoLecturaDatosFijosServo(descriptorFich))
  {
    return -1;
  }
  // EL PC lee la informacion de interes.
  if (leerDPRAM(descriptorFich, direccionDPRAM, datosParaPC, 2 * NBYTES))
  {
    return -1;
  }
  if (finalizarTurnoLecturaDatosFijosServo(descriptorFich))
  {
    return -1;
  }
  *cuentas = ((double)obtenerNumeroEnteroLargo(datosParaPC)) / denominador;
  return 0;
}

// Lee el bit de VELOCIDAD ZERO de los motores del brazo
int leerEstadoMotor(int descriptorFich, int numeroMotor, int * bitVelocidadCeroMotor)
{
  unsigned long direccionDPRAM = 0;
  char datosParaPC[8] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
  if (numeroMotor < 0 || numeroMotor > 8)
  {
    printf("NUMERO DE MOTOR ERRONEO U ORDEN ERRONEA");
    return -1;
  }
  // Actualmente todas las variables IX08, con X = 1, 2, ...., 8, valen 96. Asi que 96 * 32 = 3072
  // El numero decimal 15 en demical es F
  direccionDPRAM = DIR_ESTADO_MOTOR1 + 15 * (numeroMotor - 1);

  /*
   switch (numeroMotor)
   {
   // Motor #1 (Rueda derecha, la que esta en el lado del brazo)
   case 1:
   direccionDPRAM = DIR_ESTADO_MOTOR1;
   break;
   // Motor #2 (Base)
   case 2:
   direccionDPRAM = DIR_ESTADO_MOTOR2;
   break;
   // Motor #3 (Brazo-Munieca)
   case 3:
   direccionDPRAM = DIR_ESTADO_MOTOR3;
   break;
   // Motor #4 (Brazo-Hombro)
   case 4:
   direccionDPRAM = DIR_ESTADO_MOTOR4;
   break;
   // Motor #5 (Brazo-Munieca)
   case 5:
   direccionDPRAM = DIR_ESTADO_MOTOR5;
   break;
   // Motor #6 (Brazo-Hombro)
   case 6:
   direccionDPRAM = DIR_ESTADO_MOTOR6;
   break;
   // Motor #7 (Brazo-Munieca)
   case 7:
   direccionDPRAM = DIR_ESTADO_MOTOR7;
   break;
   // Motor #8 (Brazo-Codo)
   case 8:
   direccionDPRAM = DIR_ESTADO_MOTOR8;
   break;
   default:
   printf("ERROR: Numero de motor invalido\n");
   return -1;
   }
   */

  if (pedirTurnoLecturaDatosFijosServo(descriptorFich))
  {
    return -1;
  }
  // EL PC lee la informacion de interes.
  if (leerDPRAM(descriptorFich, direccionDPRAM, datosParaPC, NBYTES))
  {
    return -1;
  }
  if (finalizarTurnoLecturaDatosFijosServo(descriptorFich))
  {
    return -1;
  }
  int i = NBYTES - 1;
  printf("Estado del motor.\n");
  for (; i >= 0; --i)
  {
    printf("%#.2X ", datosParaPC[i]);
  }
  printf("\n");
  // Bit de velocidad cero: bit 13 -> 2 byte, bit 6
  *bitVelocidadCeroMotor = (datosParaPC[1] & 0x20) >> 5;
  printf("Motor: %d     bitVelocidadCeroMotor: %d.\n", numeroMotor, *bitVelocidadCeroMotor);
// Funcion finaliza con exito.
  return 0;
}

// Lee la posicion, la velocidad actual y el error de seguimiento
int leerInfoMotores(int descriptorFich, float * posicion, float * velocidad, float * error)
{

  // Dir para posicion del motor 1
  unsigned long direccionDPRAM = 0x00000346;
  // PVE: posicion, velocidad, error.
  int flagPVEActualizados = 0;
  unsigned int i = 0;
  char pos[NBYTES] = {'\0', '\0', '\0', '\0'};
  char vel[NBYTES] = {'\0', '\0', '\0', '\0'};
  char err[NBYTES] = {'\0', '\0', '\0', '\0'};
// Esperar a que la PMAC actualice en la DPRAM la posicion, velocidad y error de todos los motores.
// Cuando la PMAC haya finalizado la actualizacion de los datos en la DPRAM escribe en esta a partir de la direccion
// 'DIR_FLAG_PVE_ACTUALIZADOS' el valor 1, usando 4 bytes.

// Espera activa
  do
  {
    if (leerDPRAM(descriptorFich, DIR_FLAG_PVE_ACTUALIZADOS, (char *)&flagPVEActualizados, NBYTES) == -1)
    {
      return -1;
    }
  } while (flagPVEActualizados == 0x00000000);
  // La pmac ha terminado de actualizar la informacion que es interesante.
  for (; i < NUM_MOTORES; i++)
  {
    // Posicion
    if (leerDPRAM(descriptorFich, direccionDPRAM, pos, NBYTES))
    {
      return -1;
    }
    memcpy(posicion + i, pos, NBYTES);
    // Velocidad
    direccionDPRAM += 1;
    if (leerDPRAM(descriptorFich, direccionDPRAM, vel, NBYTES))
    {
      return -1;
    }
    memcpy(velocidad + i, vel, NBYTES);
    // Error
    direccionDPRAM += 1;
    if (leerDPRAM(descriptorFich, direccionDPRAM, err, NBYTES))
    {
      return -1;
    }
    memcpy(error + i, err, NBYTES);
    direccionDPRAM += 1;
  }
  // El PC indica que ha terminado de leer de la DPRAM limpiando el flag 'flagPVEActualizados' (valor 0)
  flagPVEActualizados = 0;
  if (escribirDPRAM(descriptorFich, DIR_FLAG_PVE_ACTUALIZADOS, (char *)&flagPVEActualizados, NBYTES) == -1)
  {
    // La funcion finaliza sin exito.
    return -1;
  }
// La funcion finaliza con exito.
  return 0;
}

/* Lee la posicion, la velocidad actual y el error de seguimiento
 int leerInfoMotores(int fd_dpram, float posicion[NUM_MOTORES], float velocidad[NUM_MOTORES], float error[NUM_MOTORES])
 {
 unsigned int estado_pmac = 0;
 int tamanio_leido = 0;
 char flag_pc = 1;
 unsigned int i = 0;
 unsigned long dir_dpram = 0x00000346;     //dir para posicion del motor 3
 // Cada palabra es de 32 bits.
 float palabra_baja;

 // Esperar mientras la PMAC esta ocupada actualizando los segmentos de memoria de
 // la DPRAM que contienen los datos de servo
 lseek(fd_dpram, DIR_REGXY_D009, SEEK_SET);
 do
 {
 tamanio_leido = read(fd_dpram, &estado_pmac, NBYTES);
 estado_pmac = 0x80000000 & estado_pmac;
 } while (estado_pmac == 0x80000000);
 // Poner a 1 el flag de 'pc-leyendo'. Se escribe un solo byte.
 tamanio_leido = write(fd_dpram, (void *)&flag_pc, 1);
 // Mirar de nuevo el estado de la PMAC  (actualizando segmentos de memoria o esperando
 // a que el pc termine de leer. Este bloque de codigo es una precaucion que nos aconsejan
 // hacer en el manual de la DPRAM, pag.14.
 do
 {
 tamanio_leido = read(fd_dpram, &estado_pmac, NBYTES);
 estado_pmac = 0x80000000 & estado_pmac;
 } while (estado_pmac == 0x80000000);

 for (i = 0; i < NUM_MOTORES; i++)
 {
 // El pc ya puede leer.
 lseek(fd_dpram, dir_dpram, SEEK_SET);
 tamanio_leido = read(fd_dpram, (void *)&palabra_baja, NBYTES);
 posicion[i] = palabra_baja;
 //printf("pb: %#8X\n", palabra_baja);
 //palabra_baja = 0x00FFFFFF & palabra_baja;
 //printf("lectura: %f\n", palabra_baja);
 //printf("dir: %X\n", dir_dpram);

 dir_dpram = dir_dpram + 1;
 lseek(fd_dpram, dir_dpram, SEEK_SET);
 // Los datos de interes pueden tener un tamanio de 4 u 8 bytes.
 tamanio_leido = read(fd_dpram, (void *)&palabra_baja, NBYTES);
 velocidad[i] = palabra_baja;

 dir_dpram = dir_dpram + 1;
 lseek(fd_dpram, dir_dpram, SEEK_SET);
 // Los datos de interes pueden tener un tamanio de 4 u 8 bytes.
 tamanio_leido = read(fd_dpram, (void *)&palabra_baja, NBYTES);
 error[i] = palabra_baja;

 dir_dpram = dir_dpram + 1;
 }
 // El pc indica que ha terminado de leer.
 lseek(fd_dpram, DIR_REGXY_D009, SEEK_SET);
 flag_pc = 0;
 // Los datos de servo tienen una tama�o de 64 bits como mucho. (8 bytes).
 tamanio_leido = write(fd_dpram, (void *)&flag_pc, 1);
 // La funcion finaliza con exito.

 return 0;
 }
 */

// SRO: Sistema de referencia odometrico.
// Leer la ubicacion del robot en el sistema de referencia odometrico de las variables M
// correspondientes en la DPRAM.
int leerUbicacionRobotSRO(int descriptorFich, float *xRobotSRO, float *yRobotSRO, float *tRobotSRO)
{

  int flagOdometriaActualizada = 1;
// Esperar a que la PMAC actualice en la DPRAM la ubicacion del robot
// en el SRO.
// Cuando la PMAC haya finalizado la actualizacion de la ubicacion
// del robot en la DPRAM escribe en esta a partir de la direccion
// 'DIR_FLAG_ACTUALIZAR_ODOMETRIA' el valor 1, usando 4 bytes.

  // Comprobar si la pmac esta ocupada
  do
  {
    if (leerDPRAM(descriptorFich, DIR_FLAG_ODOMETRIA_ACTUALIZADA, (char *)&flagOdometriaActualizada, NBYTES) == -1)
    {
      return -1;
    }
  } while (flagOdometriaActualizada == 0x00000000);
  // La pmac queda libre y el pc indica que esta ocupado.
  //if (escribirDPRAM(descriptorFich, DIR_FLAG_PC_OCUPADO_ODOM, (char *)&flagPcOcupadoOdom, NBYTES))
  //{
  //return -1;
  //}
// El pc ya puede leer de la DPRAM la ubicacion del robot en el SRO.
  if (leerDPRAM(descriptorFich, DIR_X_ROBOT_ODOM, (char *)xRobotSRO, NBYTES) == -1)
  {
    return -1;
  }
  if (leerDPRAM(descriptorFich, DIR_Y_ROBOT_ODOM, (char *)yRobotSRO, NBYTES) == -1)
  {
    return -1;
  }
  if (leerDPRAM(descriptorFich, DIR_T_ROBOT_ODOM, (char *)tRobotSRO, NBYTES) == -1)
  {
    return -1;
  }
// El PC indica que ha terminado de leer de la DPRAM la ubicacion del
// robot en el SRO.
  flagOdometriaActualizada = 0;
  if (escribirDPRAM(descriptorFich, DIR_FLAG_ODOMETRIA_ACTUALIZADA, (char *)&flagOdometriaActualizada, NBYTES) == -1)
  {
    // La funcion finaliza sin exito.
    return -1;
  }
  *xRobotSRO =  *xRobotSRO * 0.01; // En m.
  *yRobotSRO =  *yRobotSRO * 0.01; // En m. ddddddddddddddddddddddddddddd
  printf("pmac X: %.3f m.\n", *xRobotSRO);
  printf("pmac Y: %.3f m.\n", *yRobotSRO);
  printf("pmac THETA: %.3f rad.\n", *tRobotSRO);
// La funcion finaliza con exito.
  return 0;
}

/**

 La pinza tiene una libreria: la libreria del faulhaber (como la peana donde se apoya el laser).

 int control_pinza(int comando){
 // Descriptor fichero para el puerto
 int df=-1;
 // Estructuras definidas en termios.h
 // En "guardado" guardaremos la configuraciNsn actual de los puertos para una vez terminado el programa
 // restaurar los valores iniciales antes de su ejecuciNsn. Config lo usaremos para variar los parametros del puerto
 struct termios config;
 struct termios guardado;
 int n_bytes_mandados=0;
 //char *abrir="V-10000\r";
 //char *cerrar="V10000\r";
 //char *home="HO\r";
 //df = open(PUERTO1, O_RDWR | O_NOCTTY | O_NDELAY);
 if (df == -1){
 printf("\nIncapaz de abrir puerto\n");
 return (-1);
 }
 tcgetattr(df, &config);
 tcgetattr(df, &guardado);
 configura_puerto(df,&config);
 switch(comando){
 case 1:
 n_bytes_mandados=write(df,"V-10000\r",sizeof("V-10000\r"));
 break;
 case 2:
 n_bytes_mandados=write(df,"V10000\r",sizeof("V10000\r"));
 break;
 case 0:
 n_bytes_mandados=write(df,"ho",sizeof("ho"));
 break;
 }
 tcsetattr(df, TCSAFLUSH, &guardado);
 close(df);
 if (n_bytes_mandados < 0 || n_bytes_mandados==0){
 return(-1);
 return (0);
 }
 }
 */

//FUNCION CLEAR BUFFER
/*
 int clear(int fd_dpram, char *number_buffer){

 int error = 0;

 char *ABS = "ABS";
 char *LINEAR = "LINEAR";
 char *TS0 = "TS0";

 char *aceleracion = "TA3000";
 char apertura_buffer[159] = "open prog ";
 char *clear = "clear";
 char *close = "close";
 char lectura[159];

 strcat(apertura_buffer,number_buffer);
 comunicacion_dpram(fd_dpram, apertura_buffer, lectura, &error);
 comunicacion_dpram(fd_dpram, clear, lectura, &error);
 comunicacion_dpram(fd_dpram, LINEAR, lectura, &error);
 comunicacion_dpram(fd_dpram, ABS, lectura, &error);
 comunicacion_dpram(fd_dpram, aceleracion, lectura, &error);
 comunicacion_dpram(fd_dpram, TS0, lectura, &error);
 comunicacion_dpram(fd_dpram, close, lectura, &error);

 return(0);
 }
 */

int finalizarTurnoLecturaDatosFijosServo(int descriptorFich)
{

// El pc indica que ha terminado de leer poniendo el bit 0 del registro Y:$D009 a 0.
  int flag = 0;
  // la respuesta de la PMAC en la DPRAM.
  if (escribirDPRAM(descriptorFich, DIR_REGXY_D009, (char *)&flag, NBYTES))
  {
    return -1;
  }
  return 0;
}

int learn(int fd_dpram, int num_buffer)
{

  char buffer_pmac_pc[NUM_MAX_CARAC_BUFFER_DPRAM_PC];
  char comando[NUM_MAX_CARAC_BUFFER_PC_DPRAM_1];
  char apertura_buffer[] = {'o', 'p', 'e', 'n', ' ', 'p', 'r', 'o', 'g', '\0'};
  char learn[] = {'l', 'e', 'a', 'r', 'n', '\0'};
  char close[] = {'c', 'l', 'o', 's', 'e', '\0'};
  int error = 0;

  memset(comando, '\0', NUM_MAX_CARAC_BUFFER_PC_DPRAM_1 * sizeof(char));
  sprintf(comando, "open prog %d", num_buffer);
  comunicacionASCII(fd_dpram, apertura_buffer, buffer_pmac_pc, &error);
  comunicacionASCII(fd_dpram, learn, buffer_pmac_pc, &error);
  comunicacionASCII(fd_dpram, close, buffer_pmac_pc, &error);

  return 0;
}

long long int obtenerNumeroEnteroLargo(char * datosParaPC)
{

  long long int numeroEnteroLargo = 0x0000000000000000;
  // Copiar los 4 bytes mas significativos de datosParaPC en numeroEnteroLargo
  memcpy(&numeroEnteroLargo, datosParaPC + 4, 4);
  numeroEnteroLargo <<= 24;
  memcpy(&numeroEnteroLargo, datosParaPC, 3);
  //numeroEnteroLargo |= (palabraBaja & 0x00FFFFFF);

  // palabraAlta con signo negativo
  if ((datosParaPC[7] & 0x80) == 0x80)
  {

    numeroEnteroLargo |= 0xFF00000000000000;

  }

  return numeroEnteroLargo;

}

int pedirTurnoLecturaDatosFijosServo(int descriptorFich)
{

  char registroXY[4] = {'\0', '\0', '\0', '\0'};
  int flag = 1;
  // Esperar a que la PMAC finalice de escribir la respuesta para el PC en la DPRAM.
  // Cuando la PMAC esta ocupada escribiendo en la DPRAM el bit 15 del registro
  // X:$D009 esta a 1. Cuando la PMAC acaba la escritura en la DPRAM el bit 15 del
  // registro anterior se pone a 0.
  do
  {
    if (leerDPRAM(descriptorFich, DIR_REGXY_D009, registroXY, NBYTES))
    {
      return -1;
    }
  } while ((registroXY[3] & 0x80) == 0x80);
  // El PC pone el bit 0 del registro Y:$D009 a 1 indicando que va a leer
  // la respuesta de la PMAC en la DPRAM.
  if (escribirDPRAM(descriptorFich, DIR_REGXY_D009, (char *)&flag, NBYTES))
  {
    return -1;
  }
  // Comprobar de nuevo el estado de la PMAC. Este bloque es util para
  // evitar un bloque mutuo, es decir, que el PC espere a que termine la PMAC, y
  // que la PMAC espere a que termine el PC, en cuyo caso el programa no termina
  // nunca. Este bloque de codigo es una precaucion que aconsejan
  // hacer en el manual de la DPRAM, pag.14. Nos aconsejan leer el registro X:$D009
  // solo unas cuantas veces o pasar directamente. Aqui se lee hasta que la PMAC
  // finalice su tarea.
  do
  {
    if (leerDPRAM(descriptorFich, DIR_REGXY_D009, registroXY, NBYTES))
    {
      return -1;
    }
  } while ((registroXY[3] & 0x80) == 0x80);
  return 0;

}

int teleoperacion(int fd, float vdesp, float vang)
{

  char g_buffer_pmac_pc[NUM_MAX_CARAC_BUFFER_DPRAM_PC];
  char g_buffer_pc_pmac[NUM_MAX_CARAC_BUFFER_PC_DPRAM_1];
// int cuadrante=0;
  float W1 = 0;
  float W2 = 0;
  float V = 0;
  float W = 0;
  char aux1[20];
  char aux2[20];
  char i122[20] = "i122=";
  char i222[20] = "i222=";
  int ret = 0;
  int error = 0;
  char lectura[20];
  char vm1_vm2_orig[] = {'i', '1', '2', '2', '=', '1', ' ', 'i', '2', '2', '2', '=', '1', '\0'};
  char parar[] = {'#', '1', 'j', '/', '#', '2', 'j', '/', '\0'};
  char arrancar[] = {'#', '1', 'j', '+', '#', '2', 'j', '+', '\0'};

// Limipeza de los buffer.
  memset(g_buffer_pc_pmac, '\0', NUM_MAX_CARAC_BUFFER_PC_DPRAM_1);
  memset(g_buffer_pmac_pc, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC);

// Compruebo que la teleoperacion ha parado
  if (vdesp == 0 && vang == 0)
  {
    // Parar el robot
    comunicacionASCII(fd, parar, g_buffer_pmac_pc, &error);
    // Restaurar valores iniciales.
    comunicacionASCII(fd, vm1_vm2_orig, g_buffer_pmac_pc, &error);
    return 1;
  }

  V = vdesp * 0.15;
  W = vang * 0.15;

//Obtengo con el modelo w1 y w2 en rad/seg
  W1 = (V + (0.26125 * W)) / (0.0775);
  W2 = (V - (0.26125 * W)) / (0.0775);
//Paso rad/seg a cuentas/msg
  W1 = W1 * (21.4425778 / 3.5);
  W2 = W2 * (21.4425778 / 3.5);
//Escribo en la PMAC el comando con las velocidades deseadas
  sprintf(aux1, "%f", W1);
  strncpy(&i122[5], aux1, 5);
//printf("Lo que escribe en el terminal es: %s \n", i122);
  sprintf(aux2, "%f", W2);
  strncpy(&i222[5], aux2, 5);
//  printf("Lo que escribe en el terminal es: %s \n", i222);
  comunicacionASCII(fd, i122, lectura, &error);
  comunicacionASCII(fd, i222, lectura, &error);
  comunicacionASCII(fd, arrancar, lectura, &error);

  return (ret);
}
