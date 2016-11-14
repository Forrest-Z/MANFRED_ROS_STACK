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

//#include <ros/ros.h>

/** Esta funcion permite abrir la comunicacion con la DPRAM de la PMAC.
 * @param void Esta funcion no acepta parametros.
 * @return Entero que representa el descriptor de fichero para comunicarse con la DPRAM.
 */
int abrir_dpram()
{
  return open(DPRAM, O_RDWR);
}

/** Funcion que permite cerrar la comunicacion con la DPRAM de la PMAC.
 * @param int fd_dpram Entero que representa el descriptor de fichero para comunicarse con la DPRAM de la PMAC.
 * @return Entero que indica si el cierre de la comunicacion 'PC - DPRAM' se ha llevado a cabo con exito. Si el valor
 * devuelto es '0' la funcion ha retornado con exito. En caso de que la funcion finalice sin exito se retorna un numero
 * negativo.
 */
int cerrar_dpram(int fd_dpram)
{
  return close(fd_dpram);
}

void comunicacion_dpram(int fd_dpram, void *comando, char *buffer_pmac_pc, int *error)
{
  int flag = 0;
  int tamanio = 0;
  // Limpiar registros $D18B y $D1B4.
  flag = 0;
  escribir_ascii_dpram(fd_dpram, DIR_REG_YX_D18B, &flag, NBYTES);
  escribir_ascii_dpram(fd_dpram, DIR_REG_YX_D1B4, &flag, NBYTES);
  // Enviar instrucciones a la PMAC a traves de la DPRAM.
  // Ojo: un caracter '\0' al final de la cadena enviada a la PMAC.
  tamanio = strlen((char *)comando) + 1;
  escribir_ascii_dpram(fd_dpram, DIR_BUFF_PC_PMAC, comando, tamanio);
  // Establecer a 1 el valor del flag de control de salida del equipo, indicando que
  // la transferencia del equipo se ha completado.
  flag = 1;
  // lseek(fd, DIR_REG_YX_D18B, SEEK_SET);
  // write(fd, &flag_send, NBYTES);
  escribir_ascii_dpram(fd_dpram, DIR_REG_YX_D18B, &flag, NBYTES);
  // Lectura de la respuesta de la PMAC en la DPRAM.
  // LECTURA;
  leer_ascii_dpram(fd_dpram, buffer_pmac_pc, error);
}

int escribir_ascii_dpram(int fd_dpram, unsigned long direccion, void *datos, int tamanio)
{
  int tamanioLeido = 0;
  lseek(fd_dpram, direccion, SEEK_SET);
  tamanioLeido = write(fd_dpram, datos, tamanio);
  printf("tamanioEscrito:%d\n", tamanioLeido);
  return tamanioLeido;
}

void leer_ascii_dpram(int fd_dpram, char *buffer_pmac_pc, int *error)
{
  int flag = 0;
  int num_caracteres = 0;
  int num_posiciones = 0;
  int lectura_terminada = 0;
  int tipoRespuesta = 0;
  int num_bytes_leidos = 0;
  int j = 0;

  char registro_YX_D1B4[NBYTES];

  do
  {
    memset(registro_YX_D1B4, '\0', NBYTES);
    lseek(fd_dpram, DIR_REG_YX_D1B4, SEEK_SET);
    num_bytes_leidos = read(fd_dpram, &registro_YX_D1B4, NBYTES);
    // Hay respuesta?
    if (registro_YX_D1B4[0] > 0)
    {
      // �Existio un error?
      if ((int)registro_YX_D1B4[1] == -128)
      {
        // Averiguar el tipo de error.
        *error = (int)registro_YX_D1B4[0];
        lectura_terminada = 1;
      }
      else
      {
        memcpy(&tipoRespuesta, registro_YX_D1B4, 2);
        // printf("Tipo respuesta: %d.\n", tipoRespuesta);
        if (tipoRespuesta == 6 || tipoRespuesta == 269 || tipoRespuesta == 525)
        {
          lectura_terminada = 1;
        }
        else
        {
          num_caracteres = (int)registro_YX_D1B4[2];
          num_posiciones = (num_caracteres / NBYTES) + 1;
          for (j = 0; j < num_posiciones; j++)
          {
            lseek(fd_dpram, DIR_BUFF_PMAC_PC + j, SEEK_SET);
            num_bytes_leidos = read(fd_dpram, buffer_pmac_pc + (j * NBYTES), NBYTES);
          }
          // printf("Respuesta pmac: %s.\n", buffer_pmac_pc);
          // Borrar palabra de control
          flag = 0;
          // lseek(fd_dpram, DIR_REG_YX_D1B4, SEEK_SET);
          // write(fd_dpram, &flag_send, NBYTES);
          escribir_ascii_dpram(fd_dpram, DIR_REG_YX_D1B4, &flag, NBYTES);
        }
      }
    }
  } while (!lectura_terminada);
}

int leerDatosServo(int fd_dpram, unsigned long dir_dpram, long long int* datos_servo, int tamanio_4_8)
{
  unsigned int estado_pmac = 0;
  int tamanio_leido = 0;
  char flag_pc = 1;
  // Cada palabra es de 32 bits.
  int palabra_baja = 0;
  int palabra_alta = 0;
  // Esperar mientras la PMAC esta ocupada actualizando los segmentos de memoria de
  // la DPRAM que contienen los datos de servo
  lseek(fd_dpram, DIR_CTRL_SERVO, SEEK_SET);
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
  // El pc ya puede leer.
  lseek(fd_dpram, dir_dpram, SEEK_SET);
  // Los datos de servo tienen una tama�o de 64 bits como mucho. (8 bytes).
  tamanio_leido = read(fd_dpram, (void *)&palabra_baja, NBYTES);
  //printf("pb: %#8X\n", palabra_baja);
  palabra_baja = 0x00FFFFFF & palabra_baja;
  //ROS_INFO("pb(%#X): %#8X\n", (unsigned int)dir_dpram, palabra_baja);
  if (tamanio_4_8)
  {
    lseek(fd_dpram, dir_dpram + 1, SEEK_SET);
    tamanio_leido = read(fd_dpram, (void *)&palabra_alta, NBYTES);
    //printf("pa: %#8X\n", palabra_alta);
  }
  *datos_servo = (palabra_alta << 24) + palabra_baja;
  //ROS_INFO("ds: %#X\n", *datos_servo);
  // El pc indica que ha terminado de leer.
  lseek(fd_dpram, DIR_CTRL_SERVO, SEEK_SET);
  flag_pc = 0;
  // Los datos de servo tienen una tama�o de 64 bits como mucho. (8 bytes).
  tamanio_leido = write(fd_dpram, (void *)&flag_pc, 1);
  // La funcion finaliza con exito.
  //   ROS_INFO("1 datos_servo: %#x.\n", 0x0000FFFF & *datos_servo);
  //   int zero_velocity = 0x00002000 & *datos_servo;
  //   zero_velocity = zero_velocity >> 13;
  //   ROS_INFO("zero_velocity: %d.\n", zero_velocity);

  return 0;
}

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

int leerEncoder(int fd_dpram, int num_motor, long long int* cuentas)
{
  unsigned long dir_dpram = 0;

  switch (num_motor)
  {
    // Motor #1 (Rueda derecha, la que esta en el lado del brazo)
    case 1:
      dir_dpram = 0x00000014;
      break;
      // Motor #2 (Base)
    case 2:
      dir_dpram = 0x00000023;
      break;
      // Motor #3 (Brazo-Munieca)
    case 3:
      dir_dpram = 0x00000032;
      break;
      // Motor #4 (Brazo-Hombro)
    case 4:
      dir_dpram = 0x00000041;
      break;
      // Motor #5 (Brazo-Munieca)
    case 5:
      dir_dpram = 0x00000050;
      break;
      // Motor #6 (Brazo-Hombro)
    case 6:
      dir_dpram = 0x0000005F;
      break;
      // Motor #7 (Brazo-Munieca)
    case 7:
      dir_dpram = 0x0000006E;
      break;
      // Motor #8 (Brazo-Codo)
    case 8:
      dir_dpram = 0x0000007D;
      break;
    default:
      printf("ERROR: Numero de motor invalido\n");
      return -1;
  }
  leerDatosServo(fd_dpram, dir_dpram, cuentas, 1);
  return 0;
}

//Funcion que el flag para el control de la trayectoria en modo
int leerflagPVT(int fd_dpram, int *m204)
{
  int tamanio_leido = 0;

  lseek(fd_dpram, 0x00000204, SEEK_SET);
  tamanio_leido = read(fd_dpram,(void *)m204, NBYTES);

  return 0;
}

// Lee el bit de VELOCIDAD ZERO de los motores del brazo
int leerMotorStatus(int fd_dpram, long long int* status, int num_artic_brazo)  
{                                                                                                                                  
  int i = 0;
  //  long long int aux;
  unsigned long dir_dpram = 0x00000039;

  for (i = 0; i < num_artic_brazo; i++)
  {
    leerDatosServo(fd_dpram, dir_dpram, status + i, 0);
    // printf("estado: %d\n", status[i] & 0x0FFFFF);
    status[i] = (status[i] & DES_VEL_ZERO) >> 13;
    // status[i] = (status[i] & MOV_RUNNING) >> 17;
    // unsigned int dir = (unsigned int)dir_dpram;
    // ROS_INFO("%d - LMS(%#X): %lld.\n",i + 3, dir, status[i]);
    dir_dpram = dir_dpram + 0x0F;
  }
  return 0;
}

// Lee la posicion, la velocidad actual y el error de seguimiento
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
  lseek(fd_dpram, DIR_CTRL_SERVO, SEEK_SET);
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
  }while (estado_pmac == 0x80000000);
    
  for (i = 0; i < NUM_MOTORES; i++)
  {    
	// El pc ya puede leer.
	lseek(fd_dpram, dir_dpram, SEEK_SET);
	tamanio_leido = read(fd_dpram, (void *)&palabra_baja, NBYTES);
	//printf("pb: %#8X\n", palabra_baja);
	//palabra_baja = 0x00FFFFFF & palabra_baja;
	posicion[i] = palabra_baja;
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
  lseek(fd_dpram, DIR_CTRL_SERVO, SEEK_SET);
  flag_pc = 0;
  // Los datos de servo tienen una tama�o de 64 bits como mucho. (8 bytes).
  tamanio_leido = write(fd_dpram, (void *)&flag_pc, 1);
  // La funcion finaliza con exito.
	
  return 0;
}

int leerPosicionRobot(int fd_dpram, float *x, float *y, float *theta)
{
  int estado_pmac = 0;
  int flag_estado = 0;
  int tamanio_leido = 0;

  // Esperar mientras la PMAC no ha terminado de actualizar la posicion
  // del robot en DPRAM.
  // Cuando la PMAC haya terminado de actualizar la posicion del robot en la DPRAM
  // indicara poniendo el bit 0 del registro Y:$D203 a 0.
  lseek(fd_dpram, 0x00000203, SEEK_SET);

  do
  {
    tamanio_leido = read(fd_dpram, &estado_pmac, NBYTES);
    flag_estado = 0x00000001 & estado_pmac;
  } while (flag_estado == 0x00000001);

  // El pc ya puede leer.
  lseek(fd_dpram, 0x00000200, SEEK_SET);
  // Los datos de interes pueden tener un tamanio de 4 u 8 bytes.
  tamanio_leido = read(fd_dpram, (void *)x, NBYTES);

  lseek(fd_dpram, 0x00000201, SEEK_SET);
  // Los datos de interes pueden tener un tamanio de 4 u 8 bytes.
  tamanio_leido = read(fd_dpram, (void *)y, NBYTES);

  lseek(fd_dpram, 0x00000202, SEEK_SET);
  // Los datos de interes pueden tener un tamanio de 4 u 8 bytes.
  tamanio_leido = read(fd_dpram, (void *)theta, NBYTES);

  // El pc indica que ha terminado de leer.
  lseek(fd_dpram, 0x00000203, SEEK_SET);
  flag_estado = 1;
  // Los datos de servo tienen una tamanio de 64 bits como mucho. (8 bytes).
  tamanio_leido = write(fd_dpram, (void *)&flag_estado, 1);
  // La funcion finaliza con exito.
  return 0;
}

/* Existe una libreria que tiene programado todas las funciones del puerto serie.
 int configura_puerto(int df,struct termios  *opciones){
 opciones->c_cflag |= (CLOCAL | CREAD);
 opciones->c_cflag &= ~CSIZE;
 opciones->c_cflag |= CS8;
 opciones->c_cflag &= ~PARENB;
 opciones->c_cflag &= ~CSTOPB;
 opciones->c_cflag &= ~CRTSCTS;
 opciones->c_oflag &= ~OPOST;
 opciones->c_iflag &=~(IXON|IXOFF|IXANY);
 opciones->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 cfsetispeed(opciones, B9600);
 cfsetospeed(opciones, B9600);
 tcsetattr(df, TCSAFLUSH, opciones);
 return(0);
 }
 */

/** La pinza deberia tener una libreria a parte.

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

int learn(int fd_dpram, int num_buffer)
{

  char buffer_pmac_pc[MAX_CARAC_BUFFER_PMAC_PC];
  char comando[MAX_CARAC_BUFFER_PC_PMAC];
  char apertura_buffer[] = {'o', 'p', 'e', 'n', ' ', 'p', 'r', 'o', 'g', '\0'};
  char learn[] = {'l', 'e', 'a', 'r', 'n', '\0'};
  char close[] = {'c', 'l', 'o', 's', 'e', '\0'};
  int error = 0;

  memset(comando, '\0', MAX_CARAC_BUFFER_PC_PMAC * sizeof(char));
  sprintf(comando, "open prog %d", num_buffer);
  comunicacion_dpram(fd_dpram, apertura_buffer, buffer_pmac_pc, &error);
  comunicacion_dpram(fd_dpram, learn, buffer_pmac_pc, &error);
  comunicacion_dpram(fd_dpram, close, buffer_pmac_pc, &error);

  return 0;
}

int teleoperacion(int fd, float vdesp, float vang)
{

  char g_buffer_pmac_pc[MAX_CARAC_BUFFER_PMAC_PC];
  char g_buffer_pc_pmac[MAX_CARAC_BUFFER_PC_PMAC];
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
  memset(g_buffer_pc_pmac, '\0', MAX_CARAC_BUFFER_PC_PMAC);
  memset(g_buffer_pmac_pc, '\0', MAX_CARAC_BUFFER_PMAC_PC);

  // Compruebo que la teleoperacion ha parado
  if (vdesp == 0 && vang == 0)
  {
    // Parar el robot
    comunicacion_dpram(fd, parar, g_buffer_pmac_pc, &error);
    // Restaurar valores iniciales.
    comunicacion_dpram(fd, vm1_vm2_orig, g_buffer_pmac_pc, &error);
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
  comunicacion_dpram(fd, i122, lectura, &error);
  comunicacion_dpram(fd, i222, lectura, &error);
  comunicacion_dpram(fd, arrancar, lectura, &error);

  return (ret);
}
