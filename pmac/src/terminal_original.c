/*-----------------------------------------------------------------------------------------------
    Programas que emula el terminal de PWIN reconociendo errores,imprimiendo respuestas y 
ejecutando acciones.     
-----------------------------------------------------------------------------------------------*/


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

#define NBYTES 4

//DEFINICION DE FUNCIONES


   int ascii_read(int, int , int *, int);
   int ascii_write(int, int, char *, int);
   int ascii_dflag();


//FUNCIÓN DE LECTURA
    int ascii_read(int fd, int address, int *buff, int size){
   
      int tam;
      lseek(fd, address, SEEK_SET);
      tam = read(fd, buff, size);
      return tam;
   
   }
//FUNCIÓN DE ESCRITURA
    int ascii_write(int fd, int address, char *buff, int size){
   
      int tam;
      lseek(fd, address, SEEK_SET);
      size = write(fd, buff, size);
      return tam;
   }


//MAIN
    int main (int argc __attribute__ ((unused)), char **argv __attribute__ ((unused))){
   
   
   
   //variables para apertura dispositivo
      int fd;
      char *device = "/dev/dpram_driver_2010";
   
   //variables para leer y escribir en la dpram
      int size, flag_send, flag_receive=0, ctrl=0 , longitud=0;
      char *cmd="/0";
      char lectura[4];
      char lectura2[159];
      char cadena[159];
      int  terminada_lectura=1;
      int numero_bytes=0;
   
   //contadores
      int j=0,i=0;
   
   //dacodificacion de errores
      int error=0,error2=0;
   
   //variables para mostrar por pantalla datos
      char *caracter;
   
      system("clear");
   
   //********************* Opening DPRAM read/write....********************************************/
   
      fd = open(device, O_RDWR);
      if(fd < 0){
      
         perror("DPRAM:");
         printf("\nDPRAM::CAN'T OPEN DEVICE\n");
         exit(EXIT_FAILURE);
      }
   
      printf("\33[0;33m-----------------------------------DPRAM DETECTADA => TERMINAL OPERATIVO-----------------------------------\33[0;30m\n"); 
      printf("\33[0;34m*******************************\n"); 
      system("date");
      printf("*******************************\33[0;33m\n"); 
      printf("\33[1;31mRecuerde enviar comando ctrl-k por seguridad\33[0;30m\n"); 
   /************************************************************************************************/
   
   
   
   /**********************************MENU TERMINAL*************************************************/
      cadena[0]='0';
      while(cadena[0]!='q' &&  cadena[0] != 'Q' ){       //caracter de salida 
      
         terminada_lectura=1; //Flag de control para que se produzca una lectura por intrusccion
      
         printf("\33[0;32mIntroduzca instruccion:\33[0;30m\n");
         gets(cadena);
      
      /************************************************************************************************/
      
      
      
      //********************************ESCRIBIR EN LA DPRAM******************************************
      
      
      //LIMPIO RESPUESTA Y FLAG DE ESCRITURA
      
         flag_send = 0;
         printf("Llamo a lseek 'terminal': %x.\n", 0x018B);
         lseek(fd, 0x018B, SEEK_SET);
         write(fd, &flag_send, NBYTES);
         lseek(fd, 0x01B4, SEEK_SET);
         write(fd, &flag_send, NBYTES);
      
      
      //Envio cadena de caracters
      
      //Compruebo en primer lugar si se trata de un comando tipo Ctrl^
      
         if(cadena[1]=='-' || cadena[2]=='-'){
         
            ctrl=atoi(cadena); //Paso de código ASCCII a valor entero 
            lseek(fd, 0x018C, SEEK_SET);
            write(fd, &ctrl, NBYTES);
         
            size = (strlen(cmd)+ 1)*sizeof(char);  //Escribo el '/0'
            ascii_write(fd, 0x018D, cmd, size);   
         
         }
         //Si no se trata de un comando Ctrl^ trato el comando como una cadena de carácteres en ASCII
         else{
         
            size = (strlen(cadena)+ 1)*sizeof(char);  
            ascii_write(fd, 0x018C, cadena, size);  
         
         }
      
      // Mando flag para indicar a la PMAC que he escrito
      
         flag_send = 1;
         lseek(fd, 0x018B, SEEK_SET);
         write(fd, &flag_send, NBYTES);
      /**************************************************************************************************/
      
      /******************************LECTURA DPRAM*******************************************************/
      
      
         while(terminada_lectura>0){
         
         
            lseek(fd, 0x01B4, SEEK_SET);
            read(fd, &lectura, NBYTES);
         
            flag_receive=lectura[0];
         
            if(flag_receive > 0){   //¿Hay respuesta?
            
            /***************************Modo prueba para identificacion de errores***************************************/
            //printf(" leido en pos 0: %d\n",lectura[0]);//Si es >0 indica que hay  respuesta de la PMAC
            // printf(" leido en pos 1: %d\n",lectura[1]);
            // printf(" leido en pos 2: %d\n",lectura[2]);//Aqui se guarda el numero de bytes de la respuesta
            // printf(" leido en pos 3: %d\n",lectura[3]);
            //En lectura[0] es donde se encuentra el error codificado en BCD de la forma 8d/dd y los errores en pag 23
            //del software manual
            ///***********************************************************************************************************/
            
            /****************************TRATAMIENTO DE ERROR*********************************************/
            //codifico en decimal los errores
               error=(int)lectura[0];
               error2=(int)lectura[1];
            
               if(error2==-128){  //¿Existió un error?
               
                  printf("\33[1;31mERROR\33[0;30m\n"); //Si se produjo error decodifico de que error se trata  
                  switch(error){
                  
                     case 5: 
                        printf("\33[0;31mComando no permitido sin tener un buffer abierto\33[0;30m\n");
                        break;
                     case 11: 
                        printf("\33[0;31mMovimiento anterior no completado\33[0;30m\n");
                        break;
                     case 18:
                        printf("\33[0;31mUno o mas motores del SC esta desactivado\33[0;30m\n");
                        break;
                     case 3:
                        printf("\33[0;31mComando no valido\33[0;30m\n");
                        break;
                     default:
                        printf("\33[0;31mError desconocido\33[0;30m\n");
                  }
               
               }
            
            
            /***************************************************************************************************/
               numero_bytes=(int)lectura[2]; //paso a entero el numero de bytes a leer para la función read
            
               lseek(fd, 0x01B5, SEEK_SET);
               read(fd, &lectura2,numero_bytes);
               caracter=malloc(numero_bytes*(sizeof(char))); //Reservo memoria dinamica
            
            //bucle para leer el numero de caracteres enviados en la respuesta de la dpram
               for(i=0;i<lectura[2];i++){
               
                  caracter[i]=lectura2[i];     
               
               }
               printf("\33[0;34m%s\33[0;30m\n",caracter); //Muestro por pantalla respuesta PMAC      
            
            
            
            //Borro respuesta
            
               for(j=0;j<lectura[2]%4;j++){
                  flag_send = 0;
                  lseek(fd, 0x01B5+j, SEEK_SET);
                  write(fd, &flag_send, NBYTES);
               }
            
            //Borro palabra de control
               flag_send = 0;
               lseek(fd, 0x01B4, SEEK_SET);
               write(fd, &flag_send, NBYTES);
            
               terminada_lectura=0;  //Flag de salida del modo lectura
            }
         
         }
      
      }
   
      printf("\33[0;33m-------------------------------------------------FIN SESION-------------------------------------------------\33[0;30m\n");
   
   //Cerrar dispositivo y liberar memoria dinamica reservada
   
      free(caracter); 
      close (fd);
      return 0;
   }
