
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pmac/pmac.h"
#include <time.h>

void wait()
{
  char aux;
  printf("Press ENTER to continue...\n");
  scanf("%c",&aux);
}

int main(int argc, char** argv)
{
     float pos_leida[NUM_MOTORES];
     float velocidad_leida[NUM_MOTORES];
     float error_leido[NUM_MOTORES];
     //int i = 0;
     int fd_dpram = 0;
     int error_pmac = 0;
     //char buffer_pmac_pc[MAX_CARAC_BUFFER_PMAC_PC];
     if( (fd_dpram = open(DPRAM, O_RDWR)) == -1 ){
      printf("No se puede abrir %s.\n", DPRAM);
      return -1;
     }
  //   if((posicion_cont = fopen("posicion_cont.dat", "wt")) == NULL){
  //        printf("Error al abrir el fichero, los datos no seran guardados.\n");
  //   }

     while(1){
          //memset(pos_leida, '\0', NUM_ARTIC_BRAZO * sizeof(float));
          //memset(vel_leida, '\0', NUM_ARTIC_BRAZO * sizeof(float));
          error_pmac = leerInfoMotores(fd_dpram, pos_leida, velocidad_leida, error_leido);
   //       fprintf(posicion_cont, "%f   %f   %f   %f   %f   %f\n", pos_leida[0],pos_leida[1],pos_leida[2],pos_leida[3],pos_leida[4],pos_leida[5]);
          //fprintf(posicion_cont, "%f   %f   %f   %f   %f   %f\n", vel_leida[0],vel_leida[1],vel_leida[2],vel_leida[3],vel_leida[4],vel_leida[5]);
          //printf("m3: %f\t%f\t%f\nm4: %f\t%f\t%f\nm5: %f\t%f\t%f\nm6: %f\t%f\t%f\nm7: %f\t%f\t%f\nm8: %f\t%f\t%f\nm1: %f\t%f\t%f\nm2: %f\t%f\t%f\n\n", 
          printf("m1: %f\t%f\t%f\nm2: %f\t%f\t%f\nm3: %f\t%f\t%f\nm4: %f\t%f\t%f\nm5: %f\t%f\t%f\nm6: %f\t%f\t%f\nm7: %f\t%f\t%f\nm8: %f\t%f\t%f\n\n", 
                pos_leida[0], velocidad_leida[0], error_leido[0],
				pos_leida[1], velocidad_leida[1], error_leido[1],
				pos_leida[2], velocidad_leida[2], error_leido[2],
				pos_leida[3], velocidad_leida[3], error_leido[3],
				pos_leida[4], velocidad_leida[4], error_leido[4],
				pos_leida[5], velocidad_leida[5], error_leido[5],
				pos_leida[6], velocidad_leida[6], error_leido[6],
				pos_leida[7], velocidad_leida[7], error_leido[7]);
          
          wait();
     }
   return 0;
}
