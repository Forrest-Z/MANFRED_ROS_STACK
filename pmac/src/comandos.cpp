//#include <unistd.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <string>
#include <time.h>
#include "pmac/pmac.h"

int main(int argc, char *argv[])
{

  int descriptorFich = 0;

  int errorPMAC = 0;

  char datosPMACaPC[NUM_MAX_CARAC_BUFFER_DPRAM_PC];

  char *comando = (char *)calloc(100, sizeof(char));

  //char comandoGather[7] = {'G', 'A', 'T', 'H', 'E', 'R', '\0'};
  //std::string enable = "ENABLE PLC 1";
  if ((descriptorFich = open(DPRAM, O_RDWR)) == -1)
  {
    printf("\x1B[31;1m");

    printf("NO SE PUEDE ABRIR EL DISPOSITIVO: %s.\n", DPRAM);

    printf("\x1B[0m");

    return -1;
  }

  if (apagarOEncenderMotores(descriptorFich, 1, datosPMACaPC, &errorPMAC))
  {
    printf("\x1B[31;1m");

    printf("ERROR ENCENDIENDO LOS MOTORES\n");

    printf("\x1B[0m");

    return -1;
  }

  // Se envía el comando GATHER para poder activar el 'Backgroung Fixed Data' en la DPRAM.
  sprintf(comando, "GATHER");

  printf("%s\n", comando);

  if (comunicacionASCII(descriptorFich, comando, datosPMACaPC, &errorPMAC))
  {

    printf("\x1B[31;1m");

    printf("ERROR AL COMUCARSE CON LA PMAC\n");

    printf("\x1B[0m");

    return -1;

  }

  // Programa inicializador
  memset(datosPMACaPC, '\0', sizeof(char));
  sprintf(comando, "&1B8R");
  printf("%s\n", comando);

  if (comunicacionASCII(descriptorFich, comando, datosPMACaPC, &errorPMAC))
  {
    printf("\x1B[31;1m");
    printf("ERROR AL COMUCARSE CON LA PMAC\n");
    printf("\x1B[0m");
    return -1;
  }

  /*
   memset(datosPMACaPC, '\0', sizeof(char));
   sprintf(comando, "ENABLE PLC 3");
   printf("%s\n", comando);
   comunicacionASCII(descriptorFich, comando, datosPMACaPC, &errorPMAC);
   memset(datosPMACaPC, '\0', sizeof(char));
   sprintf(comando, "ENABLE PLC 4");
   printf("%s\n", comando);
   comunicacionASCII(descriptorFich, comando, datosPMACaPC, &errorPMAC);
   */
  //El parametro de usleep es: (unsigned int) microsegundos
  //usleep(10000);
  return 0;
}
