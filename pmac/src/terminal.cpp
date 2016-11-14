#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pmac/pmac.h"


/** Funcion que traduce los comandos de movimiento a instrucciones de la PMAC.
  * Los comandos de movimiento no son mas que unas instrucciones que me he inventado
  * cuya sintaxis es mas facil de escribir que la sintaxis de los comandos de movimiento
  * de la PMAC, y por lo tanto son faciles y rapidos de escribir, ya sea en un PC convencional,
  * en un portatil o en un dispositivo movil como un ipod o pda.
  * Listado de comandos:
  * Si quiero que la base del robot cese su movimiento escribo:         .00
  * Si quiero que la base del robot gire 90 grados horario:             .01 90
  * Si quiero que la base del robot gire 90 grados antihorario:         .01 -90
  * Si quiero que la base del robot avance en linea recta 100 cm        .02 100
  * Si quiero que la base del robot retoceda en linea recta 100 cm      .02 -100
  * Si quiero que el robot avance en linea recta indefinidamente        .03 +
  * Si quiero que el robot avance en linea recta indefinidamente        .03 -
  * Si quiero que la articulacion n ( 3 <= n <= 8) avance x cm          .n1 x
  * Si quiero que la articulacion n ( 3 <= n <= 8) retroceda x cm       .n1 -x
  * Si quiero que la articulacion n ( 3 <= n <= 8) cese su movimiento   .n0
  */
   void traductor(char *comandoTerminal, char *comandoPmac){

      int coef = 0;
      int cuentas = 0;
      int tipoMov = 0;
   // Un comando de movimiento comienza con el caracter '.', de no ser asi, el comando no es
   // de movimiento.
      if(comandoTerminal[0] != '.'){
         memcpy(comandoPmac, comandoTerminal, sizeof(char) * (strlen(comandoTerminal) + 1));
         return;
      }
      tipoMov = ((comandoTerminal[1] - '0') * 10) + (comandoTerminal[2] - '0');
   // printf("Tipo mov:%d.\n", tipoMov);
   // Parar movimiento.
   // Averiguar si el comando es de 'cese de movimiento'.
      if(comandoTerminal[2] == '0'){
      // Cese de movimiento de los motores 3, 4, 5, 6, 7, 8.
         switch(tipoMov){
            case 30: case 40: case 50: case 60: case 70: case 80:
               sprintf(comandoPmac, "#%dj/", comandoTerminal[1] - '0');
               break;
         // Cese de movimiento de la base, es decir, de los motores, 1 y 2.
            default:
               memcpy(comandoPmac, "#1j/#2j/", 8 * sizeof(char));
               break;
         }
      // printf("Comando PMAC: %s\n", comandoPmac);
         return;
      }
      // Giro angular de la base o de cualquier motor del brazo.
      // El giro angular de la base se proporciona en angulos, mientras
      // que el giro del resto de motores se proporciona en cm.
      else if(comandoTerminal[2] == '1'){
         switch(tipoMov){
         // CASOS DEL 31 AL 81 NO IMPLEMENTADOS. DEPENDIENDO
         // DEL MOTOR SE TIENE UNA CONSTANTE DIFERENTE PARA TRANSFORMAR
         // LOS CM EN CUENTAS DE ENCODER.
            case 31:
               coef = 0;
               break;
            case 41:
               coef = 0;
               break;
            case 51:
               coef = 0;
               break;
            case 61:
               coef = 0;
               break;
            case 71:
               coef = 0;
               break;
            case 81:
               coef = 0;
               break;
         // El giro de la base se proporciona en grados, y se debe transformar
         // en cuentas de encoder.
            default:
            // cuentas = atoi(comandoTerminal + 4) * 374.24;
               cuentas = atoi(comandoTerminal + 4) * 354.24;
               sprintf(comandoPmac, "#1j:%d#2j:%d", -cuentas, cuentas);
            // printf("Comando PMAC: %s\n", comandoPmac);
               return;
         }
         cuentas = atoi(comandoTerminal + 4) * coef;
         sprintf(comandoPmac, "#%dj:%d", comandoTerminal[1] - '0', cuentas);
      // printf("Comando PMAC: %s\n", comandoPmac);
         return;
      }
      // Moviemiento lineal de la base, bien de avance, bien de retroceso.
      else if(comandoTerminal[2] == '2'){
      // cuentas = atoi(comandoTerminal + 4)/0.00118883;
         cuentas = atoi(comandoTerminal + 4)/0.00118955;
         sprintf(comandoPmac, "#1j:%d#2j:%d", cuentas, cuentas);
      // printf("Comando PMAC: %s\n", comandoPmac);
         return;
      }
      // else que corresponde al caso comandoTerminal[2] = '3', es decir, movimiento
      // lineal de avance o de retroceso indefinido.
      else{
         sprintf(comandoPmac, "#1j%c#2j%c", comandoTerminal[4], comandoTerminal[4]);
      // printf("Comando PMAC: %s\n", comandoPmac);
         return;
      }
   }


   int main (int argc __attribute__ ((unused)), char **argv __attribute__ ((unused))){
      long long cts = 0;
      int error = 0;
      int fd_dpram = 0;

      char buffer_pmac_pc[NUM_MAX_CARAC_BUFFER_DPRAM_PC];
      char comandoTerminal[NUM_MAX_CARAC_BUFFER_PC_DPRAM_1];
      char comandoPmac[NUM_MAX_CARAC_BUFFER_PC_DPRAM_1];

      system("clear");
   // Apertura del fichero de dispositivo vinculado a la DPRAM.
      if( (fd_dpram = open(DPRAM, O_RDWR)) == -1 ){
         printf("No se puede abrir %s.\n", DPRAM);
         return -1;
      }

      printf("\33[0;33m-----------------------------------DPRAM DETECTADA => TERMINAL OPERATIVO-----------------------------------\33[0;30m\n");
      printf("\33[0;34m*******************************\n");
      system("date");
      printf("*******************************\33[0;33m\n");
      printf("\33[1;31mRecuerde enviar comando ctrl-k por seguridad\33[0;30m\n");

      do{
      // Limipeza de los buffer.
         memset(comandoTerminal, '\0', NUM_MAX_CARAC_BUFFER_PC_DPRAM_1);
         memset(comandoPmac, '\0', NUM_MAX_CARAC_BUFFER_PC_DPRAM_1);
         memset(buffer_pmac_pc, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC);
         error = 0;
         printf("\n");
         printf("\33[0;32mIntroduzca instruccion:\33[0;30m\n");
      // Lectura del comando que el usuario introduce por el terminal.
         gets(comandoTerminal);
         printf("numCaracteres:%d.\n", strlen(comandoTerminal));
         comandoTerminal[strlen(comandoTerminal)] = '\0';
         printf("\n");
      // Traducir el comando si fuera necesario.
         traductor(comandoTerminal, comandoPmac);
      // Comunicarle el comando a la PMAC a traves de la DPRAM.
         printf("Comando PMAC: %s\n", comandoPmac);
         printf("Comando PMAC: %#X\n", comandoPmac[0]);
         
         comunicacionASCII(fd_dpram, comandoPmac, buffer_pmac_pc, &error);		  
	     
         if(error != 0){
            printf("\33[1;31mERROR\33[0;30m: ");
            switch(error){
               case 3:
                  printf("\33[0;31mComando no valido\33[0;30m\n");
                  break;
               case 5:
                  printf("\33[0;31mComando no permitido sin tener un buffer abierto\33[0;30m\n");
                  break;
               case 11:
                  printf("\33[0;31mMovimiento anterior no completado\33[0;30m\n");
                  break;
               case 18:
                  printf("\33[0;31mUno o mas motores del SC esta desactivado\33[0;30m\n");
                  break;
               default:
                  printf("\33[0;31mError desconocido\33[0;30m\n");
            }
         }
         else{
            if(strlen(buffer_pmac_pc) == 0){
               memset(buffer_pmac_pc, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC);
               memcpy(buffer_pmac_pc, "Sin respuesta", sizeof("Sin respuesta"));
            }
            printf("%s\n", buffer_pmac_pc);
         }        
         
      }while(comandoTerminal[0]!='q' &&  comandoTerminal[0] != 'Q' );
      printf("\33[0;33m-------------------------------------------------FIN SESION-------------------------------------------------\33[0;30m\n");

   // printf("\n");
   // read_enc(fd_dpram, 1, &cts);
   // printf("M1 cts: %lld.\n", cts);
      cts = 0;
   // read_enc(fd_dpram, 2, &cts);
   // printf("M2 cts: %lld.\n", cts);

      close(fd_dpram);
      return 0;
   }
