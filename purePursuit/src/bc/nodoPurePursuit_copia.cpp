// C/C++
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

// ROS
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialAction.h"
#include "nav_msgs/Odometry.h"
#include "odometria/servicioOdometria.h"
#include "pmac.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

// OpenMRL
#include "datatype/gridmap_includes.h"
#include "datatype/geometry/path2d.h"
#include "algorithms/planning/fm2pathplanner.h"

nav_msgs::Odometry *p_msj_odometria = NULL;
bool flag_topic_odometria_recibido = false;

void obtenerOdometria(const nav_msgs::Odometry::ConstPtr& rp_msj_odometria_rx)
{

  *p_msj_odometria = *rp_msj_odometria_rx;
  flag_topic_odometria_recibido = true;

}

#define PRUEBA
#define DEBUG

// Constante 2*PI;
static const double dosPi = 2.0f * M_PIl;
// Constante que pasa de grados a radianes.
static const double gradRad = 180.0f * M_1_PIl;
// Constante que pasa de radianes a grados.
static const double radGrad = M_PIl / 180.0f;
// Estructura que almacena informacion util del algoritmo pure pursuit.
struct algoritmoPurePursuit
{
  // Radio de la base, en cm.
  double radioBase;
  // Radio de curvatura, en cm, a partir del cual se considera que el robot se desplaza en linea recta.
  double radioUmbralLinRecta;
  // Radio de curvatura, en cm, con el que gira la base.
  double radioConSignoFinal;
  // Tiempo que puede durar como mucho una curva S, en cualquiera de la dos ruedas. En ms.
  int tiempoCurvaSMax;
  // Tiempo en desplazamiento en ms.
  int tiempoDesplazamiento;
  // Velocidad maxima de la rueda en cm/s.
  double velRuedaMax;
  // Aceleracion maxima de la rueda en cm/s^2.
  double acelRuedaMax;
  // Velocidad angular de la base en grad/s.
  double velAngBaseMax;
  // Velocidad de la rueda derecha al final del tramo actual, en cm/s.
  double velRuedaDerFinal;
  // Velocidad de la rueda izquierda al final del tramo actual, en cm/s.
  double velRuedaIzqFinal;
  // Tiempo, en ms, que dura la mitad de la curva S de velocidad de la rueda derecha. En ms.
  int tiempoMediaCurvaSRuedaDer;
  // Tiempo, en ms, que dura la mitad de la curva S de velocidad de la rueda izquierda. En ms.
  int tiempoMediaCurvaSRuedaIzq;
};
typedef struct algoritmoPurePursuit algoritmoPurePursuit;

int escribirDPRAMVelocidadFinalTiempoMediaCurvaS(const int & descriptorDPRAM, const algoritmoPurePursuit & algPP);

int sign(double numero)
{
  // numero < 0 --> return -1
  // numero = 0 --> numero > 0 es false, false equivale a 0, por tanto return 0.
  // return > 0 --> numero > 0
  return numero < 0 ? -1 : numero > 0; // # -1, 0, +1.
}

int rotacion(const int & descriptorDPRAM, const double & xRobotSRG, const double & yRobotSRG, const double & tRobotSRG,
             const double * puntoTrayectSRG, algoritmoPurePursuit & algPP)
{
// Comprobar errores.
  if (algPP.velAngBaseMax <= 0 || algPP.acelRuedaMax <= 0 || algPP.radioBase <= 0)
  {
    return -1;
  }
  // La PMAC trabaja con numeros reales de 32 bits, es decir, float.
  float comandosPDer[2] = {0.0f, 0.0f};
  float comandoVDer = 0.0f;
  int comandosT[2] = {0, 0};
  double deltaX = puntoTrayectSRG[0] - xRobotSRG;
  double deltaY = puntoTrayectSRG[1] - yRobotSRG;
  double orientacionTramo = atan2f(deltaY, deltaX);
// Angulo a rotar por el robot. Entre [-pi, pi].
  double deltaTheta = orientacionTramo - tRobotSRG;
  if (deltaTheta == 0)
  {
    // El robot no rota.
    return -1;
  }
// deltaTheta debe estar entre -pi y pi.
  double signoDeltaThetaDosPi = sign(deltaTheta) * dosPi;
  if (fabs(deltaTheta) > dosPi)
  {
    // Si el valor fabs(deltaTheta) esta entre 0 y 2*pi el valor retornado por la operacion
    // fmodf(deltaTheta, signoDeltaThetaDosPi) es el valor de su primer argumento, es decir,
    // deltaTheta.
    // Si el valor de fabs(deltaTheta) es mayor que 2*pi, entonces la operacion fmod retorna
    // el angulo equivalente entre 0 y 2*pi.
    deltaTheta = fmod(deltaTheta, signoDeltaThetaDosPi);
  }
  // Finalmente hacer que el valor de deltaTheta este entre -pi y pi.
  if (fabs(deltaTheta) > M_PI)
  {
    deltaTheta -= signoDeltaThetaDosPi;
  }
  int signoDeltaTheta = sign(deltaTheta);
  // Distancia que tiene que recorrer cada rueda. En cm.
  double arco = deltaTheta * algPP.radioBase;
  double arcoValorAbsoluto = fabs(arco);
  // Pasar la velocidad angular de la base de grad/s a rad/s.
  double velAngBaseMax = radGrad * algPP.velAngBaseMax;
  // floorf para no exceder la velocidad angular maxima de la base. Dividir y multiplicar por 10
  // para usar un decimal en la velocidad de las ruedas. En cm/s.
  double velRuedaDerRotMax = floorf(10.0f * velAngBaseMax * algPP.radioBase) / 10.0f;
  // Tiempo en pasar de 0 cm/s a (velRuedaDerRotMax/2) cm/s usando la aceleracion maxima. La funcion
  // ceilf es para asegurar un numero entero (la PMAC necesita que el tiempo que dura un comando PVT
  // sea un numero entero) y que no se exceda la aceleracion. En ms.
  int tiempoMediaCurvaSRot_1 = ceilf(1000.0f * velRuedaDerRotMax / algPP.acelRuedaMax);
  //double acelRuedaRotMaxReal = velRuedaDerRotMax/(tiempoMediaCurvaSRot_1/1000.0f);
  // El numero 1000.0f pasa al numerador.
  double acelRuedaRotMaxReal = 1000.0f * velRuedaDerRotMax / tiempoMediaCurvaSRot_1;
  // Desplazamiento realizado en el tiempo que dura la primera curvaS. En cm.
  double desplazRot_1 = velRuedaDerRotMax * (tiempoMediaCurvaSRot_1 / 1000.0f);
  // Desplazamiento si solo considero: CurvaS-CurvaS.
  double desplazRot2CurvaS = 2 * desplazRot_1;
#ifdef DEBUG
  printf("puntoTrayectSRG: (%f, %f) \n", puntoTrayectSRG[0], puntoTrayectSRG[1]);
  printf("deltaX: %f m \n", deltaX);
  printf("deltaY: %f m \n", deltaY);
  printf("orientacionTramo: %f grad \n", gradRad * orientacionTramo);
  printf("deltaTheta: %f grad \n", gradRad * deltaTheta);
  printf("arco: %f cm (+ rotacion antihoraria, - rotacion horaria) \n", arco);
  printf("velAngBaseMax: %f rad/s \n", velAngBaseMax);
  printf("velRuedaDerRotMax: %f cm/s \n", velRuedaDerRotMax);
  printf("tiempoMediaCurvaSRot_1: %d ms \n", tiempoMediaCurvaSRot_1);
  printf("acelRuedaRotMaxReal: %f cm/s^2 \n", acelRuedaRotMaxReal);
  printf("desplazRot_1: %f cm \n", desplazRot_1);
  printf("desplazRot2CurvaS: %f cm \n", desplazRot2CurvaS);
#endif
  if (velRuedaDerRotMax == 0 || tiempoMediaCurvaSRot_1 == 0)
  {
    return -1;
  }
  if (arcoValorAbsoluto > desplazRot2CurvaS)
  {
    // Tiempo que dura la mitad de la primera curva S. Numero entero en ms.
    comandosT[0] = tiempoMediaCurvaSRot_1;
    // Recorrido con signo. En cm y con decimales.
    comandosPDer[0] = static_cast<float>(signoDeltaTheta * desplazRot_1);
    // Velocidad con 1 decimal. En cm/s.
    comandoVDer = static_cast<float>(signoDeltaTheta * velRuedaDerRotMax);
    // Recorrido con signo realizado durante el segundo tramo.
    // El segundo tramo se recorre a velocidad constante, que es
    // la velocidad final del tramo 1.
    double desplazRot_2 = arcoValorAbsoluto - desplazRot2CurvaS;
#ifdef DEBUG
    printf("desplazRot_2: %f cm \n", desplazRot_2);
#endif
    // round porque no tengo que satisfacer ninguna restriccion, solo que el tiempo sea
    // numero entero. La PMAC necesita numeros enteros para el tiempo. En ms.
    // int tiempoMediaCurvaSRot_2 = round(1000 * desplazRot_2/(2*velRuedaDerRotMax));
    // 1000/2 = 500
    // round(X) equivale a floorf(X + 0'5)
    int tiempoMediaCurvaSRot_2 = static_cast<int>(round(500.0f * (desplazRot_2 / velRuedaDerRotMax)));
    // Recalcular el valor de desplazRot_2, ya que tiempoMediaCurvaSRot_2 ha sido redondeado.
    // desplazRot_2 = 2 * velRuedaDerRotMax*(tiempoMediaCurvaSRot_2/1000.0f);
    // 2/1000  = 2*10^-3 = 2.0e-3
    desplazRot_2 = 2.0e-3 * velRuedaDerRotMax * tiempoMediaCurvaSRot_2;
    comandosT[1] = tiempoMediaCurvaSRot_2;
    comandosPDer[1] = static_cast<float>(signoDeltaTheta * desplazRot_2);
#ifdef DEBUG
    printf("tiempoMediaCurvaSRot_2: %d s.\n", tiempoMediaCurvaSRot_2);
    printf("desplazRot_2 corregido: %f cm.\n", desplazRot_2);
#endif
  }
  else if (arcoValorAbsoluto < desplazRot2CurvaS)
  {
    // Uso acelRuedaRotMaxReal en lugar de acelRuedaMax, porque la primera acel (acelRuedaRotMaxReal) es la
    // que se obtuvo al usar velRuedaDerRotMax y tiempoMediaCurvaSRot_1.
    // Al usar el mismo valor de acel junto con un arco que es menor que
    // desplazRot2CurvaS me aseguro que la vel hallada es menor que la
    // velRuedaDerRotMax.
    double velRuedaDer = floorf(10.0f * sqrt((arcoValorAbsoluto * acelRuedaRotMaxReal) / 2.0f)) / 10.0f;
    int tiempoMediaCurvaS = ceilf(1000.0f * velRuedaDer / algPP.acelRuedaMax);
    double desplazamiento = velRuedaDer * (tiempoMediaCurvaS / 1000.0f);
    // Primer valor: tiempo que dura la mitad de la primera curva S. Numero entero en ms.
    // Segundo valor: cero porque NO hay tramo a velocidad constante.
    // Tercer valor: tiempo que dura la mitad de la segunda curva S. Numero entero en ms.
    comandosT[0] = tiempoMediaCurvaS;
    comandosT[1] = 0;
    // Recorridos con signo. En cm.
    comandosPDer[0] = static_cast<float>(signoDeltaTheta * desplazamiento);
    comandosPDer[1] = 0.0f;
    // Velocidad de la rueda derecha en cm/s.
    comandoVDer = static_cast<float>(signoDeltaTheta * velRuedaDer);
  }
// arco = desplazMax
  else if (arcoValorAbsoluto == desplazRot2CurvaS)
  {
    // Primer valor: tiempo que dura la mitad de la primera curva S. Numero entero en ms.
    // Segundo valor: cero porque NO hay tramo a velocidad constante.
    // Tercer valor: tiempo que dura la mitad de la segunda curva S. Numero entero en ms.
    comandosT[0] = tiempoMediaCurvaSRot_1;
    comandosT[1] = 0;
    comandosPDer[0] = static_cast<float>(signoDeltaTheta * desplazRot_1);
    comandosPDer[1] = 0.0f;
    comandoVDer = static_cast<float>(signoDeltaTheta * velRuedaDerRotMax);
  }
  double deltaThetaReal = (comandosPDer[0] + comandosPDer[1] + comandosPDer[0]) / algPP.radioBase;
#ifdef DEBUG
  printf("\n");
  printf("deltaThetaReal: %f grados \n", gradRad * deltaThetaReal);
  printf("\n");
  printf("PVT(%d) \n", comandosT[0]);
  printf("X(%f):%f Y(%f):%f \n", comandosPDer[0], comandoVDer, -comandosPDer[0], -comandoVDer);
  if (comandosT[1])
  {
    printf("PVT(%d) \n", comandosT[1]);
    printf("X(%f):%f Y(%f):%f \n", comandosPDer[1], comandoVDer, -comandosPDer[1], -comandoVDer);
  }
  printf("PVT(%d) \n", comandosT[0]);
  printf("X(%f):%f Y(%f):%f \n", comandosPDer[0], 0.0f, -comandosPDer[0], 0.0f);
#endif

  int error = 0;
  char datosParaPC[NUM_MAX_CARAC_BUFFER_DPRAM_PC];
  memset(datosParaPC, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC * sizeof(char));
  if (escribirRotacionBaseDPRAM(descriptorDPRAM, comandosPDer, &comandoVDer, comandosT, datosParaPC, &error))
  {
    std::cout << "NO SE HA PODIDO ROTAR" << std::endl;
  }
  if (error != 0)
  {
    std::cout << "ERROR INDICADO POR LA PMAC: " << error << std::endl;
  }
  //printf("datosParaPC: %s.\n", datosParaPC);
  int bitVelocidadCeroMotor1 = -1;
  // EL bit de velocidad 0 vale 1 porque el robot esta parado. En el momento que
  // se mueve se pone a 1. Cuando el robot se para se pone a 1.
  do
  {
    if (leerEstadoMotor(descriptorDPRAM, 1, &bitVelocidadCeroMotor1))
    {
      return -1;
    }
  } while (bitVelocidadCeroMotor1 != 0);
  do
  {
    if (leerEstadoMotor(descriptorDPRAM, 1, &bitVelocidadCeroMotor1))
    {
      return -1;
    }
  } while (bitVelocidadCeroMotor1 != 1);
  return 0;
}

int actualizarVelFinalRuedas(const int & descriptorDPRAM, algoritmoPurePursuit & algPP)
{

  double radioConSignoFinalValorAbs = fabs(algPP.radioConSignoFinal);
  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;
  double vel = 0.0f;
  int flagActualizarVelocidades = 0;
  // He controlado fuera de esta funcion que el desplazamiento sea siempre hacia delante forzando que
  // distanciaPP >= diametroBase, por lo tanto nunca deberia ocurrir que radioConSignoFinal < radioBase,
  // aun asi se dispone de esta condicion por si fuese necesario.
  if (radioConSignoFinalValorAbs < algPP.radioBase)
  {
    printf("\x1B[31;1m");
    printf("Radio de giro menor que el radio de la base.\n");
    printf("\x1B[0m");
    algPP.radioConSignoFinal = sign(algPP.radioConSignoFinal) * algPP.radioBase;
    radioConSignoFinalValorAbs = algPP.radioBase;
  }
  // Movimiento rectilineo + Giro.
  if (radioConSignoFinalValorAbs < algPP.radioUmbralLinRecta)
  {
    double cocienteRadios = algPP.radioBase / algPP.radioConSignoFinal;
    // Se calcular una velocidad 'inicial' que tiene una dependencia lineal con el
    // radio de giro.
    vel = (algPP.velRuedaMax / algPP.radioUmbralLinRecta) * radioConSignoFinalValorAbs;
    // Giro antihorario (+)
    if (algPP.radioConSignoFinal > 0)
    {
      std::cout << "+++++ Mov_Rect + Giro_AntiH. vel: " << vel << " +++++" << std::endl;
      // Velocidad con un decimal.
      algPP.velRuedaDerFinal = floorf(10 * vel) / 10;
      algPP.velRuedaIzqFinal = floorf(10 * ((1 - cocienteRadios) / (1 + cocienteRadios)) * algPP.velRuedaDerFinal) / 10;
    }
    // Giro horario (-).
    else
    {
      std::cout << "+++++ Mov_Rect + Giro_H. vel: " << vel << " +++++" << std::endl;
      algPP.velRuedaIzqFinal = floorf(10 * vel) / 10;
      algPP.velRuedaDerFinal = floorf(10 * ((1 + cocienteRadios) / (1 - cocienteRadios)) * algPP.velRuedaIzqFinal) / 10;
    }
  }
  // Movimiento rectilineo: radioConSignoFinalValorAbs >= algPP.radioUmbralLinRecta
  else
  {
    std::cout << "___Mov_Rect___" << std::endl;
    algPP.velRuedaDerFinal = algPP.velRuedaMax;
    algPP.velRuedaIzqFinal = algPP.velRuedaMax;
  }
  algPP.tiempoMediaCurvaSRuedaDer = ceilf(
      1000 * fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / algPP.acelRuedaMax);
  algPP.tiempoMediaCurvaSRuedaIzq = ceilf(
      1000 * fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / algPP.acelRuedaMax);
#ifdef DEBUG
  printf("\n");
  printf("vD: %.3f cm/s --> %.3f cm/s \n", velRuedaDerInicial, algPP.velRuedaDerFinal);
  printf("tMCSD: %d ms \n", algPP.tiempoMediaCurvaSRuedaDer);
  printf("vI: %.3f cm/s --> %.3f cm/s \n", velRuedaIzqInicial, algPP.velRuedaIzqFinal);
  printf("tMCSI: %d ms \n", algPP.tiempoMediaCurvaSRuedaIzq);
#endif
  int tiempoMediaCurvaSMax = algPP.tiempoCurvaSMax / 2;
  // Cuando la rueda debe mantener la velocidad constante, no hay cambio entre la orden de control
  // anterior y la nueva orden de control (Vf = Vi), el tiempo de desplazamiento calculado para esa rueda
  // es de 0 ms. Este resultado es debido a que se ha calculado este tiempo con la formula de
  // un perfil de velocidad de tipo curva S. tiempoMed... = ceil(Vf - Vi/acelMax). Obviamente un
  // tiempo de 0 ms no es correcto, lo que ocurre es que la rueda se debe mover durante to_do
  // el tiempo que dura tiempoCurvaSMax a la misma velocidad.
  // Deberia ser == 0 pero debido al redondeo donde debiera aparecer un 0 a veces apareace un 1.
  // De ahí que se ponga un <= 1
  if (algPP.tiempoMediaCurvaSRuedaDer <= 1)
  {
    // NOTA: Si un perfil de velocidad de tipo curva S, la velocidad inicial y final
    // son iguales, la curva S se convierte en un tramo recto.
    algPP.velRuedaDerFinal = velRuedaDerInicial;
    algPP.tiempoMediaCurvaSRuedaDer = tiempoMediaCurvaSMax;
#ifdef DEBUG
    printf("\n");
    printf("tMCSD = 0 ms -- > %d ms \n", algPP.tiempoMediaCurvaSRuedaDer);
    printf("vDF = %f cm/s \n", algPP.velRuedaDerFinal);
#endif
  }
  else if (algPP.tiempoMediaCurvaSRuedaDer > tiempoMediaCurvaSMax)
  {
    algPP.tiempoMediaCurvaSRuedaDer = tiempoMediaCurvaSMax;
    // Rectificar la velocidad de la rueda derecha.
    int signoDeltaVelRuedaDer = sign(algPP.velRuedaDerFinal - velRuedaDerInicial);
    vel = velRuedaDerInicial
        + (signoDeltaVelRuedaDer * algPP.acelRuedaMax * ((double)algPP.tiempoMediaCurvaSRuedaDer / 1000.0f));
    if (signoDeltaVelRuedaDer > 0)
    {
      algPP.velRuedaDerFinal = floorf(10 * vel) / 10;
    }
    else
    {
      algPP.velRuedaDerFinal = ceilf(10 * vel) / 10;
    }
#ifdef DEBUG
    printf("\n");
    printf("tMCSD > tMCSMAX\n");
    printf("vD: %.3f --> %.3f\n", velRuedaDerInicial, algPP.velRuedaDerFinal);
#endif
  }
  // else if(algPP.tiempoMediaCurvaSRuedaDer < 50)
  //  {
  //  tiempoMediaCurvaSRuedaDer = Tmin;
  //  adM = abs(VdF - VdI)/(tiempoMediaCurvaSRuedaDer/1000);
  // }
  if (algPP.tiempoMediaCurvaSRuedaIzq <= 1)
  {
    algPP.velRuedaIzqFinal = velRuedaIzqInicial;
    algPP.tiempoMediaCurvaSRuedaIzq = tiempoMediaCurvaSMax;
#ifdef DEBUG
    printf("\n");
    printf("tMCSI = 0 ms -- > %d ms \n", algPP.tiempoMediaCurvaSRuedaIzq);
    printf("vIF = %f cm/s \n", algPP.velRuedaIzqFinal);
#endif
  }
  else if (algPP.tiempoMediaCurvaSRuedaIzq > tiempoMediaCurvaSMax)
  {
    algPP.tiempoMediaCurvaSRuedaIzq = tiempoMediaCurvaSMax;
    int signoDeltaVelRuedaIzq = sign(algPP.velRuedaIzqFinal - velRuedaIzqInicial);
    vel = velRuedaIzqInicial
        + (signoDeltaVelRuedaIzq * algPP.acelRuedaMax * ((double)algPP.tiempoMediaCurvaSRuedaIzq / 1000.0f));

    if (signoDeltaVelRuedaIzq > 0)
    {
      algPP.velRuedaIzqFinal = floorf(10 * vel) / 10;
    }
    else
    {
      algPP.velRuedaIzqFinal = ceilf(10 * vel) / 10;
    }
#ifdef DEBUG
    printf("\n");
    printf("tMCSI > tMCSMAX\n");
    printf("vI: %.3f --> %.3f\n", velRuedaIzqInicial, algPP.velRuedaIzqFinal);
#endif
  }
  // else if (algPP.tiempoMediaCurvaSRuedaIzq < 50)
  // {
  //algPP.tiempoMediaCurvaSRuedaIz = 50;
  //aiM = abs(ViF - ViI)/(tiempoMediaCurvaSRuedaIzq/1000);
  //}
#ifdef DEBUG
  printf("\n");
  printf("vD: %.3f --> %.3f\n", velRuedaDerInicial, algPP.velRuedaDerFinal);
  printf("tMCSD: %d.\n", algPP.tiempoMediaCurvaSRuedaDer);
  printf("vI: %.3f --> %.3f\n", velRuedaIzqInicial, algPP.velRuedaIzqFinal);
  printf("tMCSI: %d.\n", algPP.tiempoMediaCurvaSRuedaIzq);
#endif
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP);
}

int escribirDPRAMVelocidadFinalTiempoMediaCurvaS(const int & descriptorDPRAM, const algoritmoPurePursuit & algPP)
{

  int flagActualizarVelocidades = 1;
  do
  {
    if (leerDPRAM(descriptorDPRAM, DIR_FLAG_ACTUALIZAR_VELOCIDADES, (char *)&flagActualizarVelocidades, NBYTES))
    {
      return -1;
    }
  } while (flagActualizarVelocidades == 1);
  // Cuando flagActualizarVelocidades sea '0' entonces la PMAC ha procesado las velocidades enviadas en la ejecucion
  // anterior de esta funcion.
  if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_DER_FINAL, (char *)&(algPP.velRuedaDerFinal), NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_IZQ_FINAL, (char *)&(algPP.velRuedaIzqFinal), NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorDPRAM, DIR_TIEMPO_MEDIA_CURVAS_RUEDA_DER, (char *)&(algPP.tiempoMediaCurvaSRuedaDer),
                    NBYTES))
  {
    return -1;
  }
  if (escribirDPRAM(descriptorDPRAM, DIR_TIEMPO_MEDIA_CURVAS_RUEDA_IZQ, (char *)&(algPP.tiempoMediaCurvaSRuedaIzq),
                    NBYTES))
  {
    return -1;
  }
  flagActualizarVelocidades = 1;
  if (escribirDPRAM(descriptorDPRAM, DIR_FLAG_ACTUALIZAR_VELOCIDADES, (char *)&flagActualizarVelocidades, NBYTES) == -1)
  {
    return -1;
  }
  // La funcion finaliza con exito.
  return 0;
}

int pararRobot(const int & descriptorDPRAM, algoritmoPurePursuit & algPP)
{

  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;
  algPP.velRuedaDerFinal = 0;
  algPP.velRuedaIzqFinal = 0;
  algPP.tiempoMediaCurvaSRuedaDer = ceilf(
      1000 * fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / algPP.acelRuedaMax);
  algPP.tiempoMediaCurvaSRuedaIzq = ceilf(
      1000 * fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / algPP.acelRuedaMax);
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP);
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * */

int main(int argc, char **argv)
{
  // NO HAY QUE LIBERAR LA MEMORIA ASOCIADA AL PUNTERO GUARDADO EN LA VARIABLE 'rutaROS'
  //char* rutaROS = getenv("ROS_WORKSPACE");
  //if (rutaROS == NULL)
  //{
  // printf("\n");
  //printf("\x1B[31;1m");
  //printf("La ruta hasta el sistema de ficheros de ROS (personal) no existe \n");
  //printf("\x1B[0m");
  //return -1;
  //}

//#ifdef DEBUG
  //printf("\n");
  //printf("Ruta del sistema de ficheros de ROS (personal): %s \n", rutaROS);
//#endif

  ros::init(argc, argv, "nodoPurePursuit");
  ros::NodeHandle nh;
  std::string parametro;

  double tamanioCeldaM = 0;
  int numeroLocalizacionesOdometria = 0;
  int iSel = 0;

  double xRobotInicialSRG = 0.0f;
  double yRobotInicialSRG = 0.0f;
  double tRobotInicialSRG = 0.0f;

  double xRobotSRGAnt = 0.0f;
  double yRobotSRGAnt = 0.0f;
  double tRobotSRGAnt = 0.0f;

  double xRobotSRG = 0.0f;
  double yRobotSRG = 0.0f;
  double tRobotSRG = 0.0f;

  double xRobotSROAnt = 0.0f;
  double yRobotSROAnt = 0.0f;
  double tRobotSROAnt = 0.0f;

  double xRobotSRO = 0.0f;
  double yRobotSRO = 0.0f;
  double tRobotSRO = 0.0f;

  double x_goal = 9.1;
  double y_goal = 39.4;

  double comandosPDer[2] = {0.0f, 0.0f};
  double comandoVDer = 0.0f;
  int comandosT[2] = {0, 0};

  double deltaX = 0.0f;
  double deltaY = 0.0f;
  double distanciaAlObjetivo = 0.0f;
  double thetaError = 0.0f;

  // ----- ----- ----- ----- -----

  if (!nh.hasParam("/NombreMapa"))
  {
    return -1;
  }

  std::string nombreMapa;
  nh.getParam("/NombreMapa", nombreMapa);

  //if (!nh.hasParam("/Mapa"))
  //{
  //  return -1;
  //}
  //nh.getParam("/Mapa", parametro);
  //nombreMapa = new char[strlen(rutaROS) + parametro.length() + 1]();
  //strcat(nombreMapa, rutaROS);
  //strcat(nombreMapa, parametro.c_str());

  // ----- ----- ----- ----- -----

  if (!nh.hasParam("/TamanioCeldaM"))
  {
    //delete nombreMapa;
    cout << "No se ha encontrado el parametro TamanioCeldaM en el master" << endl;
    return -1;
  }
  nh.getParam("/TamanioCeldaM", tamanioCeldaM);
  //std::istringstream iss(parametro);
  //iss >> tamanioCeldaM;
  //tamanioCeldaM = atof(parametro.c_str()); // Convertir de string a double.

  // ----- ----- ----- ----- -----

  algoritmoPurePursuit algPP;
  if (!nh.hasParam("/RabioBaseM"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/RabioBaseM", parametro);
  algPP.radioBase = 100 * atof(parametro.c_str()); // Convertir de string a double. En cm.

  // ----- ----- ----- ----- -----

  // Radio a partir del que se considera que el robot se desplaza en linea recta.
  algPP.radioUmbralLinRecta = 10.0f * algPP.radioBase;
  algPP.radioConSignoFinal = 0.0f;

  // ----- ----- ----- ----- -----

  if (!nh.hasParam("/VelocidadRuedaMaximaMpS"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/VelocidadRuedaMaximaMpS", parametro);
  algPP.velRuedaMax = 100 * atof(parametro.c_str()); // Convertir de string a double. En cm/s

  // ----- ----- ----- ----- -----

  if (!nh.hasParam("/AceleracionRuedaMaximaMpS2"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/AceleracionRuedaMaximaMpS2", parametro);
  algPP.acelRuedaMax = 100 * atof(parametro.c_str()); //En cm/s^2.

  // ----- ----- ----- ----- -----

  if (!nh.hasParam("/VelocidadAngularBaseMaximaGpS"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/VelocidadAngularBaseMaximaGpS", parametro);
  algPP.velAngBaseMax = atof(parametro.c_str()); //En grad/s.

  // ----- ----- ----- ----- -----

  algPP.velRuedaDerFinal = 0.0f;
  algPP.velRuedaIzqFinal = 0.0f;
  algPP.tiempoMediaCurvaSRuedaDer = 0.0f;
  algPP.tiempoMediaCurvaSRuedaIzq = 0.0f;

  // ----- ----- ----- ----- -----

  if (!nh.hasParam("/TiempoCurvaSMaximoS"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/TiempoCurvaSMaximoS", parametro);
  algPP.tiempoCurvaSMax = static_cast<int>(1000 * atof(parametro.c_str())); //En En ms

  // ----- ----- ----- ----- -----

  // El tiempo de desplazamiento es 50 ms superior al tiempo que puede
  // durar como maximo el cambio de velocidad (curva S) en las ruedas de la base.
  algPP.tiempoDesplazamiento = algPP.tiempoCurvaSMax + 50;  // En ms
  double diametroBase = 2.0f * algPP.radioBase;
  double radioConSignoFinalAnterior = 0.0f;
  double deltaRadioConSigno = 0.0f;
  double lado = 0.01 * algPP.velRuedaMax * (algPP.tiempoDesplazamiento / 1000.0f);
  double xInferiorAreaReposo = x_goal - lado;
  double xSuperiorAreaReposo = x_goal + lado;
  double yInferiorAreaReposo = y_goal - lado;
  double ySuperiorAreaReposo = y_goal + lado;

  int descriptorDPRAM = 0;

#ifndef PRUEBA
  if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1)
  {
    printf("\x1B[31;1m");
    printf("No se puede abrir el dispositivo: %s.\n", DPRAM);
    printf("\x1B[0m");
    //delete nombreMapa;
    return -1;
  }
#endif

  // COMENTADO PERO MODIFICARLO

  /*
   odometria::servicioOdometria servicioOdom;
   ros::ServiceClient clienteServicioOdom = nh.serviceClient<odometria::servicioOdometria>("servicioOdometria");

   actionlib::SimpleActionClient<manfred_arm_msgs::LocalizacionEvolucionDiferencialAction> clienteAccion(
   "servidorAccionLocalizacionDE", true);
   */

  ros::Subscriber suscriptorTopicOdometria = nh.subscribe("topic_odometria_OK", 1, obtenerOdometria);
  nav_msgs::Odometry msj_odometria;
  p_msj_odometria = &msj_odometria;

#ifdef DEBUG
  printf("\n");
  cout << "NombreMapa: " << nombreMapa << std::endl;
  printf("Tamanio celda %f m \n", tamanioCeldaM);
  printf("Radio de la base: %f cm \n", algPP.radioBase);
  printf("Diametro de la base: %f cm \n", diametroBase);
  printf("Radio umbral para considerar trayectoria en linea recta: %f cm \n", algPP.radioUmbralLinRecta);
  printf("Velocidad maxima de las ruedas de la base: %f cm/s \n", algPP.velRuedaMax);
  printf("Aceleracion maxima de las ruedas de la base: %f cm/s^2 \n", algPP.acelRuedaMax);
  printf("Velocidad angular maxima de la base: %f grad/s \n", algPP.velAngBaseMax);
  printf("Tiempo de curva S maximo: %d ms \n", algPP.tiempoCurvaSMax);
  printf("Tiempo de desplazamiento: %d ms \n", algPP.tiempoDesplazamiento);
  printf("Lado: %f.\n", lado);
  printf("xInferiorAreaReposo: %f.\n", xInferiorAreaReposo);
  printf("xSuperiorAreaReposo: %f.\n", xSuperiorAreaReposo);
  printf("yInferiorAreaReposo: %f.\n", yInferiorAreaReposo);
  printf("ySuperiorAreaReposo: %f.\n", ySuperiorAreaReposo);
  printf("Esperando a que el servidor de localizacion global este disponible \n");
#endif

  //clienteAccion.waitForServer();

#ifdef DEBUG
  printf("\n");
  printf("Servidor de localizacion global disponible \n");
#endif

  //manfred_arm_msgs::LocalizacionEvolucionDiferencialGoal objetivo;
  //objetivo.dummyFlagGoal = true;

  //clienteAccion.sendGoal(objetivo);

  bool accionAtendida = false;

  /*
   while (!(accionAtendida = clienteAccion.waitForResult(ros::Duration(300))))
   {
   printf("\n");
   printf("\x1B[31;1m");
   printf("La ubicacion del robot en sist.ref.global no llega en el tiempo de espera definido \n");
   printf("\x1B[0m");
   }

   actionlib::SimpleClientGoalState state = clienteAccion.getState();
   const manfred_arm_msgs::LocalizacionEvolucionDiferencialResult * resultado = clienteAccion.getResult().get();

   */

  //xRobotInicialSRG = resultado->ubicacionRobotSRG.pose.position.x;
  xRobotInicialSRG = 6.85;
  xRobotSRGAnt = xRobotInicialSRG;
  xRobotSRG = xRobotInicialSRG;

  //yRobotInicialSRG = resultado->ubicacionRobotSRG.pose.position.y;
  yRobotInicialSRG = 3.1;
  yRobotSRGAnt = yRobotInicialSRG;
  yRobotSRG = yRobotInicialSRG;

  //tRobotInicialSRG = tf::getYaw(resultado->ubicacionRobotSRG.pose.orientation);
  tRobotInicialSRG = M_PI / 2;
  tRobotSRGAnt = tRobotInicialSRG;
  tRobotSRG = tRobotInicialSRG;

  mr::Image* img = new mr::Image(1, 1);
  img->load(nombreMapa);

#ifdef DEBUG
  printf("\n");
  printf("xRobotInicialSRG: %f m \n", xRobotInicialSRG);
  printf("yRobotInicialSRG: %f m \n", yRobotInicialSRG);
  printf("tRobotInicialSRG: %f grad \n", gradRad * tRobotInicialSRG);
  printf("Anchura del mapa: %f m \n", tamanioCeldaM * img->getWidth());
  printf("Altura  del mapa: %f m \n", tamanioCeldaM * img->getHeight());
#endif

  mr::LabeledGridMap* gridmap = new mr::LabeledGridMap();
  gridmap->loadFromImage(img, tamanioCeldaM * img->getWidth(), tamanioCeldaM, 0, 0);
  mr::FM2PathPlanner2d* fm2pathplanner = new mr::FM2PathPlanner2d(gridmap, true);
  fm2pathplanner->configure(0.2, 2);
  mr::Pose* p_init = new mr::Pose(xRobotInicialSRG, yRobotInicialSRG);
  mr::Pose* p_goal = new mr::Pose(x_goal, y_goal);
  bool path_found_star = fm2pathplanner->computePath(*p_init, *p_goal);
  if (!path_found_star)
  {
    printf("\n");
    printf("\x1B[31;1m");
    printf("Camino no encontrado \n");
    printf("\x1B[0m");
    //delete nombreMapa;
    delete img;
    delete gridmap;
    delete fm2pathplanner;
    delete p_init;
    delete p_goal;
    return -1;
  }
  mr::Path2D path;
  path = fm2pathplanner->getPath();

  std::vector<mr::Vector2D> points_new(path.points.begin(), path.points.end());
#ifdef DEBUG
  printf("\n");
  printf("Trayectoria antigua y nueva\n");

  // Primer y ultima punto de la trayectoria seguida por el punto medio de la base
  double ang_first_segment = atan2(path.points[1].y - path.points[0].y, path.points[1].x - path.points[0].x);
  int last_index = path.points.size() - 1;
  double ang_last_segment = atan2(path.points[last_index].y - path.points[last_index - 1].y,
                                  path.points[last_index].x - path.points[last_index - 1].x);
  points_new[0].x = 0.3 * cos(ang_first_segment + M_PI / 2) + path.points[0].x;
  points_new[0].y = 0.3 * sin(ang_first_segment + M_PI / 2) + path.points[0].y;
  points_new[last_index].x = 0.3 * cos(ang_last_segment + M_PI / 2) + path.points[last_index].x;
  points_new[last_index].y = 0.3 * sin(ang_last_segment + M_PI / 2) + path.points[last_index].y;
  // Resto de puntos seguidos por el punto medio de la base.
  for (unsigned int i = 1; i < path.points.size() - 1; i++)
  {
    printf("( %f m, %f m) \n", path.points[i].x, path.points[i].y);

    double ang_prev = atan2(path.points[i].y - path.points[i - 1].y, path.points[i].x - path.points[i - 1].x);
    double ang_next = atan2(path.points[i + 1].y - path.points[i].y, path.points[i + 1].x - path.points[i].x);
    double difAng = ang_next - ang_prev;
    double ang_final = ang_prev + M_PI / 2 + difAng / 2;
    points_new[i].x = 0.3 * cos(ang_final) + path.points[i].x;
    points_new[i].y = 0.3 * sin(ang_final) + path.points[i].y;
    printf("( %f m, %f m) \n", points_new[i].x, points_new[i].y);
    cout << endl;
    //gridmap->WorldToGrid(path.points[i].x, path.points[i].y, x, y);
  }
#endif

  return 0;

  int iFinal = path.points.size() - 1;

  // tiempoDesplazamiento en ms
  // frec = 1 /tiempoDesplazamientoSeg o 1000/tiempoDesplazamientoMSeg
  /*
   Si hago que la frecuencia de muestreo de la ubicacion del robot, es decir, la frecuencia a la que
   se van a dar acciones de control sea menor que este valor, es equivalente a permitir que el
   robot se desplace un poco más de tiempo con la accion de control anterior.
   Para hacer que la frec sea un poco menor de lo estricamente necesaria hago que el
   tiempo desplazamiento real sea un 25% mayor del tiempo de desplazamiento teorico.
   tiempoDesplazamientoReal = 1'25 * tiempoDesplazamiento
   NOTA: Para calcular el tiempo que dura una curva S de velocidad en las ruedas se usa 'tiempoDesplazamiento',
   no 'tiempoDesplazamientoReal'
   frec = 1000 / (1'25 * tiempoDesplazamientoMSeg) = (1000 * 100)/(125 * t) = (0'8 * 1000)/t = 800/t
   inalmente con floor aseguro que la frecuencia sea un numero entero que tiende a la baja. A la baja
   porque cuanto menor sea la frecuencia mayor es el tiempo de desplazamiento (denominador).
   */
  int frec = floorf(1000 / algPP.tiempoDesplazamiento);
#ifdef DEBUG
  printf("Frecuencia muestreo: %d.\n", frec);
#endif
  ros::Rate r(frec);
  double puntosTrayectSRG[1][2] = {path.points[1].x, path.points[1].y};
  rotacion(descriptorDPRAM, xRobotSRG, yRobotSRG, tRobotSRG, puntosTrayectSRG[0], algPP);

  //std::cout << "Introduce una tecla para continuar: ";
  //std::cin >> teclaParaContinuar;
  //char temp = 0;
  //scanf("%c", &temp);
  //std::cout << std::endl;

#ifndef DEBUG
  clienteAccion.sendGoal(objetivo);
  accionAtendida = false;
  while (!(accionAtendida = clienteAccion.waitForResult(ros::Duration(30))))
  {
    printf("\n");
    printf("\x1B[31;1m");
    printf("La ubicacion del robot en sist.ref.global no llega en el tiempo de espera definido \n");
    printf("\x1B[0m");
  }
  state = clienteAccion.getState();
  resultado = clienteAccion.getResult().get();

  xRobotSRG = resultado->ubicacionRobotSRG.pose.position.x;
  yRobotSRG = resultado->ubicacionRobotSRG.pose.position.y;
  tRobotSRG = tf::getYaw(resultado->ubicacionRobotSRG.pose.orientation);

  if (clienteServicioOdom.call(servicioOdom))
  {
    xRobotSRO = servicioOdom.response.msjOdometria.pose.pose.position.x;
    yRobotSRO = servicioOdom.response.msjOdometria.pose.pose.position.y;
    tRobotSRO = tf::getYaw(servicioOdom.response.msjOdometria.pose.pose.orientation);
  }
  else
  {
    printf("\n");
    printf("\x1B[31;1m");
    printf("Fallo servicio odometria \n");
    printf("\x1B[0m");
    // Parar el robot y salir del programa porque se ha producido un fallo en la odometria.
    // Para conseguir que el robot se pare hago como si el robot hubiera llegado al destino, de ese modo
    // el bucle while principal de la funcion main finaliza y se invoca la funcion 'pararRobot()'.
    xRobotSRG = xInferiorAreaReposo;
    yRobotSRG = yInferiorAreaReposo;
  }

#ifdef DEBUG
  printf("\n");
  printf("xRobotSRG: %f m \n", xRobotSRG);
  printf("yRobotSRG: %f m \n", yRobotSRG);
  printf("tRobotSRG: %f grad \n", gradRad * tRobotSRG);
  printf("xRobotSRO: %f m \n", xRobotSRO);
  printf("yRobotSRO: %f m \n", yRobotSRO);
  printf("tRobotSRO: %f grad \n", gradRad * tRobotSRO);
#endif

  while (xRobotSRG < xInferiorAreaReposo || xRobotSRG > xSuperiorAreaReposo || yRobotSRG < yInferiorAreaReposo
      || yRobotSRG > ySuperiorAreaReposo)
  {
    // deltaX y deltaY en m
    // distanciaAlObjetivo en m ==> pasar a cm.
    // diametroBase en cm.
    //deltaX = puntosTrayectSRG[iSel][0] - xRobotSRG;
    //deltaY = puntosTrayectSRG[iSel][1] - yRobotSRG;
    deltaX = path.points[iSel].x - xRobotSRG;
    deltaY = path.points[iSel].y - yRobotSRG;
    distanciaAlObjetivo = 100 * hypot(deltaX, deltaY);
    // Seleccion del proximo punto objetivo dentro de la trayectoria.
    while (iSel < iFinal && distanciaAlObjetivo < diametroBase)
    {
      iSel += 1;
      printf("while - iSel: %d.\n", iSel);
      //deltaX = puntosTrayectSRG[iSel][0] - xRobotSRG;
      //deltaY = puntosTrayectSRG[iSel][1] - yRobotSRG;
      deltaX = path.points[iSel].x - xRobotSRG;
      deltaY = path.points[iSel].y - yRobotSRG;
      distanciaAlObjetivo = 100 * hypot(deltaX, deltaY);
    }
    thetaError = atan2f(deltaY, deltaX) - tRobotSRG;
    algPP.radioConSignoFinal = distanciaAlObjetivo / (2 * sin(thetaError));
    deltaRadioConSigno = algPP.radioConSignoFinal - radioConSignoFinalAnterior;

#ifdef DEBUG
    printf("\n");
    printf("Ubicacion robot (%.3f m, %.3f m, %.3f grad) \n", xRobotSRG, yRobotSRG, gradRad * tRobotSRG);
    printf("Punto objetivo (%d): (%.3f m, %.3f m) \n", iSel, path.points[iSel].x, path.points[iSel].y);
    printf("Dist: %f cm \n", distanciaAlObjetivo);
    printf("tError: %f grad \n", gradRad * thetaError);
    printf("rSF: %f cm \n", algPP.radioConSignoFinal);
#endif

    /*
     if (fabs(deltaRadioConSigno) < 5)
     {
     printf("|DeltaR| < 5.\n");
     algPP.radioConSignoFinal = radioConSignoFinalAnterior;
     }
     else
     {
     radioConSignoFinalAnterior = algPP.radioConSignoFinal;
     }
     */
    actualizarVelFinalRuedas(descriptorDPRAM, algPP);
#ifdef DEBUG
    printf("\n");
    printf("ZZZZZzzzzz \n");
    printf("ZZZZZzzzzz \n");
    printf("ZZZZZzzzzz \n");
#endif

    r.sleep();

#ifdef DEBUG
    printf("\n");
    printf("Despertarse \n");
#endif

    xRobotSROAnt = xRobotSRO;
    yRobotSROAnt = yRobotSRO;
    tRobotSROAnt = tRobotSRO;

    xRobotSRGAnt = xRobotSRG;
    yRobotSRGAnt = yRobotSRG;
    tRobotSRGAnt = tRobotSRG;

    if (numeroLocalizacionesOdometria == 4)
    {
      numeroLocalizacionesOdometria = 0;
      clienteAccion.sendGoal(objetivo);
      accionAtendida = clienteAccion.waitForResult(ros::Duration(1));
      if (!accionAtendida)
      {
        printf("\n");
        printf("\x1B[31;1m");
        printf("La ubicacion del robot en sist.ref.global no llega en el tiempo de espera definido \n");
        printf("\x1B[0m");
        xRobotSRG = xInferiorAreaReposo;
        yRobotSRG = yInferiorAreaReposo;
      }
      else
      {
        state = clienteAccion.getState();
        resultado = clienteAccion.getResult().get();
        xRobotSRG = resultado->ubicacionRobotSRG.pose.position.x;
        yRobotSRG = resultado->ubicacionRobotSRG.pose.position.y;
        tRobotSRG = tf::getYaw(resultado->ubicacionRobotSRG.pose.orientation);

#ifdef DEBUG
        printf("\n");
        printf("LGDE - xRobotSRG: %f m \n", xRobotSRG);
        printf("LGDE - yRobotSRG: %f m \n", yRobotSRG);
        printf("LCGE - tRobotSRG: %f grad \n", gradRad * tRobotSRG);
#endif

      }
    }
    else
    {
      if (clienteServicioOdom.call(servicioOdom))
      {
        do
        {
          ros::spinOnce();
        } while (!flag_topic_odometria_recibido);
        flag_topic_odometria_recibido = false;


        xRobotSRO = servicioOdom.response.msjOdometria.pose.pose.position.x;
        yRobotSRO = servicioOdom.response.msjOdometria.pose.pose.position.y;
        tRobotSRO = tf::getYaw(servicioOdom.response.msjOdometria.pose.pose.orientation);
        xRobotSRG = xRobotSRGAnt + ((xRobotSRO - xRobotSROAnt) * cos(tRobotInicialSRG))
        - ((yRobotSRO - yRobotSROAnt) * sin(tRobotInicialSRG));
        yRobotSRG = yRobotSRGAnt + ((xRobotSRO - xRobotSROAnt) * sin(tRobotInicialSRG))
        + ((yRobotSRO - yRobotSROAnt) * cos(tRobotInicialSRG));
        tRobotSRG = tRobotSRGAnt + (tRobotSRO - tRobotSROAnt);

#ifdef DEBUG
        printf("\n");
        printf("OD - xRobotSRG: %f m \n", xRobotSRG);
        printf("OD - yRobotSRG: %f m \n", yRobotSRG);
        printf("OD - tRobotSRG: %f grad \n", gradRad * tRobotSRG);
#endif

        numeroLocalizacionesOdometria++;
      }
      else
      {
        printf("\n");
        printf("\x1B[31;1m");
        printf("Fallo servicio odometria \n");
        printf("\x1B[0m");
        // Parar el robot y salir del programa porque se ha producido un fallo en la odometria.
        // Para conseguir que el robot se pare hago como si el robot hubiera llegado al destino, de ese modo
        // el bucle while principal de la funcion main finaliza y se invoca la funcion 'pararRobot()'.
        xRobotSRG = xInferiorAreaReposo;
        yRobotSRG = yInferiorAreaReposo;
      }
    }
  }
  pararRobot(descriptorDPRAM, algPP);
  //}

#endif
  //delete nombreMapa;
  delete img;
  delete gridmap;
  delete fm2pathplanner;
  delete p_init;
  delete p_goal;
  return 0;
}
