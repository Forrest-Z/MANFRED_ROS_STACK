// C/C++
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

// ROS
//#include "actionlib/client/simple_action_client.h"
//#include "actionlib/client/terminal_state.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
//#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialAction.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
//#include "odometria/servicioOdometria.h"
#include "pmac.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

// OpenMRL
#include "datatype/gridmap_includes.h"
#include "datatype/geometry/path2d.h"
#include "algorithms/planning/fm2pathplanner.h"

//nav_msgs::Odometry *p_msj_odometria = NULL;
//bool flag_topic_odometria_recibido = false;
bool flag_mapa_recibido = false;
double tamanio_celda_M = 0;

/*
 void obtenerOdometria(const nav_msgs::Odometry::ConstPtr& rp_msj_odometria_rx)
 {

 *p_msj_odometria = *rp_msj_odometria_rx;
 flag_topic_odometria_recibido = true;
 }*/

//#define PRUEBA
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

int rotacion(const int& descriptorDPRAM, const double& x_mbase_en_mmapa, const double & y_mbase_en_mmapa,
             const double & t_mbase_en_mmapa, const double *puntoTrayectSRG, algoritmoPurePursuit& algPP)
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
  double deltaX = puntoTrayectSRG[0] - x_mbase_en_mmapa;
  double deltaY = puntoTrayectSRG[1] - y_mbase_en_mmapa;
  double orientacionTramo = atan2f(deltaY, deltaX);
// Angulo a rotar por el robot. Entre [-pi, pi].
  double deltaTheta = orientacionTramo - t_mbase_en_mmapa;
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
  double velRuedaRotMax = floorf(10.0f * velAngBaseMax * algPP.radioBase) / 10.0f;
  // Tiempo en pasar de 0 cm/s a (velRuedaRotMax/2) cm/s usando la aceleracion maxima. La funcion
  // ceilf es para asegurar un numero entero (la PMAC necesita que el tiempo que dura un comando PVT
  // sea un numero entero) y que no se exceda la aceleracion. En ms.
  int tiempoMediaCurvaSRot_1 = ceilf(1000.0f * velRuedaRotMax / algPP.acelRuedaMax);
  //double acelRuedaRotMaxReal = velRuedaRotMax/(tiempoMediaCurvaSRot_1/1000.0f);
  // El numero 1000.0f pasa al numerador.
  double acelRuedaRotMaxReal = 1000.0f * velRuedaRotMax / tiempoMediaCurvaSRot_1;
  // Desplazamiento realizado en el tiempo que dura la primera curvaS. En cm.
  double desplazRot_1 = velRuedaRotMax * (tiempoMediaCurvaSRot_1 / 1000.0f);
  // Desplazamiento si solo considero: CurvaS-CurvaS.
  double desplazRot2CurvaS = 2 * desplazRot_1;

#ifdef DEBUG
  cout << "puntoTrayectSRG: (" << puntoTrayectSRG[0] << " m, " << puntoTrayectSRG[1] << " m)" << endl;
  cout << "deltaX: " << deltaX << " m " << endl;
  cout << "deltaY: " << deltaY << " m" << endl;
  cout << "orientacionTramo: " << gradRad * orientacionTramo << " grad" << endl;
  cout << "deltaTheta: " << gradRad * deltaTheta << " grad" << endl;
  cout << "arco: " << arco << " cm (+ rotacion antihoraria, - rotacion horaria)" << endl;
  cout << "velAngBaseMax: " << velAngBaseMax << " rad/s" << endl;
  cout << "velRuedaRotMax: " << velRuedaRotMax << " cm/s" << endl;
  cout << "tiempoMediaCurvaSRot_1: " << tiempoMediaCurvaSRot_1 << " ms " << endl;
  cout << "acelRuedaRotMaxReal: " << acelRuedaRotMaxReal << " cm/s^2" << endl;
  cout << "desplazRot_1: " << desplazRot_1 << " cm" << endl;
  cout << "desplazRot2CurvaS: " << desplazRot2CurvaS << " cm" << endl;
#endif

  if (velRuedaRotMax == 0 || tiempoMediaCurvaSRot_1 == 0)
  {
    cout << "velRuedaRotMax = 0 o tiempoMediaCurvaSRot_1 = 0" << endl;
    return -1;
  }
  if (arcoValorAbsoluto > desplazRot2CurvaS)
  {
    // Tiempo que dura la mitad de la primera curva S. Numero entero en ms.
    comandosT[0] = tiempoMediaCurvaSRot_1;
    // Recorrido con signo. En cm y con decimales.
    comandosPDer[0] = static_cast<float>(signoDeltaTheta * desplazRot_1);
    // Velocidad con 1 decimal. En cm/s.
    comandoVDer = static_cast<float>(signoDeltaTheta * velRuedaRotMax);
    // Recorrido con signo realizado durante el segundo tramo.
    // El segundo tramo se recorre a velocidad constante, que es
    // la velocidad final del tramo 1.
    double desplazRot_2 = arcoValorAbsoluto - desplazRot2CurvaS;

#ifdef DEBUG
    cout << "desplazRot_2: " << desplazRot_2 << " cm" << endl;
#endif

    // round porque no tengo que satisfacer ninguna restriccion, solo que el tiempo sea
    // numero entero. La PMAC necesita numeros enteros para el tiempo. En ms.
    // int tiempoMediaCurvaSRot_2 = round(1000 * desplazRot_2/(2*velRuedaRotMax));
    // 1000/2 = 500
    // round(X) equivale a floorf(X + 0'5)
    int tiempoMediaCurvaSRot_2 = static_cast<int>(round(500.0f * (desplazRot_2 / velRuedaRotMax)));
    // Recalcular el valor de desplazRot_2, ya que tiempoMediaCurvaSRot_2 ha sido redondeado.
    // desplazRot_2 = 2 * velRuedaRotMax*(tiempoMediaCurvaSRot_2/1000.0f);
    // 2/1000  = 2*10^-3 = 2.0e-3
    desplazRot_2 = 2.0e-3 * velRuedaRotMax * tiempoMediaCurvaSRot_2;
    comandosT[1] = tiempoMediaCurvaSRot_2;
    comandosPDer[1] = static_cast<float>(signoDeltaTheta * desplazRot_2);

#ifdef DEBUG
    cout << "tiempoMediaCurvaSRot_2: " << tiempoMediaCurvaSRot_2 << " ms" << endl;
    cout << "desplazRot_2 corregido: " << desplazRot_2 << " m" << endl;
#endif

  }
  else if (arcoValorAbsoluto < desplazRot2CurvaS)
  {
    // Uso acelRuedaRotMaxReal en lugar de acelRuedaMax, porque la primera acel (acelRuedaRotMaxReal) es la
    // que se obtuvo al usar velRuedaRotMax y tiempoMediaCurvaSRot_1.
    // Al usar el mismo valor de acel junto con un arco que es menor que
    // desplazRot2CurvaS me aseguro que la vel hallada es menor que la
    // velRuedaRotMax.
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
    comandoVDer = static_cast<float>(signoDeltaTheta * velRuedaRotMax);
  }
  double deltaThetaReal = (comandosPDer[0] + comandosPDer[1] + comandosPDer[0]) / algPP.radioBase;

#ifdef DEBUG
  cout << endl;
  cout << "deltaThetaReal: " << gradRad * deltaThetaReal << " grados " << endl;
  cout << endl;
  cout << "PVT(" << comandosT[0] << ")" << endl;
  cout << "X(" << comandosPDer[0] << "):" << comandoVDer << "Y(" << -comandosPDer[0] << "):" << -comandoVDer << endl;
  if (comandosT[1])
  {
    cout << "PVT(" << comandosT[1] << ")" << endl;
    cout << "X(" << comandosPDer[1] << "):" << comandoVDer << "Y(" << -comandosPDer[1] << "):" << -comandoVDer << endl;
  }
  cout << "PVT(" << comandosT[0] << ")" << endl;
  cout << "X(" << comandosPDer[0] << "):" << 0.0f << "Y(" << -comandosPDer[0] << "):" << 0.0f << endl;
#endif

  int error = 0;
  char datosParaPC[NUM_MAX_CARAC_BUFFER_DPRAM_PC];
  memset(datosParaPC, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC * sizeof(char));

#ifndef PRUEBA
  if (escribirRotacionBaseDPRAM(descriptorDPRAM, comandosPDer, &comandoVDer, comandosT, datosParaPC, &error))
  {
    std::cout << "NO SE HA PODIDO ROTAR" << std::endl;
  }
  if (error != 0)
  {
    std::cout << "ERROR INDICADO POR LA PMAC: " << error << std::endl;
  }
//cout <<"datosParaPC: %s.\n", datosParaPC);
  int bitVelocidadCeroMotor1 = -1;
// EL bit de velocidad 0 vale 1 porque el robot esta parado. En el momento que
// se mueve se pone a 1. Cuando el robot se para se pone a 1.
  do
  {
    if (leerEstadoMotor(descriptorDPRAM, 1, &bitVelocidadCeroMotor1))
    {
      return -1;
    }
  }while (bitVelocidadCeroMotor1 != 0);
  do
  {
    if (leerEstadoMotor(descriptorDPRAM, 1, &bitVelocidadCeroMotor1))
    {
      return -1;
    }
  }while (bitVelocidadCeroMotor1 != 1);
#endif

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
    cout << endl;
    cout << "\x1B[31;1m";
    cout << "Radio de giro menor que el radio de la base" << endl;
    cout << "\x1B[0m";
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

#ifdef DEBUG
      cout << endl;
      std::cout << "+++++ Mov_Rect + Giro_AntiH. vel: " << vel << " +++++" << std::endl;
#endif

      // Velocidad con un decimal.
      algPP.velRuedaDerFinal = floorf(10 * vel) / 10;
      algPP.velRuedaIzqFinal = floorf(10 * ((1 - cocienteRadios) / (1 + cocienteRadios)) * algPP.velRuedaDerFinal) / 10;
    }
    // Giro horario (-).
    else
    {

#ifdef DEBUG
      cout << endl;
      std::cout << "+++++ Mov_Rect + Giro_H. vel: " << vel << " +++++" << std::endl;
#endif

      algPP.velRuedaIzqFinal = floorf(10 * vel) / 10;
      algPP.velRuedaDerFinal = floorf(10 * ((1 + cocienteRadios) / (1 - cocienteRadios)) * algPP.velRuedaIzqFinal) / 10;
    }
  }
// Movimiento rectilineo: radioConSignoFinalValorAbs >= algPP.radioUmbralLinRecta
  else
  {

#ifdef DEBUG
    cout << endl;
    std::cout << "___Mov_Rect___" << std::endl;
#endif

    algPP.velRuedaDerFinal = algPP.velRuedaMax;
    algPP.velRuedaIzqFinal = algPP.velRuedaMax;
  }
  algPP.tiempoMediaCurvaSRuedaDer = ceilf(
      1000 * fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / algPP.acelRuedaMax);
  algPP.tiempoMediaCurvaSRuedaIzq = ceilf(
      1000 * fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / algPP.acelRuedaMax);

#ifdef DEBUG
  cout << endl;
  cout << "vD: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tMCSD: " << algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
  cout << "vI: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
  cout << "tMCSI: " << algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;
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
    // NOTA: Si en un perfil de velocidad de tipo curva S, la velocidad inicial y final
    // son iguales, la curva S se convierte en un tramo recto.
    algPP.velRuedaDerFinal = velRuedaDerInicial;
    algPP.tiempoMediaCurvaSRuedaDer = tiempoMediaCurvaSMax;

#ifdef DEBUG
    cout << endl;
    cout << "tMCSD: 0 ms ==> " << algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
    cout << "vDF: " << algPP.velRuedaDerFinal << " cm/s" << endl;
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
    cout << endl;
    cout << "tMCSD > tMCSMAX" << endl;
    cout << "vD: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
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
    cout << endl;
    cout << "tMCSI = 0 ms ==> " << algPP.tiempoMediaCurvaSRuedaIzq << " ms " << endl;
    cout << "vIF = " << algPP.velRuedaIzqFinal << " cm/s " << endl;
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
    cout << endl;
    cout << "tMCSI > tMCSMAX" << endl;
    cout << "vI: " << velRuedaIzqInicial << " ms ==> " << algPP.velRuedaIzqFinal << " ms" << endl;
#endif

  }
  // else if (algPP.tiempoMediaCurvaSRuedaIzq < 50)
  // {
  //algPP.tiempoMediaCurvaSRuedaIz = 50;
  //aiM = abs(ViF - ViI)/(tiempoMediaCurvaSRuedaIzq/1000);
  //}
#ifdef DEBUG
  cout << endl;
  cout << "vD: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tMCSD: " << algPP.tiempoMediaCurvaSRuedaDer << " ms" << endl;
  cout << "vI: " << velRuedaIzqInicial << " ms ==> " << algPP.velRuedaIzqFinal << " ms" << endl;
  cout << "tMCSI: " << algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;
#endif

#ifndef PRUEBA
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP);
#else
  return 0;
#endif

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

void obtenerMapa(const nav_msgs::OccupancyGrid::ConstPtr& rp_msj_rejilla_ocupacion_rx)
{

  unsigned int num_filas = static_cast<unsigned int>(rp_msj_rejilla_ocupacion_rx->info.height);
  unsigned int num_columnas = static_cast<unsigned int>(rp_msj_rejilla_ocupacion_rx->info.width);
  unsigned int inicio = 0;

#ifdef DEBUG
  cout << "Filas: " << num_filas << endl;
  cout << "Columnas: " << num_columnas << endl;
#endif

  tamanio_celda_M = rp_msj_rejilla_ocupacion_rx->info.resolution;
  flag_mapa_recibido = true;

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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "nodoPurePursuit");
  ros::NodeHandle nh;
  tf::TransformListener tfListener;
  tf::StampedTransform stampedTf;
  ros::Subscriber suscriptorMapa = nh.subscribe("map", 1, obtenerMapa);

  std::string parametro;
  //int numeroLocalizacionesOdometria = 0;
  int iSel = 0;

  double x_mbase_en_mmapa_ini = 0.0f;
  double y_mbase_en_mmapa_ini = 0.0f;
  double z_mbase_en_mmapa_ini = 0.0f;
  double t_mbase_en_mmapa_ini = 0.0f;

  double x_mbase_en_mmapa_ant = 0.0f;
  double y_mbase_en_mmapa_ant = 0.0f;
  double z_mbase_en_mmapa_ant = 0.0f;
  double t_mbase_en_mmapa_ant = 0.0f;

  double x_mbase_en_mmapa = 0.0f;
  double y_mbase_en_mmapa = 0.0f;
  double z_mbase_en_mmapa = 0.0f;
  double t_mbase_en_mmapa = 0.0f;

  //double xRobotSROAnt = 0.0f;
  //double yRobotSROAnt = 0.0f;
  //double tRobotSROAnt = 0.0f;

  //double xRobotSRO = 0.0f;
  //double yRobotSRO = 0.0f;
  //double tRobotSRO = 0.0f;

  double x_goal = 6.05;
  double y_goal = 20.45;

  double comandosPDer[2] = {0.0f, 0.0f};
  double comandoVDer = 0.0f;
  int comandosT[2] = {0, 0};

  double deltaX = 0.0f;
  double deltaY = 0.0f;
  double distanciaAlObjetivo = 0.0f;
  double thetaError = 0.0f;

  algoritmoPurePursuit algPP;

  // Adquisicion de parametros del servidor master
  if (!nh.hasParam("/NombreMapa"))
  {
    return -1;
  }
  std::string nombreMapa;
  nh.getParam("/NombreMapa", nombreMapa);

  // ----- ----- ----- ----- -----

  /*
   if (!nh.hasParam("/TamanioCeldaM"))
   {
   //delete nombreMapa;
   cout << "No se ha encontrado el parametro TamanioCeldaM en el master" << endl;
   return -1;
   }
   nh.getParam("/TamanioCeldaM", tamanio_celda_M);
   */

  // Recibir el mapa y obtener el tamanio de la celda en M.
  do
  {
    ros::spinOnce();
  } while (!flag_mapa_recibido);

  // ----- ----- ----- ----- -----

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
    cout <<"\x1B[31;1m";
    cout <<"No se puede abrir el dispositivo: " << DPRAM << endl;
    cout <<"\x1B[0m";
    //delete nombreMapa;
    return -1;
  }
#endif

#ifdef DEBUG
  cout << endl;
  cout << "NombreMapa: " << nombreMapa << endl;
  cout << "Tamanio celda " << tamanio_celda_M << " cm" << endl;
  cout << "Radio de la base: " << algPP.radioBase << " cm" << endl;
  cout << "Diametro de la base: " << diametroBase << " cm" << endl;
  cout << "Radio umbral para considerar trayectoria en linea recta: " << algPP.radioUmbralLinRecta << " cm" << endl;
  cout << "Velocidad maxima de las ruedas de la base: " << algPP.velRuedaMax << " cm/s" << endl;
  cout << "Aceleracion maxima de las ruedas de la base: " << algPP.acelRuedaMax << " cm/s^2" << endl;
  cout << "Velocidad angular maxima de la base: " << algPP.velAngBaseMax << " grad/s" << endl;
  cout << "Tiempo de curva S maximo: " << algPP.tiempoCurvaSMax << " ms" << endl;
  cout << "Tiempo de desplazamiento: " << algPP.tiempoDesplazamiento << " ms" << endl;
  cout << "Lado: " << lado << " m" << endl;
  cout << "xInferiorAreaReposo: " << xInferiorAreaReposo << " m" << endl;
  cout << "xSuperiorAreaReposo: " << xSuperiorAreaReposo << " m" << endl;
  cout << "yInferiorAreaReposo: " << yInferiorAreaReposo << " m" << endl;
  cout << "ySuperiorAreaReposo: " << ySuperiorAreaReposo << " m" << endl;
#endif

  ros::Time marca_tiempo;

  mr::Pose* p_init = NULL;
  mr::Pose* p_goal = NULL;

  bool transformacion_escuchada = false;
  while (!transformacion_escuchada)
  {
    marca_tiempo = ros::Time::now();
    try
    {
      tfListener.waitForTransform("/map", "/link_base_desp", marca_tiempo, ros::Duration(1.0));
      tfListener.lookupTransform("/map", "/link_base_desp", marca_tiempo, stampedTf);
      transformacion_escuchada = true;
      p_init = new mr::Pose(stampedTf.getOrigin().x(), stampedTf.getOrigin().y());
      p_goal = new mr::Pose(x_goal, y_goal);

#ifdef DEUG
      cout << "p_init->x: " << p_init->x << " m " << endl;
      cout << "p_init->y: " << p_init->y << " m " << endl;
      cout << "p_goal->x: " << p_goal->x << " m " << endl;
      cout << "p_goal->y: " << p_goal->y << " m " << endl;
#endif

      // PARA EJEMPLO
      //x_mbase_en_mmapa_ini = 6.85;
      //y_mbase_en_mmapa_ini = 3.1;
      //z_mbase_en_mmapa_ini = 0.075;
      //t_mbase_en_mmapa_ini = M_PI / 2;

    }
    catch (tf::TransformException& ex)
    {
      cout << "\x1B[31;1m";
      cout << ex.what() << endl;
      cout << "\x1B[0m";
    }
    sleep(1);
  }

  float dist_separacion = 0.0f;
  transformacion_escuchada = false;

  while (!transformacion_escuchada)
  {
    marca_tiempo = ros::Time::now();
    try
    {
      tfListener.waitForTransform("/link_base_desp", "/link_base", marca_tiempo, ros::Duration(1.0));
      tfListener.lookupTransform("/link_base_desp", "/link_base", marca_tiempo, stampedTf);
      transformacion_escuchada = true;
      dist_separacion = stampedTf.getOrigin().y();

#ifdef DEBUG
      cout << "dist_separacion: " << dist_separacion << endl;
#endif

    }
    catch (tf::TransformException& ex)
    {
      cout << "\x1B[31;1m";
      cout << ex.what() << endl;
      cout << "\x1B[0m";
    }
    sleep(1);
  }

  mr::Image* img = new mr::Image(1, 1);
  img->load(nombreMapa);

#ifdef DEBUG
  cout << endl;
  cout << "Anchura del mapa: " << tamanio_celda_M * img->getWidth() << " m" << endl;
  cout << "Altura  del mapa: " << tamanio_celda_M * img->getHeight() << " m" << endl;
#endif

  mr::LabeledGridMap* gridmap = new mr::LabeledGridMap();
  gridmap->loadFromImage(img, tamanio_celda_M * img->getWidth(), tamanio_celda_M, 0, 0);

  mr::FM2PathPlanner2d* fm2pathplanner = new mr::FM2PathPlanner2d(gridmap, true);
  fm2pathplanner->configure(0.2, 2);

  bool path_found_star = fm2pathplanner->computePath(*p_init, *p_goal);
  if (!path_found_star)
  {
    cout << endl;
    cout << "\x1B[31;1m";
    cout << "Camino no encontrado" << endl;
    cout << "\x1B[0m";
    delete img;
    delete gridmap;
    delete fm2pathplanner;
    delete p_init;
    delete p_goal;
    return -1;
  }

  mr::Path2D path;
  path = fm2pathplanner->getPath();
  std::vector<mr::Vector2D> puntos_tray_ok(path.points.begin(), path.points.end());

#ifdef DEBUG
  cout << endl;
  cout << "Trayectoria link_base_desp y link_base" << endl;
#endif

  // Primer y ultima punto de la trayectoria seguida por el punto medio de la base
  double ang_primer_tramo = atan2(path.points[1].y - path.points[0].y, path.points[1].x - path.points[0].x);
  int indice_final = path.points.size() - 1;
  double ang_ultimo_tramo = atan2(path.points[indice_final].y - path.points[indice_final - 1].y,
                                  path.points[indice_final].x - path.points[indice_final - 1].x);
  // +M_PI/2 porque la trayectoria que debe seguir el punto medio de la base (link_base) es paralela a la trayectoria
  // calculada por FM, situada a su izquierda, a una distancia 'dist_separacion'
  puntos_tray_ok[0].x = path.points[0].x + (dist_separacion * cos(ang_primer_tramo + M_PI / 2));
  puntos_tray_ok[0].y = path.points[0].y + (dist_separacion * sin(ang_primer_tramo + M_PI / 2));
  puntos_tray_ok[indice_final].x = path.points[indice_final].x + (dist_separacion * cos(ang_ultimo_tramo + M_PI / 2));
  puntos_tray_ok[indice_final].y = path.points[indice_final].y + (dist_separacion * sin(ang_ultimo_tramo + M_PI / 2));

#ifdef DEBUG
  cout << "Tray link_base_desp: (" << path.points[0].x << ", " << path.points[0].y << ") ==> Tray link_base: ("
      << puntos_tray_ok[0].x << ", " << puntos_tray_ok[0].y << ")" << endl;
#endif

  double angulo_previo = 0;
  double angulo_siguiente = 0;
  double difAng = 0;
  double ang_final = 0;
  // Resto de puntos seguidos por el punto medio de la base.
  for (unsigned int i = 1; i < path.points.size() - 1; i++)
  {
    angulo_previo = atan2(path.points[i].y - path.points[i - 1].y, path.points[i].x - path.points[i - 1].x);
    angulo_siguiente = atan2(path.points[i + 1].y - path.points[i].y, path.points[i + 1].x - path.points[i].x);
    difAng = angulo_siguiente - angulo_previo;
    ang_final = angulo_previo + (M_PI / 2) + (difAng / 2);
    puntos_tray_ok[i].x = path.points[i].x + (dist_separacion * cos(ang_final));
    puntos_tray_ok[i].y = path.points[i].y + (dist_separacion * sin(ang_final));

#ifdef DEBUG
    cout << "Tray link_base_desp: (" << path.points[i].x << ", " << path.points[i].y << ") ==> Tray link_base: ("
        << puntos_tray_ok[i].x << ", " << puntos_tray_ok[i].y << ")" << endl;
#endif

    //gridmap->WorldToGrid(path.points[i].x, path.points[i].y, x, y);
  }

#ifdef DEBUG
  cout << "Tray link_base_desp: (" << path.points.back().x << ", " << path.points.back().y << ") ==> Tray link_base: ("
      << puntos_tray_ok.back().x << ", " << puntos_tray_ok.back().y << ")" << endl;
#endif

  nav_msgs::Path path_link_base_dep;
  path_link_base_dep.header.frame_id = "/map";
  path_link_base_dep.header.stamp = ros::Time::now();
  path_link_base_dep.poses.resize(path.points.size());

  nav_msgs::Path path_link_base;
  path_link_base.header.frame_id = "/map";
  path_link_base.header.stamp = path_link_base_dep.header.stamp;
  path_link_base.poses.resize(path.points.size());

  for (int i = 0; i < path.points.size(); i++)
  {
    path_link_base_dep.poses[i].header.frame_id = "/map";
    path_link_base_dep.poses[i].header.stamp = path_link_base_dep.header.stamp;
    path_link_base_dep.poses[i].pose.position.x = path.points[i].x;
    path_link_base_dep.poses[i].pose.position.y = path.points[i].y;
    path_link_base_dep.poses[i].pose.position.z = z_mbase_en_mmapa;

    path_link_base.poses[i].header.frame_id = "/map";
    path_link_base.poses[i].header.stamp = path_link_base.header.stamp;
    path_link_base.poses[i].pose.position.x = puntos_tray_ok[i].x;
    path_link_base.poses[i].pose.position.y = puntos_tray_ok[i].y;
    path_link_base.poses[i].pose.position.z = z_mbase_en_mmapa;
  }

  ros::Publisher publicadorTopicPathLBD = nh.advertise<nav_msgs::Path>("topic_path_link_base_desp", 1, true);
  publicadorTopicPathLBD.publish(path_link_base_dep);

  ros::Publisher publicadorTopicPathLB = nh.advertise<nav_msgs::Path>("topic_path_link_base", 1, true);
  publicadorTopicPathLB.publish(path_link_base);

  x_mbase_en_mmapa_ini = puntos_tray_ok[0].x;
  x_mbase_en_mmapa_ant = x_mbase_en_mmapa_ini;
  x_mbase_en_mmapa = x_mbase_en_mmapa_ini;

  y_mbase_en_mmapa_ini = puntos_tray_ok[0].y;
  y_mbase_en_mmapa_ant = y_mbase_en_mmapa_ini;
  y_mbase_en_mmapa = y_mbase_en_mmapa_ini;

  z_mbase_en_mmapa_ini = stampedTf.getOrigin().z();
  z_mbase_en_mmapa_ant = z_mbase_en_mmapa_ini;
  z_mbase_en_mmapa = z_mbase_en_mmapa_ini;

  t_mbase_en_mmapa_ini = tf::getYaw(stampedTf.getRotation());
  t_mbase_en_mmapa_ant = t_mbase_en_mmapa_ini;
  t_mbase_en_mmapa = t_mbase_en_mmapa_ini;

#ifdef DEBUG
  cout << "x_mbase_en_mmapa " << x_mbase_en_mmapa << " m " << endl;
  cout << "y_mbase_en_mmapa " << y_mbase_en_mmapa << " m" << endl;
  cout << "z_mbase_en_mmapa " << z_mbase_en_mmapa << " m " << endl;
  cout << "t_mbase_en_mmapa " << gradRad * t_mbase_en_mmapa << " grad " << endl;
#endif

  int iFinal = puntos_tray_ok.size() - 1;
  /*
   tiempoDesplazamiento en ms
   frec = 1 /tiempoDesplazamientoSeg o 1000/tiempoDesplazamientoMSeg
   Si hago que la frecuencia de muestreo de la ubicacion del robot, es decir, la frecuencia a la que
   se van a dar acciones de control sea menor que este valor, es equivalente a permitir que el
   robot se desplace un poco más de tiempo con la accion de control anterior.
   Para hacer que la frec sea un poco menor de lo estricamente necesaria uso 'floorf'.
   */
  int frec = floorf(1000 / algPP.tiempoDesplazamiento);

#ifdef DEBUG

  cout << "Frecuencia muestreo: " << frec << " Hz" << endl;

#endif

  ros::Rate r(frec);
  double puntosTrayectSRG[1][2] = {puntos_tray_ok[1].x, puntos_tray_ok[1].y};
  rotacion(descriptorDPRAM, x_mbase_en_mmapa, y_mbase_en_mmapa, t_mbase_en_mmapa, puntosTrayectSRG[0], algPP);

  //std::cout << "Introduce una tecla para continuar: ";
  //std::cin >> teclaParaContinuar;
  //char temp = 0;
  //scanf("%c", &temp);
  //std::cout << std::endl;

#ifdef DEBUG
  cout << endl;
  cout << "x_mbase_en_mmapa: " << x_mbase_en_mmapa << " m" << endl;
  cout << "y_mbase_en_mmapa: " << y_mbase_en_mmapa << " m" << endl;
  cout << "t_mbase_en_mmapa: " << gradRad * t_mbase_en_mmapa << " grad" << endl;
#endif

  while (x_mbase_en_mmapa < xInferiorAreaReposo || x_mbase_en_mmapa > xSuperiorAreaReposo
      || y_mbase_en_mmapa < yInferiorAreaReposo || y_mbase_en_mmapa > ySuperiorAreaReposo)
  {
    // deltaX y deltaY en m
    // distanciaAlObjetivo en m ==> pasar a cm.
    // diametroBase en cm.
#ifdef DEBUG
    cout << endl;
    cout << "===== ===== ===== ===== =====" << endl;
    cout << "iSel: " << iSel << endl;
#endif

    deltaX = puntos_tray_ok[iSel].x - x_mbase_en_mmapa;
    deltaY = puntos_tray_ok[iSel].y - y_mbase_en_mmapa;
    distanciaAlObjetivo = 100 * hypot(deltaX, deltaY);
    // Seleccion del proximo punto objetivo dentro de la trayectoria.
    while (iSel < iFinal && distanciaAlObjetivo < diametroBase)
    {
      iSel += 1;

#ifdef DEBUG
      cout << "iSel: " << iSel << endl;
#endif

      deltaX = puntos_tray_ok[iSel].x - x_mbase_en_mmapa;
      deltaY = puntos_tray_ok[iSel].y - y_mbase_en_mmapa;
      distanciaAlObjetivo = 100 * hypot(deltaX, deltaY);
    }
    thetaError = atan2f(deltaY, deltaX) - t_mbase_en_mmapa;
    algPP.radioConSignoFinal = distanciaAlObjetivo / (2 * sin(thetaError));
    deltaRadioConSigno = algPP.radioConSignoFinal - radioConSignoFinalAnterior;

#ifdef DEBUG
    cout << endl;
    cout << "x_mbase_en_mmapa: " << x_mbase_en_mmapa << " m " << endl;
    cout << "y_mbase_en_mmapa: " << y_mbase_en_mmapa << " m " << endl;
    cout << "t_mbase_en_mmapa: " << gradRad * t_mbase_en_mmapa << " grad " << endl;
    cout << "x_objetivo [ " << iSel << " ]: " << puntos_tray_ok[iSel].x << " m " << endl;
    cout << "y_objetivo [ " << iSel << " ]: " << puntos_tray_ok[iSel].y << " m " << endl;
    cout << "Dist: " << distanciaAlObjetivo << " cm" << endl;
    cout << "tError: " << gradRad * thetaError << " grad" << endl;
    cout << "rSF: " << algPP.radioConSignoFinal << " cm" << endl;
#endif

    /*
     if (fabs(deltaRadioConSigno) < 5)
     {
     cout <<"|DeltaR| < 5.\n");
     algPP.radioConSignoFinal = radioConSignoFinalAnterior;
     }
     else
     {
     radioConSignoFinalAnterior = algPP.radioConSignoFinal;
     }
     */
    actualizarVelFinalRuedas(descriptorDPRAM, algPP);

#ifdef DEBUG
    cout << endl;
    cout << "ZZZZZzzzzz" << endl;
    cout << "ZZZZZzzzzz" << endl;
    cout << "ZZZZZzzzzz" << endl;
#endif

    r.sleep();

#ifdef DEBUG
    cout << endl;
    cout << "Despertarse" << endl;
#endif

    x_mbase_en_mmapa_ant = x_mbase_en_mmapa;
    y_mbase_en_mmapa_ant = y_mbase_en_mmapa;
    t_mbase_en_mmapa_ant = t_mbase_en_mmapa;

    marca_tiempo = ros::Time::now();

    try
    {
      if (!tfListener.waitForTransform("/map", "/link_base", marca_tiempo, ros::Duration(1.0)))
      {
        throw tf::TransformException("La transformacion no llego a tiempo");
      }
      else
      {
        tfListener.lookupTransform("/map", "/link_base", marca_tiempo, stampedTf);
        x_mbase_en_mmapa = stampedTf.getOrigin().x();
        y_mbase_en_mmapa = stampedTf.getOrigin().y();
        z_mbase_en_mmapa = stampedTf.getOrigin().z();
        t_mbase_en_mmapa = tf::getYaw(stampedTf.getRotation());

#ifdef DEBUG
        cout << endl;
        cout << "x_mbase_en_mmapa: " << x_mbase_en_mmapa << " m " << endl;
        cout << "y_mbase_en_mmapa: " << y_mbase_en_mmapa << " m " << endl;
        cout << "t_mbase_en_mmapa: " << gradRad * t_mbase_en_mmapa << " grad " << endl;
#endif

      }
    }
    catch (tf::TransformException& ex)
    {
      cout << "\x1B[31;1m";
      cout << ex.what() << endl;
      cout << "\x1B[0m";
      // Parar el robot y salir del programa porque se ha producido un fallo en la odometria.
      // Para conseguir que el robot se pare hago como si el robot hubiera llegado al destino, de ese modo
      // el bucle while principal de la funcion main finaliza y se invoca la funcion 'pararRobot()'.
      x_mbase_en_mmapa = xInferiorAreaReposo;
      y_mbase_en_mmapa = yInferiorAreaReposo;
    }
  }
  pararRobot(descriptorDPRAM, algPP);
  //}

//delete nombreMapa;
  delete img;
  delete gridmap;
  delete fm2pathplanner;
  delete p_init;
  delete p_goal;
  return 0;
}
