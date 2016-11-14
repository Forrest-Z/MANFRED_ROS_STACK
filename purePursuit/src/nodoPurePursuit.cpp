// C/C++
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

// ROS
//#include "actionlib/client/simple_action_client.h"
//#include "actionlib/client/terminal_state.h"
//#include "geometry_msgs/Pose2D.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialAction.h"
#include "nav_msgs/OccupancyGrid.h"
//#include "nav_msgs/Odometry.h"
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

  double radioRueda;

  // Radio de la base, en cm.
  double radioBase;

  // Radio de curvatura, en cm, a partir del cual se considera que el robot se desplaza en linea recta.
  double radioUmbralLinRecta;

  // Radio de curvatura, en cm, con el que gira la base.
  double radioConSignoFinal;

  // Tiempo que puede durar como mucho una curva S, en cualquiera de la dos ruedas. En ms.
  int tiempoMaxCurvaS;

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

int escribirDPRAMVelocidadFinalTiempoMediaCurvaS(const int & descriptorDPRAM, const algoritmoPurePursuit & algPP,
                                                 const float& cuentas_cm_recorrido);

int sign(double numero)
{
  // numero < 0 --> return -1
  // numero = 0 --> numero > 0 es false, false equivale a 0, por tanto return 0.
  // return > 0 --> numero > 0
  return numero < 0 ? -1 : numero > 0; // # -1, 0, +1.
}

int actualizarVelFinalRuedas(const int & descriptorDPRAM, algoritmoPurePursuit & algPP,
                             const float& cuentas_cm_recorrido)
{

  double radioConSignoFinalValorAbs = fabs(algPP.radioConSignoFinal);
  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;
  double vel = 0.0f;

  int flagActualizarVelocidades = 0;

  // En cts / ms con 1 solo decimal, como se puede observar en el programa Windows de la PMAC (PEWIN32)
  float VbF1_cts_ms = floorf(10.0f * ((algPP.radioBase * radGrad * 10 * cuentas_cm_recorrido) / 1000.0f)) / 10.0f;
  // 4.297 cm/s ==> aprox 10 grad/s
  // Quiero que tenga solo tres decimales. Floorf para que no exceda la velocidad angular maxima.
  float VbF1 = floorf(1000.0f * (1000.0f * VbF1_cts_ms / cuentas_cm_recorrido)) / 1000.0f; // En cm/s
  float VbF2 = 5; // cm/s
  float VbF3 = 0;
  // cm/s
  float m = (VbF2 - VbF1) / (algPP.radioUmbralLinRecta - algPP.radioBase);
  float Acel_max = algPP.acelRuedaMax;

  if (radioConSignoFinalValorAbs >= algPP.radioBase && radioConSignoFinalValorAbs < algPP.radioUmbralLinRecta)
  {

    cout << "[ | rSF | >= rb && | rSF | < rULR ]" << endl;

    VbF3 = (m * (radioConSignoFinalValorAbs - algPP.radioBase)) + VbF1;
    algPP.velRuedaDerFinal = (1 + (algPP.radioBase / algPP.radioConSignoFinal)) * VbF3;
    algPP.velRuedaIzqFinal = (1 - (algPP.radioBase / algPP.radioConSignoFinal)) * VbF3;

    if (radioConSignoFinalValorAbs == algPP.radioBase)
    {
      Acel_max = 2.5f;
    }
  }
  else // radioConSignoFinalValorAbs >= algPP.radioUmbralLinRecta
  {
    cout << "[ | rSF | >= rULR ]" << endl;
    algPP.velRuedaDerFinal = VbF2;
    algPP.velRuedaIzqFinal = VbF2;
  }

  algPP.tiempoMediaCurvaSRuedaDer = ceilf(1000 * (fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / Acel_max));
  algPP.tiempoMediaCurvaSRuedaIzq = ceilf(1000 * (fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / Acel_max));

#ifdef DEBUG

  cout << endl;
  cout << "VbF1: " << VbF1_cts_ms << " cts/ms" << endl;
  cout << "VbF1: " << VbF1 << " cm/s" << endl;
  cout << "VbF2: " << VbF2 << " cm/s" << endl;
  cout << "m: " << m << " 1/s" << endl;
  cout << "VbF3: " << VbF3 << " cm/s" << endl;
  cout << "Vd: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tCSd: " << 2.0f * algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
  cout << "Vi: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
  cout << "tCSi: " << 2.0f * algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;
  cout << "Acel_max: " << Acel_max << " cm/s^2" << endl;
  cout << "adF: "
      << (algPP.velRuedaDerFinal - velRuedaDerInicial) / (static_cast<float>(algPP.tiempoMediaCurvaSRuedaDer) / 1000.0f)
      << " cm/s^2" << endl;
  cout << "aiF: "
      << (algPP.velRuedaIzqFinal - velRuedaIzqInicial) / (static_cast<float>(algPP.tiempoMediaCurvaSRuedaIzq) / 1000.0f)
      << " cm/s^2" << endl;

#endif

  int tiempoMaximoMediaCurvaS = algPP.tiempoMaxCurvaS / 2;

// Cuando la rueda debe mantener la velocidad constante, no hay cambio entre la orden de control
// anterior y la nueva orden de control (Vf = Vi), el tiempo de desplazamiento calculado para esa rueda
// es de 0 ms. Este resultado es debido a que se ha calculado este tiempo con la formula de
// un perfil de velocidad de tipo curva S. tiempoMed... = ceil(Vf - Vi/acelMax). Obviamente un
// tiempo de 0 ms no es correcto, lo que ocurre es que la rueda se debe mover durante to_do
// el tiempo que dura tiempoMaxCurvaS a la misma velocidad.
// Deberia ser == 0 pero debido al redondeo donde debiera aparecer un 0 a veces apareace un 1.
// De ahí que se ponga un <= 1
  if (algPP.tiempoMediaCurvaSRuedaDer <= 1)
  {
    // NOTA: Si en un perfil de velocidad de tipo curva S, la velocidad inicial y final
    // son iguales, la curva S se convierte en un tramo recto.
    algPP.velRuedaDerFinal = velRuedaDerInicial;
    algPP.tiempoMediaCurvaSRuedaDer = tiempoMaximoMediaCurvaS;

#ifdef DEBUG
    cout << endl;
    cout << "Vd: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
    cout << "tCSd: 0 ms ==> " << 2.0f * algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
#endif

  }
  else if (algPP.tiempoMediaCurvaSRuedaDer > tiempoMaximoMediaCurvaS)
  {

    float tiempoCurvaSRuedaDer = 2.0f * algPP.tiempoMediaCurvaSRuedaDer;

#ifdef DEBUG
    cout << endl;
    cout << "tCSd (" << tiempoCurvaSRuedaDer << ") > tMaxCS (" << algPP.tiempoMaxCurvaS << ")" << endl;
#endif

    algPP.tiempoMediaCurvaSRuedaDer = tiempoMaximoMediaCurvaS;
    // Rectificar la velocidad de la rueda derecha. Se calcula cual es la velocidad maxima
    // que se puede conseguir usando una aceleracion acelRuedaMax y un tiempo de media
    // curva S de valor tiempoMaximoMediaCurvaS.
    int signoDeltaVelRuedaDer = sign(algPP.velRuedaDerFinal - velRuedaDerInicial);
    vel = velRuedaDerInicial
        + (signoDeltaVelRuedaDer * Acel_max * (static_cast<float>(algPP.tiempoMediaCurvaSRuedaDer) / 1000.0f));

    // floorf o ceilf en funcion de si la velocidad aumenta o disminuye para no exceder la aceleracion
    // maxima permitida.
    if (signoDeltaVelRuedaDer > 0)
    {
      algPP.velRuedaDerFinal = floorf(1000.0f * vel) / 1000.0f;
    }
    else
    {
      algPP.velRuedaDerFinal = ceilf(1000.0f * vel) / 1000.0f;
    }

#ifdef DEBUG
    cout << endl;
    cout << "Vd: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
    cout << "tCSd:" << tiempoCurvaSRuedaDer << " ms ==> " << 2.0f * algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
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
    algPP.tiempoMediaCurvaSRuedaIzq = tiempoMaximoMediaCurvaS;

#ifdef DEBUG
    cout << endl;
    cout << "Vd: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
    cout << "tCSd: 0 ms ==> " << 2.0f * algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
#endif

#ifdef DEBUG
    cout << endl;
    cout << "Vi: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
    cout << "tCSi = 0 ms ==> " << 2.0f * algPP.tiempoMediaCurvaSRuedaIzq << " ms " << endl;
#endif

  }
  else if (algPP.tiempoMediaCurvaSRuedaIzq > tiempoMaximoMediaCurvaS)
  {

    float tiempoCurvaSRuedaIzq = 2.0f * algPP.tiempoMediaCurvaSRuedaIzq;

#ifdef DEBUG
    cout << endl;
    cout << "tCSi (" << tiempoCurvaSRuedaIzq << ") > tMaxCS (" << algPP.tiempoMaxCurvaS << ")" << endl;
#endif

    algPP.tiempoMediaCurvaSRuedaIzq = tiempoMaximoMediaCurvaS;
    int signoDeltaVelRuedaIzq = sign(algPP.velRuedaIzqFinal - velRuedaIzqInicial);
    vel = velRuedaIzqInicial
        + (signoDeltaVelRuedaIzq * Acel_max * (static_cast<float>(algPP.tiempoMediaCurvaSRuedaIzq) / 1000.0f));

    if (signoDeltaVelRuedaIzq > 0)
    {
      algPP.velRuedaIzqFinal = floorf(1000.0f * vel) / 1000.0f;
    }
    else
    {
      algPP.velRuedaIzqFinal = ceilf(1000.0f * vel) / 1000.0f;
    }

#ifdef DEBUG
    cout << endl;
    cout << "Vi: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
    cout << "tCSi:" << tiempoCurvaSRuedaIzq << " ms ==> " << 2.0f * algPP.tiempoMediaCurvaSRuedaIzq << " ms " << endl;
#endif

  }
// else if (algPP.tiempoMediaCurvaSRuedaIzq < 50)
// {
//algPP.tiempoMediaCurvaSRuedaIz = 50;
//aiM = abs(ViF - ViI)/(tiempoMediaCurvaSRuedaIzq/1000);
//}
#ifdef DEBUG
  cout << endl;
  cout << "Vd: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tCSd: " << 2.0f * algPP.tiempoMediaCurvaSRuedaDer << " ms" << endl;
  cout << "Vi: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
  cout << "tCSi: " << 2.0f * algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;
  cout << "Acel_max: " << Acel_max << " cm/s^2" << endl;
  cout << "adF: "
      << (algPP.velRuedaDerFinal - velRuedaDerInicial) / (static_cast<float>(algPP.tiempoMediaCurvaSRuedaDer) / 1000.0f)
      << " cm/s^2" << endl;
  cout << "aiF: "
      << (algPP.velRuedaIzqFinal - velRuedaIzqInicial) / (static_cast<float>(algPP.tiempoMediaCurvaSRuedaIzq) / 1000.0f)
      << " cm/s^2" << endl;
#endif

#ifndef PRUEBA
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP, cuentas_cm_recorrido);
#else
  return 0;
#endif
}

int escribirDPRAMVelocidadFinalTiempoMediaCurvaS(const int & descriptorDPRAM, const algoritmoPurePursuit & algPP,
                                                 const float& cuentas_cm_recorrido)
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
  float vdF = cuentas_cm_recorrido * (algPP.velRuedaDerFinal / 1000.0f); // En cts/ms

  if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_DER_FINAL, (char *)&(vdF), NBYTES))
  {
    return -1;
  }

  float viF = cuentas_cm_recorrido * (algPP.velRuedaIzqFinal / 1000.0f); // En cts/ms

  if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_IZQ_FINAL, (char *)&(viF), NBYTES))
  {
    return -1;
  }

  int tiempoCurvaSRuedaDer = 2 * algPP.tiempoMediaCurvaSRuedaDer;

  if (escribirDPRAM(descriptorDPRAM, DIR_TIEMPO_MEDIA_CURVAS_RUEDA_DER, (char *)&tiempoCurvaSRuedaDer, NBYTES))
  {
    return -1;
  }

  int tiempoCurvaSRuedaIzq = 2 * algPP.tiempoMediaCurvaSRuedaIzq;

  if (escribirDPRAM(descriptorDPRAM, DIR_TIEMPO_MEDIA_CURVAS_RUEDA_IZQ, (char *)&tiempoCurvaSRuedaIzq, NBYTES))
  {
    return -1;
  }

  char teclaParaContinuar = '\0';
  //std::cout << "Introduce una tecla para continuar: ";
  //std::cin >> teclaParaContinuar;
  //char temp = 0;
  //scanf("%c", &temp);
  //std::cout << std::endl;

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

int pararRobot(const int& descriptorDPRAM, algoritmoPurePursuit& algPP, const float& cuentas_cm_recorrido)
{

  cout << "PARAR EL ROBOT" << endl;

  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;
  algPP.velRuedaDerFinal = 0;
  algPP.velRuedaIzqFinal = 0;
  algPP.tiempoMediaCurvaSRuedaDer = ceilf(
      1000 * fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / algPP.acelRuedaMax);
  algPP.tiempoMediaCurvaSRuedaIzq = ceilf(
      1000 * fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / algPP.acelRuedaMax);
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP, cuentas_cm_recorrido);
}

int rotacion(const int& descriptorDPRAM, double& deltaTheta, algoritmoPurePursuit& algPP,
             const float& cuentas_cm_recorrido)
{

// Comprobar errores.
  if (algPP.velAngBaseMax <= 0 || algPP.acelRuedaMax <= 0 || algPP.radioBase <= 0)
  {
    return -1;
  }

  if (deltaTheta == 0)
  {
    cout << "El robot no rota" << endl;
    return -1;
  }

// La PMAC trabaja con numeros reales de 32 bits, es decir, float.
  float comandosPDer[2] = {0.0f, 0.0f};
  float comandoVDer = 0.0f;
  int comandosT[2] = {0, 0};

  double signoDeltaThetaDosPi = (deltaTheta < 0 ? -1 : deltaTheta > 0) * dosPi;

// Angulo que debe rotar por el robot. Entre [-pi, pi].
  if (fabs(deltaTheta) > dosPi)
  {
    // Si el valor fabs(deltaTheta) esta entre 0 y 2*pi el valor retornado por la operacion
    // fmodf(deltaTheta, signoDeltaThetaDosPi) es el valor de su primer argumento, es decir,
    // deltaTheta.
    // Si el valor de fabs(deltaTheta) es mayor que 2*pi, entonces la operacion fmod retorna
    // el angulo equivalente entre 0 y signoDeltaTheta*2*pi.
    deltaTheta = fmod(deltaTheta, signoDeltaThetaDosPi);
  }

// Finalmente hacer que el valor de deltaTheta este entre -pi y pi.
  if (fabs(deltaTheta) > M_PI)
  {
    deltaTheta -= signoDeltaThetaDosPi;
  }

  int signoDeltaTheta = (deltaTheta < 0 ? -1 : deltaTheta > 0);

  cout << endl;
  cout << "deltaTheta: " << gradRad * deltaTheta << " grad" << endl;

  float theta_min = radGrad * 5.0f;

  if (fabs(deltaTheta) < theta_min)
  {
    cout << endl;
    cout << "| deltaTheta | < " << gradRad * theta_min << " grad" << endl;
    cout << "El robot no rot_a sobre si mismo" << endl;
    return -1;
  }

//double velAngBaseMax = radGrad * algPP.velAngBaseMax;

  float Acel_max = 2.500; // cm/s^2

  if (algPP.velAngBaseMax > 10) // grad/s
  {
    algPP.velAngBaseMax = 10;
  }

// En cts / ms con 1 solo decimal, como se puede observar en el programa Windows de la PMAC (PEWIN32)
  float Vf_cts_ms = floorf(10.0f * ((algPP.radioBase * radGrad * algPP.velAngBaseMax * cuentas_cm_recorrido) / 1000.0f))
      / 10.0f;
// 4.297 cm/s ==> aprox 10 grad/s
// Quiero que tenga solo tres decimales. Floorf para que no exceda la velocidad angular maxima.
  float Vf = floorf(1000.0f * (1000.0f * Vf_cts_ms / cuentas_cm_recorrido)) / 1000.0f; // En cm/s
  float T1 = ceilf(1000.0f * (Vf / Acel_max)); // ms
  float L1 = Vf * (T1 / 1000.0f); // Lleva signo
  float Theta1 = L1 / algPP.radioBase;
  float Theta_2cs = 2.0f * Theta1;

  cout << "Acel_max: " << Acel_max << " cm/s^2" << endl;
  cout << "Vf: " << Vf << " cm/s" << endl;
  cout << "T1: " << T1 << " s" << endl;
  cout << "L1 " << L1 << " cm" << endl;
  cout << "Theta1: " << gradRad * Theta1 << " grad " << endl;
  cout << "Theta_2cs: " << gradRad * Theta_2cs << " grad " << endl;

  if (fabs(deltaTheta) <= Theta_2cs)
  {

    float L1_ = (deltaTheta * algPP.radioBase) / 2.0f;
    // ceilf para no exceder la aceleracion.
    float T1_ = ceilf(1000 * sqrt(fabs(L1_) / Acel_max));
    comandosT[0] = 2 * T1_;
    // floorf para no exceder la aceleracion.
    comandoVDer = signoDeltaTheta * floorf(1000 * Acel_max * (T1_ / 1000.0f)) / 1000;
    // Recalcular la longitud recorrida. Debido a la funcion ceilf y floorf la longitud
    // no coincide con L1_, aunque es muy aproximada.
    comandosPDer[0] = comandoVDer * (T1_ / 1000.0f);

    comandosT[1] = 0;
    comandosPDer[1] = 0.0f;

    cout << endl;
    cout << endl;
    cout << "| deltaTheta |  <= Theta_2cs" << endl;
    cout << "T1_: " << T1_ << " ms" << endl;
    cout << "L1_: " << L1_ << " cm" << endl;
  }
  else
  {
    comandosT[0] = 2 * T1;
    comandoVDer = signoDeltaTheta * Vf;
    comandosPDer[0] = signoDeltaTheta * L1;

    float L_tramo_vcte = (deltaTheta * algPP.radioBase) - (2 * comandosPDer[0]);
    // Aqui se puede usar ceilf o floorf porque no existe aceleracion, ya que vf
    // se mantiene constante durante 2*T_tramo_cte ms
    float T_tramo_vcte = ceilf(1000 * (L_tramo_vcte / (comandoVDer * 2.0f)));
    comandosT[1] = 2 * T_tramo_vcte;
    // Recalcular la longitud recorrida. Debido a la funcion ceilf la longitud
    // no coincide con L_tramo_cte, aunque es muy aproximada.
    comandosPDer[1] = 2 * comandoVDer * (T_tramo_vcte / 1000.0f);

    cout << endl;
    cout << endl;
    cout << "| deltaTheta |  > Theta_2cs" << endl;
    cout << "T_tramo_vcte: " << T_tramo_vcte << " ms" << endl;
    cout << "L_tot: " << (deltaTheta * algPP.radioBase) << " cm" << endl;
    cout << "L_tramo_vcte    : " << L_tramo_vcte << " cm" << endl;
    cout << "L_tramo_vcte / 2: " << L_tramo_vcte / 2.0f << " cm" << endl;
  }

#ifdef DEBUG

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
  cout << endl;
  cout << "Aceleracion: " << fabs(comandoVDer) / ((comandosT[0] / 2) / 1000.0f) << " cm/s^2" << endl;

#endif

  int error = 0;
  char datosParaPC[NUM_MAX_CARAC_BUFFER_DPRAM_PC];
  memset(datosParaPC, '\0', NUM_MAX_CARAC_BUFFER_DPRAM_PC * sizeof(char));

#ifndef PRUEBA

  if (escribirRotacionBaseDPRAM(descriptorDPRAM, comandosPDer, &comandoVDer, comandosT, datosParaPC, &error))
  {
    std::cout << "ROTACION FALLIDA" << std::endl;
  }

  if (error != 0)
  {
    std::cout << "ERROR INDICADO POR LA PMAC: " << error << std::endl;
  }

#ifdef DEBUG

  cout << "COMANDOS DE ROTACION ESCRITOS EN DPRAM" << endl;

#endif

//cout <<"datosParaPC: %s.\n", datosParaPC);
  int bitVelocidadCeroMotor1 = -1;
// EL bit de velocidad 0 vale 1 porque el robot esta parado. En el momento que
// se mueve se pone a 1. Cuando el robot se para se pone a 1.

// LEER ESTADO MIENTRAS EL ROBOT ESTE PARADO
//int i = 0;

  do
  {
    //cout << "\n[  " << i << "  ]" << endl;
    usleep(50000);
    if (leerEstadoMotor(descriptorDPRAM, 1, &bitVelocidadCeroMotor1))
    {
      return -1;
    }
    //i++;
  } while (bitVelocidadCeroMotor1);

#ifdef DEBUG

  printf("\nROBOT EN MOVIMIENTO\n");

#endif

// LEER ESTADO MIENTRAS EL ROBOT ESTE EN MOVIMIENTO
//i = 0;

  do
  {
    //cout << "\n[  " << i << "  ]" << endl;
    usleep(50000);
    if (leerEstadoMotor(descriptorDPRAM, 1, &bitVelocidadCeroMotor1))
    {
      return -1;
    }
    //i++;
  } while (!bitVelocidadCeroMotor1);

#ifdef DEBUG

  printf("\nROBOT PARADO.\n");

#endif

#endif

  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "nodoPurePursuit");
  ros::NodeHandle nh;
  tf::TransformListener tfListener;
  tf::StampedTransform stampedTf;
  ros::Subscriber suscriptorMapa = nh.subscribe("map", 1, obtenerMapa);

  std::cout << std::setprecision(3) << std::fixed;

//int numeroLocalizacionesOdometria = 0;

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

  double deltaX = 0.0f;
  double deltaY = 0.0f;
  double distanciaAlObjetivo = 0.0f;
  double thetaError = 0.0f;

  algoritmoPurePursuit algPP;

// Adquisicion de parametros del servidor master
  std::string parametro;

  printf("1\n");

  if (!nh.hasParam("/NombreMapa"))
  {
    return -1;
  }
  std::string nombreMapa;
  nh.getParam("/NombreMapa", nombreMapa);

  printf("2\n");

  if (!nh.hasParam("/RelacionReduccion"))
  {
    return -1;
  }
  nh.getParam("/RelacionReduccion", parametro);
  int relacion_reduccion = atoi(parametro.c_str());

  printf("3\n");

  if (!nh.hasParam("/CuentasEncoder"))
  {
    return -1;
  }
  nh.getParam("/CuentasEncoder", parametro);
  int cuentas_encoder = atoi(parametro.c_str());

  printf("4\n");

  if (!nh.hasParam("/FactorMultiplicativoCuentasPMAC"))
  {
    return -1;
  }
  nh.getParam("/FactorMultiplicativoCuentasPMAC", parametro);
  int factor_multiplicativo_cuentas_PMAC = atoi(parametro.c_str());

  printf("5\n");

  if (!nh.hasParam("/RadioRuedaM"))
  {
    return -1;
  }
  nh.getParam("/RadioRuedaM", parametro);
  algPP.radioRueda = 100 * atof(parametro.c_str()); // En cm

  printf("6\n");

  if (!nh.hasParam("/RadioBaseM"))
  {
    return -1;
  }
  nh.getParam("/RadioBaseM", parametro);
  algPP.radioBase = 100 * atof(parametro.c_str()); // En cm

  printf("7\n");

// Radio a partir del que se considera que el robot se desplaza en linea recta.
  algPP.radioUmbralLinRecta = 10.0f * algPP.radioBase;
  algPP.radioConSignoFinal = 0.0f;

  if (!nh.hasParam("/VelocidadRuedaMaximaMpS"))
  {
    return -1;
  }
  nh.getParam("/VelocidadRuedaMaximaMpS", parametro);
  algPP.velRuedaMax = 100 * atof(parametro.c_str()); // En cm/s

  if (!nh.hasParam("/AceleracionRuedaMaximaMpS2"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/AceleracionRuedaMaximaMpS2", parametro);
  algPP.acelRuedaMax = 100 * atof(parametro.c_str()); //En cm/s^2

  if (!nh.hasParam("/VelocidadAngularBaseMaximaGpS"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/VelocidadAngularBaseMaximaGpS", parametro);
  algPP.velAngBaseMax = atof(parametro.c_str()); //En grad/s

  algPP.velRuedaDerFinal = 0.0f;
  algPP.velRuedaIzqFinal = 0.0f;
  algPP.tiempoMediaCurvaSRuedaDer = 0.0f;
  algPP.tiempoMediaCurvaSRuedaIzq = 0.0f;

  if (!nh.hasParam("/TiempoMaximoMediaCurvaSS"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/TiempoMaximoMediaCurvaSS", parametro);
  algPP.tiempoMaxCurvaS = 2 * static_cast<int>(ceilf(1000 * atof(parametro.c_str()))); //En ms

// Recibir el mapa y obtener el tamanio de la celda en M.
  do
  {
    ros::spinOnce();
  } while (!flag_mapa_recibido);

// El tiempo de desplazamiento es 50 ms superior al tiempo maximo que puede
// una curva S de velocidad
  algPP.tiempoDesplazamiento = algPP.tiempoMaxCurvaS + 50;  // En ms
  double diametroBase = 2.0f * algPP.radioBase; // En cm
  double radioConSignoFinalAnterior = 0.0f;
  double deltaRadioConSigno = 0.0f;
  double lado = 1.25 * 0.01 * algPP.velRuedaMax * (algPP.tiempoDesplazamiento / 1000.0f); // 25 % mayor de lo estrictamente necesario
  double xInferiorAreaReposo = x_goal - lado;
  double xSuperiorAreaReposo = x_goal + lado;
  double yInferiorAreaReposo = y_goal - lado;
  double ySuperiorAreaReposo = y_goal + lado;

// 6 decimales. Como en la definicion usada en el fichero de configuracion de la PMAC.
  float cuentas_revolucion = factor_multiplicativo_cuentas_PMAC * relacion_reduccion * cuentas_encoder;
  float cuentas_cm_recorrido = floorf(
      1000000.0f * (static_cast<float>(cuentas_revolucion) / (2 * M_PI * algPP.radioRueda))) / 1000000.0f;

  int descriptorDPRAM = 0;

#ifndef PRUEBA
  if ((descriptorDPRAM = open(DPRAM, O_RDWR)) == -1)
  {
    cout << "\x1B[31;1m";
    cout << "No se puede abrir el dispositivo: " << DPRAM << endl;
    cout << "\x1B[0m";
    //delete nombreMapa;
    return -1;
  }
#endif

#ifdef DEBUG
  cout << endl;
  cout << "NombreMapa: " << nombreMapa << endl;
  cout << "Tamanio celda: " << tamanio_celda_M << " m" << endl;
  cout << "RelacionReduccion: " << relacion_reduccion << endl;
  cout << "CuentasEncoder: " << cuentas_encoder << " ppr" << endl;
  cout << "FactorMultiplicativoCuentasPMAC: " << factor_multiplicativo_cuentas_PMAC << endl;
  cout << "cuentas_revolucion: " << cuentas_revolucion << endl;
  cout << "cuentas_cm_recorrido: " << cuentas_cm_recorrido << endl;
  cout << "Radio de la rueda: " << algPP.radioRueda << " cm" << endl;
  cout << "Radio de la base: " << algPP.radioBase << " cm" << endl;
  cout << "Diametro de la base: " << diametroBase << " cm" << endl;
  cout << "Radio umbral de trayectoria en linea recta: " << algPP.radioUmbralLinRecta << " cm" << endl;
  cout << "V ruedas max: " << algPP.velRuedaMax << " cm/s" << endl;
  cout << "A ruedas max: " << algPP.acelRuedaMax << " cm/s^2" << endl;
  cout << "V angular max: " << algPP.velAngBaseMax << " grad/s" << endl;
  cout << "T curva S maximo: " << algPP.tiempoMaxCurvaS << " ms" << endl;
  cout << "T desplazamiento: " << algPP.tiempoDesplazamiento << " ms" << endl;
  cout << "Lado: " << lado << " m" << endl;
  cout << "xInferiorAreaReposo: " << xInferiorAreaReposo << " m" << endl;
  cout << "xSuperiorAreaReposo: " << xSuperiorAreaReposo << " m" << endl;
  cout << "yInferiorAreaReposo: " << yInferiorAreaReposo << " m" << endl;
  cout << "ySuperiorAreaReposo: " << ySuperiorAreaReposo << " m" << endl;
#endif

  ros::Time marca_tiempo;
  mr::Pose* p_init = NULL;
  mr::Pose* p_goal = new mr::Pose(x_goal, y_goal);
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

#ifdef DEUG
      cout << "p_init->x: " << p_init->x << " m " << endl;
      cout << "p_init->y: " << p_init->y << " m " << endl;
      cout << "p_goal->x: " << p_goal->x << " m " << endl;
      cout << "p_goal->y: " << p_goal->y << " m " << endl;
#endif

      z_mbase_en_mmapa_ini = stampedTf.getOrigin().z();
      t_mbase_en_mmapa_ini = tf::getYaw(stampedTf.getRotation());

    }
    catch (tf::TransformException& ex)
    {
      cout << "\x1B[31;1m";
      cout << ex.what() << endl;
      cout << "\x1B[0m";
    }
    sleep(1);
  }

  transformacion_escuchada = false;
  float dist_separacion = 0.0f;

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
// Primer y ultima punto de la trayectoria seguida por el punto medio de la base
  double ang_primer_tramo = atan2(path.points[1].y - path.points[0].y, path.points[1].x - path.points[0].x);

  int iFinal = path.points.size() - 1;

  double ang_ultimo_tramo = atan2(path.points[iFinal].y - path.points[iFinal - 1].y,
                                  path.points[iFinal].x - path.points[iFinal - 1].x);
// +M_PI/2 porque la trayectoria que debe seguir el punto medio de la base (link_base) es paralela a la trayectoria
// calculada por FM, situada a su izquierda, a una distancia 'dist_separacion'
  puntos_tray_ok[0].x = path.points[0].x + (dist_separacion * cos(ang_primer_tramo + M_PI / 2));
  puntos_tray_ok[0].y = path.points[0].y + (dist_separacion * sin(ang_primer_tramo + M_PI / 2));
  puntos_tray_ok[iFinal].x = path.points[iFinal].x + (dist_separacion * cos(ang_ultimo_tramo + M_PI / 2));
  puntos_tray_ok[iFinal].y = path.points[iFinal].y + (dist_separacion * sin(ang_ultimo_tramo + M_PI / 2));

#ifdef DEBUG

  cout << "(" << path.points[0].x << ", " << path.points[0].y << ")  --  (" << puntos_tray_ok[0].x << ", "
      << puntos_tray_ok[0].y << ")" << endl;

#endif

  double angulo_previo = 0;
  double angulo_siguiente = 0;
  double difAng = 0;
  double ang_final = 0;
// Resto de puntos seguidos por el punto medio de la base.
  for (unsigned int i = 1; i < iFinal; i++)
  {
    angulo_previo = atan2(path.points[i].y - path.points[i - 1].y, path.points[i].x - path.points[i - 1].x);
    angulo_siguiente = atan2(path.points[i + 1].y - path.points[i].y, path.points[i + 1].x - path.points[i].x);
    difAng = angulo_siguiente - angulo_previo;
    ang_final = angulo_previo + (M_PI / 2) + (difAng / 2);
    puntos_tray_ok[i].x = path.points[i].x + (dist_separacion * cos(ang_final));
    puntos_tray_ok[i].y = path.points[i].y + (dist_separacion * sin(ang_final));

#ifdef DEBUG

    cout << "(" << path.points[i].x << ", " << path.points[i].y << ")  --  (" << puntos_tray_ok[i].x << ", "
        << puntos_tray_ok[i].y << ")" << endl;

#endif
  }

#ifdef DEBUG

  cout << "(" << path.points.back().x << ", " << path.points.back().y << ")  --  (" << puntos_tray_ok.back().x << ", "
      << puntos_tray_ok.back().y << ")" << endl;

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

  z_mbase_en_mmapa_ant = z_mbase_en_mmapa_ini;
  z_mbase_en_mmapa = z_mbase_en_mmapa_ini;

  t_mbase_en_mmapa_ant = t_mbase_en_mmapa_ini;
  t_mbase_en_mmapa = t_mbase_en_mmapa_ini;

  /*
   tiempoDesplazamiento en ms
   frec = 1 /tiempoDesplazamientoSeg o 1000/tiempoDesplazamientoMSeg
   Si hago que la frecuencia de muestreo de la ubicacion del robot, es decir, la frecuencia a la que
   se van a dar acciones de control sea menor que este valor, es equivalente a permitir que el
   robot se desplace un poco más de tiempo con la accion de control anterior.
   Para hacer que la frec sea un poco menor de lo estricamente necesaria uso 'floorf'.
   */

  //int frec = floorf(1000 / algPP.tiempoDesplazamiento);
  float frec = 0.5;

  ros::Rate r(frec);

#ifdef DEBUG

  cout << endl;
  cout << "MBase en MMapa" << endl;
  cout << "(" << x_mbase_en_mmapa << " m,  " << y_mbase_en_mmapa << " m, " << z_mbase_en_mmapa << " m, "
      << gradRad * t_mbase_en_mmapa << " grad)" << endl;
//cout << "x_mbase_mmapa: " << x_mbase_en_mmapa << " m " << endl;
//cout << "y_mbase_mmapa: " << y_mbase_en_mmapa << " m" << endl;
//cout << "z_mbase_mmapa: " << z_mbase_en_mmapa << " m " << endl;
//cout << "t_mbase_mmapa: " << gradRad * t_mbase_en_mmapa << " grad " << endl;
  cout << "Frecuencia refresco: " << frec << " Hz" << endl;

#endif

  deltaX = puntos_tray_ok[1].x - x_mbase_en_mmapa;
  deltaY = puntos_tray_ok[1].y - y_mbase_en_mmapa;
  double orientacionTramo = atan2f(deltaY, deltaX);
// Angulo que debe rotar por el robot.
  double deltaTheta = orientacionTramo - t_mbase_en_mmapa;
  rotacion(descriptorDPRAM, deltaTheta, algPP, cuentas_cm_recorrido);

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
      cout << "MBase en MMapa" << endl;
      cout << "(" << x_mbase_en_mmapa << " m,  " << y_mbase_en_mmapa << " m, " << gradRad * t_mbase_en_mmapa << " grad)"
          << endl;
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
//std::cout << "Introduce una tecla para continuar: ";
//std::cin >> teclaParaContinuar;
//char temp = 0;
//scanf("%c", &temp);
//std::cout << std::endl;

  ros::Time timeInicio;

  int iSel = 1;

  while (x_mbase_en_mmapa < xInferiorAreaReposo || x_mbase_en_mmapa > xSuperiorAreaReposo
      || y_mbase_en_mmapa < yInferiorAreaReposo || y_mbase_en_mmapa > ySuperiorAreaReposo)
  {
    // deltaX y deltaY en m
    // distanciaAlObjetivo en m ==> pasar a cm.
    // diametroBase en cm.

#ifdef DEBUG

    cout << endl;
    cout << endl;
    cout << endl;
    cout << "\x1B[31;1m";
    cout << "[  " << iSel << "  ] " << endl;
    cout << "\x1B[0m";

#endif

    deltaX = puntos_tray_ok[iSel].x - x_mbase_en_mmapa; // En m
    deltaY = puntos_tray_ok[iSel].y - y_mbase_en_mmapa; // En m
    distanciaAlObjetivo = 100 * hypot(deltaX, deltaY); // En cm

    cout << "\t(" << x_mbase_en_mmapa << " m,  " << y_mbase_en_mmapa << " m, " << gradRad * t_mbase_en_mmapa << " grad)"
        << "  ==>  " << "(" << puntos_tray_ok[iSel].x << " m,  " << puntos_tray_ok[iSel].y << " m)" << endl;
    cout << "\tdeltaX: " << 100 * deltaX << " cm" << endl;
    cout << "\tdeltaY: " << 100 * deltaY << " cm" << endl;
    cout << "\tDist: " << distanciaAlObjetivo << " cm" << endl;

    // Seleccion del proximo punto objetivo dentro de la trayectoria.
    while (iSel < iFinal && distanciaAlObjetivo < diametroBase)
    {

      iSel += 1;

#ifdef DEBUG

      cout << "\x1B[31;1m";
      cout << "[[ " << iSel << " ]] " << endl;
      cout << "\x1B[0m";

#endif

      deltaX = puntos_tray_ok[iSel].x - x_mbase_en_mmapa;
      deltaY = puntos_tray_ok[iSel].y - y_mbase_en_mmapa;
      distanciaAlObjetivo = 100 * hypot(deltaX, deltaY);

      cout << "\t(" << x_mbase_en_mmapa << " m,  " << y_mbase_en_mmapa << " m, " << gradRad * t_mbase_en_mmapa
          << " grad)" << "  ==>  " << "(" << puntos_tray_ok[iSel].x << " m,  " << puntos_tray_ok[iSel].y << " m)"
          << endl;
      cout << "\tdeltaX: " << 100 * deltaX << " cm" << endl;
      cout << "\tdeltaY: " << 100 * deltaY << " cm" << endl;
      cout << "\tDist: " << distanciaAlObjetivo << " cm" << endl;

    }

    thetaError = atan2f(deltaY, deltaX) - t_mbase_en_mmapa;

#ifdef DEBUG
    cout << "\ttError: " << gradRad * thetaError << " grad" << endl;

    if (fabs(thetaError) > M_PI)
    {
      cout << "\t\tESTO NUNCA DEBERIA OCURRIR!!!!!" << endl;
    }

#endif

    // Si thetaError = 0.0 rad entonces division por infinito. Numero indeterminado.
    // Para evitar esto se suma un angulo muy pequeño: 0.0000001 rad.
    algPP.radioConSignoFinal = distanciaAlObjetivo / (2 * sin(thetaError+0.000001));

#ifdef DEBUG
    cout << "\trSF: " << algPP.radioConSignoFinal << " cm" << endl;
#endif

    if (fabs(algPP.radioConSignoFinal) < algPP.radioBase)
    {
      algPP.radioConSignoFinal = (algPP.radioConSignoFinal < 0 ? -1 : algPP.radioConSignoFinal > 0) * algPP.radioBase;

#ifdef DEBUG
      cout << "\trSF: " << algPP.radioConSignoFinal << " cm" << endl;
#endif
    }

    /*
     deltaRadioConSigno = algPP.radioConSignoFinal - radioConSignoFinalAnterior;
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

    actualizarVelFinalRuedas(descriptorDPRAM, algPP, cuentas_cm_recorrido);

#ifdef DEBUG

    cout << endl;
    cout << "DORMIR" << endl;

#endif

    timeInicio = ros::Time::now();

    usleep(550000);

    marca_tiempo = ros::Time::now();

#ifdef DEBUG

    cout << endl;
    cout << "DESPERTARSE [ " << (timeInicio - marca_tiempo).toSec() << " s ]" << endl;

#endif

    x_mbase_en_mmapa_ant = x_mbase_en_mmapa;
    y_mbase_en_mmapa_ant = y_mbase_en_mmapa;
    t_mbase_en_mmapa_ant = t_mbase_en_mmapa;

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
        cout << "(" << x_mbase_en_mmapa << " m,  " << y_mbase_en_mmapa << " m, " << gradRad * t_mbase_en_mmapa
            << " grad)" << endl;
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

  pararRobot(descriptorDPRAM, algPP, cuentas_cm_recorrido);

//}

//delete nombreMapa;
  delete img;
  delete gridmap;
  delete fm2pathplanner;
  delete p_init;
  delete p_goal;
  return 0;
}
