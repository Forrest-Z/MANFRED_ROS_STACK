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

int sign(double numero)
{
  // numero < 0 --> return -1
  // numero = 0 --> numero > 0 es false, false equivale a 0, por tanto return 0.
  // return > 0 --> numero > 0
  return numero < 0 ? -1 : numero > 0; // # -1, 0, +1.
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
  cout << "Aceleracion: " << fabs(comandoVDer) / (comandosT[0] / 1000.0f) << " cm/s^2" << endl;

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
  }while (bitVelocidadCeroMotor1);

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
  }while (!bitVelocidadCeroMotor1);

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

  algoritmoPurePursuit algPP;

// Adquisicion de parametros del servidor master
  std::string parametro;
  printf("0\n");
  printf("1\n");

  if (!nh.hasParam("/RelacionReduccion"))
  {
    return -1;
  }
  nh.getParam("/RelacionReduccion", parametro);
  int relacion_reduccion = atoi(parametro.c_str()); // En cm

  if (!nh.hasParam("/CuentasEncoder"))
  {
    return -1;
  }
  nh.getParam("/CuentasEncoder", parametro);
  int cuentas_encoder = atoi(parametro.c_str()); // En cm

  if (!nh.hasParam("/FactorMultiplicativoCuentasPMAC"))
  {
    return -1;
  }
  nh.getParam("/FactorMultiplicativoCuentasPMAC", parametro);
  int factor_multiplicativo_cuentas_PMAC = atoi(parametro.c_str()); // En cm

  if (!nh.hasParam("/RadioRuedaM"))
  {
    return -1;
  }
  nh.getParam("/RadioRuedaM", parametro);
  algPP.radioRueda = 100 * atof(parametro.c_str()); // En cm

  if (!nh.hasParam("/RadioBaseM"))
  {
    return -1;
  }
  nh.getParam("/RadioBaseM", parametro);
  algPP.radioBase = 100 * atof(parametro.c_str()); // En cm

  printf("2\n");
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

  printf("3\n");
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

// El tiempo de desplazamiento es 50 ms superior al tiempo maximo que puede
// una curva S de velocidad
  algPP.tiempoDesplazamiento = algPP.tiempoMaxCurvaS + 50;  // En ms
  double diametroBase = 2.0f * algPP.radioBase; // En cm

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
#endif

  ros::Time marca_tiempo;

  double angulo = 0;

  while (nh.ok())
  {

    cout << endl;
    cout << endl;
    cout << endl;
    std::cout << "Introduce una angulo (en grados) para continuar: ";
    std::cin >> angulo;

    if (angulo == 1000)
    {
      break;
    }
    //char temp = 0;
    //scanf("%c", &temp);
    //std::cout << std::endl;
    angulo = radGrad * angulo;
    rotacion(descriptorDPRAM, angulo, algPP, cuentas_cm_recorrido);

  }

  return 0;
}
