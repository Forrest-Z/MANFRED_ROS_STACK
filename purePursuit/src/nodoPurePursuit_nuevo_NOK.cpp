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
#include "datatype/geometry/vector2d.h"
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
static const double dosPi = 2.0 * M_PIl;

// Constante que pasa de radianes a grados.
static const double gradRad = 180.0 * M_1_PIl;

// Constante que pasa de grados a radianes.
static const double radGrad = M_PIl / 180.0;

// Estructura que almacena informacion util del algoritmo pure pursuit.
struct algoritmoPurePursuit
{

  // Radio de la rueda, en cm.
  double radioRueda;

  // Radio de la base, en cm.
  double radioBase;

  // Radio de curvatura, en cm, a partir del cual se considera que el robot se desplaza en linea recta.
  double radioUmbralLinRecta;

  // Radio de curvatura, en cm, con el que gira la base.
  double radioConSignoFinal;

  // Tiempo que puede durar como mucho una curva S, en cualquiera de la dos ruedas. En ms.
  int tiempoMaxCurvaS; //

  // Tiempo en desplazamiento en ms.
  int tiempoDesplazamiento;

  // Velocidad maxima de la rueda en cm/s.
  double velRuedaMax; // V2

  // Velocidad mínima de la rueda en cm/s.
  double velRuedaMin; // V1

  // Aceleracion maxima de la rueda en cm/s^2.
  double acelRuedaMax;

  // Aceleracion maxima de la rueda en cm/s^2, en la fase inicial de rotación.
  double acelRuedaMaxRotacion;

  // Velocidad angular de la base en rad/s.
  double velAngBaseMax; // W2

  // Velocidad angular de la base en rad/s.
  double velAngBaseMin; // W1

  // Ángulo mínimo para rotar, en rad.
  double thetaErrorMinimo;

  double thetaErrorMaximo;

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

int escribirDPRAMVelocidadFinalTiempoMediaCurvaS(const int & descriptorDPRAM, const algoritmoPurePursuit& algPP,
                                                 const double& cuentas_cm_recorrido);

int sign(double numero)
{
  // numero < 0 --> return -1
  // numero = 0 --> numero > 0 es false, false equivale a 0, por tanto return 0.
  // return > 0 --> numero > 0
  return numero < 0 ? -1 : numero > 0; // # -1, 0, +1.
}

void buscarPuntoDestino(const std::vector<mr::Vector2D>& trayectoria_link_base, int& indice_destino,
                        const double& x_link_base, const double& y_link_base, const double& t_link_base,
                        const double& L_ah_cm, double& theta_error, double& distancia_destino)
{

  double deltaX = 0.0;
  double deltaY = 0.0;

  // Seleccion del proximo punto objetivo dentro de la trayectoria.
  for (; indice_destino < trayectoria_link_base.size(); indice_destino++)
  {

    deltaX = trayectoria_link_base[indice_destino].x - x_link_base; // m
    deltaY = trayectoria_link_base[indice_destino].y - y_link_base; // m
    distancia_destino = 100.0 * hypot(deltaX, deltaY); // cm

#ifdef DEBUG

    cout << endl;
    cout << endl;
    cout << endl;
    cout << "\x1B[31;1m";
    cout << "[  " << indice_destino << "  ] " << endl;
    cout << "\x1B[0m";
    cout << "\t(" << x_link_base << " m,  " << y_link_base << " m, " << gradRad * t_link_base << " grad)" << "  ==>  "
        << "(" << trayectoria_link_base[indice_destino].x << " m,  " << trayectoria_link_base[indice_destino].y << " m)"
        << endl;
    cout << "\tdeltaX: " << 100 * deltaX << " cm" << endl;
    cout << "\tdeltaY: " << 100 * deltaY << " cm" << endl;
    cout << "\tdistancia_destino: " << distancia_destino << " cm" << endl;

#endif

    if (distancia_destino >= L_ah_cm)
    {

#ifdef DEBUG

      cout << "\t[[ " << indice_destino << " ]] " << endl;

#endif
      // Error de orientacion, entre la orientacion de la base y la orientacion del
      // punto destino.
      // 'theta_error' debe estar entre [-pi, pi];
      theta_error = atan2f(deltaY, deltaX) - t_link_base;
      double signoDOSPI = sign(theta_error) * 2 * M_PIl;

      // Si signoDOSPI = 0 ==> theta_error = 0, esto es,  atan2f(deltaY, deltaX) = t_link_base

      if (!signoDOSPI)
      {

        return;

      }

      theta_error = fmod(theta_error, signoDOSPI);

      if (fabs(theta_error) > M_PIl)
      {

        theta_error -= signoDOSPI;

      }

#ifdef DEBUG
      cout << "\ttheta_error: " << gradRad * theta_error << " grad" << endl;

      if (fabs(theta_error) > M_PIl / 2)
      {
        cout << "\tESTO NUNCA DEBERIA OCURRIR!!!!!" << endl;
      }
#endif
      break;

    }
  }
}

void crear_trayectoria_paralela(const mr::Path2D& trayectoria, std::vector<mr::Vector2D>& trayectoria_paralela,
                                const double& dist_separacion, const double& orientacion_marco_ref_paralelo)
{

// Primer punto de la trayectoria paralela.
  double phi = atan2(trayectoria.points[1].y - trayectoria.points[0].y,
                     trayectoria.points[1].x - trayectoria.points[0].x) + orientacion_marco_ref_paralelo;
  trayectoria_paralela.push_back(
      mr::Vector2D(trayectoria.points[0].x + (dist_separacion * cos(phi)),
                   trayectoria.points[0].y + (dist_separacion * sin(phi))));

#ifdef DEBUG

  cout << "(" << trayectoria.points[0].x << ", " << trayectoria.points[0].y << ")  -->  (" << trayectoria_paralela[0].x
      << ", " << trayectoria_paralela[0].y << ")" << endl;
#endif

  double psi_1 = 0;
  double psi_2 = 0;
  double psi_3 = 0;

// Resto de puntos seguidos por el punto medio de la base.
// El indice final es dos elementos por detras del último del vector.
  int indice_final = (trayectoria.points.size() - 1) - 2;

  for (unsigned int i = 0; i <= indice_final; i++)
  {

    psi_1 = atan2(trayectoria.points[i].y - trayectoria.points[i + 1].y,
                  trayectoria.points[i].x - trayectoria.points[i + 1].x);

    psi_2 = atan2(trayectoria.points[i + 2].y - trayectoria.points[i + 1].y,
                  trayectoria.points[i + 2].x - trayectoria.points[i + 1].x);

    psi_3 = psi_2 - psi_1;

    int signo_psi_3 = sign(psi_3);
    int signo_orient_mref_paralelo = sign(orientacion_marco_ref_paralelo);

    if (signo_psi_3 == signo_orient_mref_paralelo)
    {

      psi_3 = psi_3 - signo_orient_mref_paralelo * 2 * M_PI;
    }

    phi = psi_1 + psi_3 / 2;

    trayectoria_paralela.push_back(
        mr::Vector2D(trayectoria.points[i + 1].x + (dist_separacion * cos(phi)),
                     trayectoria.points[i + 1].y + (dist_separacion * sin(phi))));

#ifdef DEBUG

    cout << "(" << trayectoria.points[i + 1].x << ", " << trayectoria.points[i + 1].y << ")  -->  ("
        << trayectoria_paralela.back().x << ", " << trayectoria_paralela.back().y << ")" << endl;
#endif

  }

// El último indice de la trayectoria.
  indice_final += 2;

  phi = atan2(trayectoria.points[indice_final].y - trayectoria.points[indice_final - 1].y,
              trayectoria.points[indice_final].x - trayectoria.points[indice_final - 1].x)
      + orientacion_marco_ref_paralelo;
  trayectoria_paralela.push_back(
      mr::Vector2D(trayectoria.points[indice_final].x + (dist_separacion * cos(phi)),
                   trayectoria.points[indice_final].y + (dist_separacion * sin(phi))));

#ifdef DEBUG

  cout << "(" << trayectoria.points.back().x << ", " << trayectoria.points.back().y << ")  -->  ("
      << trayectoria_paralela.back().x << ", " << trayectoria_paralela.back().y << ")" << endl;

#endif

}

int actualizarVelFinalRuedas(const int & descriptorDPRAM, algoritmoPurePursuit& algPP, const double& VbF,
                             const double& deltaVmax, const double& deltaVmin, const double& cuentas_cm_recorrido)
{

  //if (radioConSignoFinalValorAbs == algPP.radioBase) --> Acel_max = 2.5f;
  // También hay que cambiar deltaVmax y deltaVmin!!

  // floorf o ceilf en funcion de si la velocidad aumenta o disminuye para no exceder la aceleracion
  // maxima permitida.
  //if (signoDeltaVelRuedaDer > 0) {
  //	algPP.velRuedaDerFinal = floorf(1000.0f * vel) / 1000.0f;
  //} else {
  //	algPP.velRuedaDerFinal = ceilf(1000.0f * vel) / 1000.0f;
  //}

  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;

  double vel_cts_ms = 0;
  double vel = 0;

  algPP.velRuedaDerFinal = (1 + algPP.radioBase / algPP.radioConSignoFinal) * VbF;
  double deltaVd = algPP.velRuedaDerFinal - velRuedaDerInicial;

  if (fabs(deltaVd) > deltaVmax)
  {

    deltaVd = sign(deltaVd) * deltaVmax;
    algPP.velRuedaDerFinal = velRuedaDerInicial + deltaVd;

  }

  if (fabs(deltaVd) >= deltaVmin && fabs(deltaVd) <= deltaVmax)
  {

    if (sign(deltaVd) > 0)
    {

      // 1 decimal, como en el PeWIN32
      vel_cts_ms = floor(10 * ((algPP.velRuedaDerFinal / 1000.0f) * cuentas_cm_recorrido)) / 10; // cts/ms
      vel = 1000.0f * (vel_cts_ms / cuentas_cm_recorrido); // cm/s
      // 5 decimal
      algPP.velRuedaDerFinal = floor(100000.0f * vel) / 100000.0f;

    }
    else
    {

      // 1 decimal, como en el PeWIN32
      vel_cts_ms = ceil(10.0 * ((algPP.velRuedaDerFinal / 1000.0) * cuentas_cm_recorrido)) / 10.0; // cts/ms
      vel = 1000.0 * (vel_cts_ms / cuentas_cm_recorrido); // cm/s
      // 5 decimal
      algPP.velRuedaDerFinal = ceil(100000.0 * vel) / 100000.0;

    }
    // aqui el floor o el ceil de arriba
    algPP.tiempoMediaCurvaSRuedaDer = static_cast<int>(ceil(1000.0 * fabs(deltaVd) / algPP.acelRuedaMax));

  }
  else
  {

    algPP.velRuedaDerFinal = velRuedaDerInicial;
    // algPP.tiempoMaxCurvaS es divisible por 2.
    algPP.tiempoMediaCurvaSRuedaDer = algPP.tiempoMaxCurvaS / 2;
  }

  algPP.velRuedaIzqFinal = (1 - algPP.radioBase / algPP.radioConSignoFinal) * VbF;
  double deltaVi = algPP.velRuedaIzqFinal - velRuedaIzqInicial;

  if (fabs(deltaVi) > deltaVmax)
  {

    deltaVi = sign(deltaVi) * deltaVmax;
    algPP.velRuedaIzqFinal = velRuedaIzqInicial + deltaVi;

  }

  if (fabs(deltaVi) >= deltaVmin && fabs(deltaVi) <= deltaVmax)
  {

    if (sign(deltaVi) > 0)
    {

      // 1 decimal, como en el PeWIN32
      vel_cts_ms = floor(10.0 * ((algPP.velRuedaIzqFinal / 1000.0) * cuentas_cm_recorrido)) / 10.0; // cts/ms
      vel = 1000.0 * (vel_cts_ms / cuentas_cm_recorrido); // cm/s
      // 5 decimal
      algPP.velRuedaIzqFinal = floor(100000.0 * vel) / 100000.0; // cm/s

    }
    else
    {

      // 1 decimal, como en el PeWIN32
      vel_cts_ms = ceil(10.0 * ((algPP.velRuedaIzqFinal / 1000.0) * cuentas_cm_recorrido)) / 10.0; // cts/ms
      vel = 1000.0 * (vel_cts_ms / cuentas_cm_recorrido); // cm/s
      // 5 decimal
      algPP.velRuedaIzqFinal = ceil(100000.0 * vel) / 100000.0; // cm/s

    }

    algPP.tiempoMediaCurvaSRuedaIzq = static_cast<int>(ceil(1000.0 * fabs(deltaVi) / algPP.acelRuedaMax)); // ms

  }
  else
  {

    algPP.velRuedaIzqFinal = velRuedaIzqInicial;
    // algPP.tiempoMaxCurvaS es divisible por 2.
    algPP.tiempoMediaCurvaSRuedaIzq = algPP.tiempoMaxCurvaS / 2;

  }

#ifdef DEBUG

  cout << endl;
  cout << "VbF: " << VbF << " cm/s" << endl;
  cout << "Vd: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tCSd: " << 2 * algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
  cout << "Vi: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
  cout << "tCSi: " << 2 * algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;
  cout << "Acel_max: " << algPP.acelRuedaMax << " cm/s^2" << endl;
  cout << "adF: " << fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / (algPP.tiempoMediaCurvaSRuedaDer / 1000.0)
      << " cm/s^2" << endl;
  cout << "aiF: " << fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / (algPP.tiempoMediaCurvaSRuedaIzq / 1000.0)
      << " cm/s^2" << endl;

#endif

#ifndef PRUEBA
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP, cuentas_cm_recorrido);
#else
  return 0;
#endif
}

int escribirDPRAMVelocidadFinalTiempoMediaCurvaS(const int& descriptorDPRAM, const algoritmoPurePursuit& algPP,
                                                 const double& cuentas_cm_recorrido)
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
  float VdF_cts_ms = static_cast<float>(cuentas_cm_recorrido * (algPP.velRuedaDerFinal / 1000.0)); // En cts/ms

  if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_DER_FINAL, (char *)&(VdF_cts_ms), NBYTES))
  {
    return -1;
  }

  float ViF_cts_ms = static_cast<float>(cuentas_cm_recorrido * (algPP.velRuedaIzqFinal / 1000.0)); // En cts/ms

  if (escribirDPRAM(descriptorDPRAM, DIR_VEL_RUEDA_IZQ_FINAL, (char *)&(ViF_cts_ms), NBYTES))
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

#ifdef DEBUG
  cout << "Filas: " << num_filas << endl;
  cout << "Columnas: " << num_columnas << endl;
#endif

  tamanio_celda_M = rp_msj_rejilla_ocupacion_rx->info.resolution;
  flag_mapa_recibido = true;

}

int pararRobot(const int& descriptorDPRAM, algoritmoPurePursuit& algPP, const double& theta_error,
               const double& cuentas_cm_recorrido)
{

  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;

  algPP.velRuedaDerFinal = 0;
  algPP.velRuedaIzqFinal = 0;

  double arco_d = 2 * theta_error * (algPP.radioConSignoFinal + algPP.radioBase);
  double arco_i = 2 * theta_error * (algPP.radioConSignoFinal - algPP.radioBase);

  algPP.tiempoMediaCurvaSRuedaDer = static_cast<int>(ceil(1000.0 * fabs(arco_d / velRuedaDerInicial)));
  algPP.tiempoMediaCurvaSRuedaIzq = static_cast<int>(ceil(1000.0 * fabs(arco_i / velRuedaIzqInicial)));

  double Td = static_cast<int>(ceil(1000.0 * fabs(-velRuedaDerInicial) / algPP.acelRuedaMax));
  double Ti = static_cast<int>(ceil(1000.0 * fabs(-velRuedaIzqInicial) / algPP.acelRuedaMax));

  algPP.tiempoMediaCurvaSRuedaDer = static_cast<int>(fmax(algPP.tiempoMediaCurvaSRuedaDer, Td));
  algPP.tiempoMediaCurvaSRuedaIzq = static_cast<int>(fmax(algPP.tiempoMediaCurvaSRuedaIzq, Ti));

  int T = static_cast<int>(fmax(algPP.tiempoMediaCurvaSRuedaDer, algPP.tiempoMediaCurvaSRuedaIzq));

  algPP.tiempoMediaCurvaSRuedaDer = T;
  algPP.tiempoMediaCurvaSRuedaIzq = T;

  cout << "PARAR EL ROBOT" << endl;

  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP, cuentas_cm_recorrido);
}

int rotacionInicial(const int& descriptorDPRAM, const std::vector<mr::Vector2D>& trayectoria_paralela,
                    const double& t_link_base, const algoritmoPurePursuit& algPP, const double& cuentas_cm_recorrido)
{

  double deltaX = trayectoria_paralela[1].x - trayectoria_paralela[0].x; // m
  double deltaY = trayectoria_paralela[1].y - trayectoria_paralela[0].y; // m
// Angulo que debe rotar por el robot.
  double theta_error = atan2f(deltaY, deltaX) - t_link_base; // rad

// float Acel_max = 2.500; // cm/s^2

// Comprobar errores.
  if (algPP.velAngBaseMax <= 0 || algPP.acelRuedaMaxRotacion <= 0 || algPP.radioBase <= 0)
  {
    return -1;
  }

// theta_error entre [0 ,+-360]
  double signoDOSPI = sign(theta_error) * dosPi; // rad

  // signoDOSPI = 0 si theta_err = 0, esto es, alfa = t_link_base, luego
  // no hace falta rotar.
  if (!signoDOSPI)
  {

    return 0;

  }

  theta_error = fmod(theta_error, signoDOSPI); // rad

// theta_error entre [-180 ,+180]
  if (fabs(theta_error) > M_PI)
  {
    theta_error -= signoDOSPI;
  }

  cout << endl;
  cout << "theta_error: " << gradRad * theta_error << " grad" << endl;

  if (fabs(theta_error) <= algPP.thetaErrorMinimo)
  {

    cout << endl;
    cout << "El robot no rota sobre sí mismo" << endl;
    cout << "| theta_error | <= " << gradRad * algPP.thetaErrorMinimo << " grad" << endl;
    return -1;
  }

  double pendiente_WbF_horaria = (-algPP.velAngBaseMax + algPP.velAngBaseMin)
      / (-algPP.thetaErrorMaximo + algPP.thetaErrorMinimo);
  double pendiente_WbF_antihoraria = (algPP.velAngBaseMax - algPP.velAngBaseMin)
      / (algPP.thetaErrorMaximo - algPP.thetaErrorMinimo);
  double WbF_rotacion = 0;

  if (theta_error < 0)
  {

    WbF_rotacion = pendiente_WbF_horaria * (theta_error + algPP.thetaErrorMinimo) - algPP.velAngBaseMin; // rad/s

  }
  else
  {

    WbF_rotacion = pendiente_WbF_antihoraria * (theta_error - algPP.thetaErrorMinimo) + algPP.velAngBaseMin; // rad/s

  }

  float VdF_rotacion = WbF_rotacion * algPP.radioBase; // cm/s

// En cts / ms con 1 solo decimal, como se puede observar en el programa Windows de la PMAC (PEWIN32)
  double VdF_rotacion_cts_ms = floor(10.0 * ((VdF_rotacion * cuentas_cm_recorrido) / 1000.0)) / 10.0;
// 4.297 cm/s ==> aprox 10 grad/s
// Quiero que tenga solo 5 decimales. Floorf para que no exceda la velocidad angular maxima.
  VdF_rotacion = static_cast<float>(floor(100000.0 * (1000.0 * VdF_rotacion_cts_ms / cuentas_cm_recorrido)) / 100000.0); // En cm/s
  int T_tramo_1 = static_cast<int>(ceil(1000.0 * (fabs(VdF_rotacion) / algPP.acelRuedaMaxRotacion))); // ms
  float arco_tramo_1 = VdF_rotacion * (T_tramo_1 / 1000.0f); // En cm, lleva signo
  float arco_total = static_cast<float>(algPP.radioBase * theta_error); // En cm
  float arco_tramo_2 = arco_total - (2.0f * arco_tramo_1); // En cm
  int T_tramo_2 = static_cast<int>(ceilf(1000.0f * (arco_tramo_2 / (2.0f * VdF_rotacion)))); // ms

// La PMAC trabaja con numeros reales de 32 bits, es decir, float.
  float comandosPDer[2] = {0.0f, 0.0f};
  float comandoVDer = 0.0f;
  int comandosT[2] = {0, 0};

  comandosT[0] = 2 * T_tramo_1;
  comandosT[1] = 2 * T_tramo_2;

  comandoVDer = VdF_rotacion;

  comandosPDer[0] = arco_tramo_1;
  comandosPDer[1] = arco_tramo_2;

  cout << "Acel_max_rotacion: " << algPP.acelRuedaMaxRotacion << " cm/s^2" << endl;
  cout << "VdF_rotacion: " << VdF_rotacion << " cm/s" << endl;
  cout << "T_tramo_1: " << T_tramo_1 << " ms" << endl;
  cout << "arco_tramo_1 " << arco_tramo_1 << " cm" << endl;
  cout << "arco_tramo_2 " << arco_tramo_2 << " cm" << endl;
  cout << "Theta_1: " << gradRad * arco_tramo_1 / algPP.radioBase << " grad " << endl;
  cout << "Theta_2: " << gradRad * arco_tramo_2 / algPP.radioBase << " grad " << endl;

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

  cout << "Aceleracion_max_real: " << fabs(comandoVDer) / ((comandosT[0] / 2) / 1000.0f) << " cm/s^2" << endl;

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
// se mueve se pone a 0. Cuando el robot se para se pone a 1.

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

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("2\n");

  if (!nh.hasParam("/RelacionReduccion"))
  {
    return -1;
  }
  nh.getParam("/RelacionReduccion", parametro);
  int relacion_reduccion = atoi(parametro.c_str());

// %%%%% %%%%% %%%%% %%%%% %%%%%  

  printf("3\n");

  if (!nh.hasParam("/CuentasEncoder"))
  {
    return -1;
  }
  nh.getParam("/CuentasEncoder", parametro);
  int cuentas_encoder = atoi(parametro.c_str());

// %%%%% %%%%% %%%%% %%%%% %%%%%  

  printf("4\n");

  if (!nh.hasParam("/FactorMultiplicativoCuentasPMAC"))
  {
    return -1;
  }
  nh.getParam("/FactorMultiplicativoCuentasPMAC", parametro);
  int factor_multiplicativo_cuentas_PMAC = atoi(parametro.c_str());

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("5\n");

  if (!nh.hasParam("/RadioRuedaM"))
  {
    return -1;
  }
  nh.getParam("/RadioRuedaM", parametro);
  algPP.radioRueda = 100.0 * atof(parametro.c_str()); // En cm

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("6\n");

  if (!nh.hasParam("/RadioBaseM"))
  {
    return -1;
  }
  nh.getParam("/RadioBaseM", parametro);
  algPP.radioBase = 100.0 * atof(parametro.c_str()); // En cm

// Radio a partir del que se considera que el robot se desplaza en siguiendo una
// trayectoria muy parecida a una línea recta.
  algPP.radioUmbralLinRecta = 10.0 * algPP.radioBase;
  algPP.radioConSignoFinal = 0.0;

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("7\n");

  if (!nh.hasParam("/VelocidadRuedaMaximaMpS"))
  {
    return -1;
  }
  nh.getParam("/VelocidadRuedaMaximaMpS", parametro);
  algPP.velRuedaMax = 100.0 * atof(parametro.c_str()); // En cm/s

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("8\n");

  if (!nh.hasParam("/VelocidadAngularBaseMaximaGpS"))
  {
//delete nombreMapa;
    return -1;
  }
  nh.getParam("/VelocidadAngularBaseMaximaGpS", parametro);
  algPP.velAngBaseMax = radGrad * atof(parametro.c_str()); //En rad/s

// %%%%% %%%%% %%%%% %%%%% %%%%%

  algPP.velRuedaMin = algPP.velAngBaseMax * algPP.radioBase; // cm/s

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("9\n");

  if (!nh.hasParam("/VelocidadAngularBaseMinimaGpS"))
  {
//delete nombreMapa;
    return -1;
  }
  nh.getParam("/VelocidadAngularBaseMinimaGpS", parametro);
  algPP.velAngBaseMin = radGrad * atof(parametro.c_str()); //En rad/s

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("10\n");

  if (!nh.hasParam("/ErrorOrientacionMinimoG"))
  {
    return -1;
  }
  nh.getParam("/ErrorOrientacionMinimoG", parametro);
  algPP.thetaErrorMinimo = radGrad * atof(parametro.c_str()); // En rad

// %%%%% %%%%% %%%%% %%%%% %%%%%

  algPP.thetaErrorMaximo = M_PIl;

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("11\n");

  if (!nh.hasParam("/AceleracionRuedaMaximaMpS2"))
  {
//delete nombreMapa;
    return -1;
  }
  nh.getParam("/AceleracionRuedaMaximaMpS2", parametro);
  algPP.acelRuedaMax = 100.0 * atof(parametro.c_str()); //En cm/s^2

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("12\n");

  if (!nh.hasParam("/AceleracionRuedaMaximaRotacionMpS2"))
  {
//delete nombreMapa;
    return -1;
  }
  nh.getParam("/AceleracionRuedaMaximaRotacionMpS2", parametro);
  algPP.acelRuedaMaxRotacion = 100.0 * atof(parametro.c_str()); //En cm/s^2

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("13\n");

  if (!nh.hasParam("/TiempoMaximoMediaCurvaSS"))
  {
//delete nombreMapa;
    return -1;
  }
  nh.getParam("/TiempoMaximoMediaCurvaSS", parametro);
  algPP.tiempoMaxCurvaS = 2 * static_cast<int>(ceil(1000.0 * atof(parametro.c_str()))); //En ms

  // %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("13\n");

  if (!nh.hasParam("/TiempoMinimoMediaCurvaSS"))
  {
    //delete nombreMapa;
    return -1;
  }
  nh.getParam("/TiempoMinimoMediaCurvaSS", parametro);
  int tiempo_min_media_CurvaS = static_cast<int>(ceil(1000.0 * atof(parametro.c_str()))); //En ms

// %%%%% %%%%% %%%%% %%%%% %%%%%

  printf("14\n");

  if (!nh.hasParam("/DeltaTiempoDesplazamientoS"))
  {
//delete nombreMapa;
    return -1;
  }
  nh.getParam("/DeltaTiempoDesplazamientoS", parametro);
// El tiempo de desplazamiento es DeltaTiempoDesplazamientoS ms superior al tiempo maximo que puede
// una curva S de velocidad
  algPP.tiempoDesplazamiento = algPP.tiempoMaxCurvaS + static_cast<int>(ceil(1000.0 * atof(parametro.c_str()))); //En ms

  algPP.velRuedaDerFinal = 0.0;
  algPP.velRuedaIzqFinal = 0.0;
  algPP.tiempoMediaCurvaSRuedaDer = 0.0;
  algPP.tiempoMediaCurvaSRuedaIzq = 0.0;

//int numeroLocalizacionesOdometria = 0;

  double x_link_base_ini = 0.0;
  double y_link_base_ini = 0.0;
  double z_link_base_ini = 0.0;
  double t_link_base_ini = 0.0;

  double x_link_base_ant = 0.0;
  double y_link_base_ant = 0.0;
  double z_link_base_ant = 0.0;
  double t_link_base_ant = 0.0;

  double x_link_base = 0.0;
  double y_link_base = 0.0;
  double z_link_base = 0.0;
  double t_link_base = 0.0;

//double xRobotSROAnt = 0.0f;
//double yRobotSROAnt = 0.0f;
//double tRobotSROAnt = 0.0f;

//double xRobotSRO = 0.0f;
//double yRobotSRO = 0.0f;
//double tRobotSRO = 0.0f;

  double x_goal = 6.05;
  double y_goal = 20.45;

  double L_ah_cm = 2.0 * algPP.radioBase;
  double thetaError = 0.0;

// Recibir el mapa y obtener el tamanio de la celda en M.
  do
  {
    ros::spinOnce();
  } while (!flag_mapa_recibido);

  double diametroBase = 2.0 * algPP.radioBase; // En cm

  double lado = 1.25 * 0.01 * algPP.velRuedaMax * (algPP.tiempoDesplazamiento / 1000.0); // 25 % mayor de lo estrictamente necesario
  double xInferiorAreaReposo = x_goal - lado;
  double xSuperiorAreaReposo = x_goal + lado;
  double yInferiorAreaReposo = y_goal - lado;
  double ySuperiorAreaReposo = y_goal + lado;

// 6 decimales. Como en la definicion usada en el fichero de configuracion de la PMAC.
  double cuentas_revolucion = factor_multiplicativo_cuentas_PMAC * relacion_reduccion * cuentas_encoder;
  double cuentas_cm_recorrido = cuentas_revolucion / (2.0 * M_PI * algPP.radioRueda);

  double pendiente_VbF = (algPP.velRuedaMax - algPP.velRuedaMin) / (9.0 * algPP.radioBase); // 1/s

  double deltaVmax = (algPP.tiempoMaxCurvaS / 2000.0) * algPP.acelRuedaMax;
  double deltaVmin = (tiempo_min_media_CurvaS / 2000.0) * algPP.acelRuedaMax;

  double VbF = 0.0;
  double theta_error = 0.0;
  // Si theta_error = 0  ==>  sin(theta_error) = 0 EN EL DENOMINADOR DE RS!!
  double theta_error_min = M_PIl * (0.1 / 180.0);  // 0.1 grad
  double distancia_destino = 0.0;

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
  cout << "V ruedas min: " << algPP.velRuedaMin << " cm/s" << endl;
  cout << "V angular max: " << gradRad * algPP.velAngBaseMax << " grad/s" << endl;
  cout << "V angular min: " << gradRad * algPP.velAngBaseMin << " grad/s" << endl;
  cout << "Theta error maximo: " << gradRad * algPP.thetaErrorMaximo << " grad" << endl;
  cout << "Theta error minimo: " << gradRad * algPP.thetaErrorMinimo << " grad" << endl;
  cout << "A ruedas max: " << algPP.acelRuedaMax << " cm/s^2" << endl;
  cout << "A ruedas max rotación: " << algPP.acelRuedaMaxRotacion << " cm/s^2" << endl;
  cout << "T curva S maximo: " << algPP.tiempoMaxCurvaS << " ms" << endl;
  cout << "T desplazamiento: " << algPP.tiempoDesplazamiento << " ms" << endl;

  cout << "L_ah: " << L_ah_cm << " cm" << endl;

  cout << "deltaVmax: " << deltaVmax << " cm/s" << endl;
  cout << "deltaVmin: " << deltaVmin << " cm/s" << endl;
  cout << "pendiente_VbF: " << pendiente_VbF << " 1/s" << endl;

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

// Esperar a que el robot se haya localizado y publique la ubicación del marco
// de referencia link_base. A partir de esta ubicación, usando la librería
// tf se obtiene la ubicación del marco de referencia link_base_desp
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

      z_link_base_ini = stampedTf.getOrigin().z(); // m
      t_link_base_ini = tf::getYaw(stampedTf.getRotation()); // rad

    }
    catch (tf::TransformException& ex)
    {

      cout << "\x1B[31;1m";
      cout << ex.what() << endl;
      cout << "\x1B[0m";
    }

    sleep(1);
  } // fin while

  transformacion_escuchada = false;

// Distancia de separación entre los marcos de referencia link_base y link_base_desp
  double dist_separacion = 0.0;
  double orientacion_marco_ref_paralelo = 0.0;

  while (!transformacion_escuchada)
  {

    marca_tiempo = ros::Time::now();

    try
    {

      tfListener.waitForTransform("/link_base_desp", "/link_base", marca_tiempo, ros::Duration(1.0));
      tfListener.lookupTransform("/link_base_desp", "/link_base", marca_tiempo, stampedTf);
      transformacion_escuchada = true;
      // Sé que posición ocupa el marco de referencia link_base respecto a link_base_desp, por eso sé
      // que la distancia de separación entre ambos es la coordena y de la transformación obtenida.
      dist_separacion = stampedTf.getOrigin().y(); // m
      // Esto debería dar pi/2 (90 grad).
      orientacion_marco_ref_paralelo = atan2(stampedTf.getOrigin().y(), stampedTf.getOrigin().x()); // rad

#ifdef DEBUG
      cout << "dist_separacion: " << 100 * dist_separacion << " cm" << endl;
      cout << "orientacion_marco_ref_paralelo: " << gradRad * orientacion_marco_ref_paralelo << " grad" << endl;
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

  mr::Path2D trayectoria_link_base_desp = fm2pathplanner->getPath();
  std::vector<mr::Vector2D> trayectoria_link_base;
  crear_trayectoria_paralela(trayectoria_link_base_desp, trayectoria_link_base, dist_separacion,
                             orientacion_marco_ref_paralelo);

  nav_msgs::Path ruta_link_base_dep;
  ruta_link_base_dep.header.frame_id = "/map";
  ruta_link_base_dep.header.stamp = ros::Time::now();
  ruta_link_base_dep.poses.resize(trayectoria_link_base_desp.points.size());

  nav_msgs::Path ruta_link_base;
  ruta_link_base.header.frame_id = "/map";
  ruta_link_base.header.stamp = ruta_link_base_dep.header.stamp;
  ruta_link_base.poses.resize(trayectoria_link_base.size());

  for (int i = 0; i < trayectoria_link_base_desp.points.size(); i++)
  {

    ruta_link_base_dep.poses[i].header.frame_id = "/map";
    ruta_link_base_dep.poses[i].header.stamp = ruta_link_base_dep.header.stamp;
    ruta_link_base_dep.poses[i].pose.position.x = trayectoria_link_base_desp.points[i].x;
    ruta_link_base_dep.poses[i].pose.position.y = trayectoria_link_base_desp.points[i].y;
    ruta_link_base_dep.poses[i].pose.position.z = z_link_base;

    ruta_link_base.poses[i].header.frame_id = "/map";
    ruta_link_base.poses[i].header.stamp = ruta_link_base.header.stamp;
    ruta_link_base.poses[i].pose.position.x = trayectoria_link_base[i].x;
    ruta_link_base.poses[i].pose.position.y = trayectoria_link_base[i].y;
    ruta_link_base.poses[i].pose.position.z = z_link_base;
  }

  ros::Publisher publicadorTopicPathLBD = nh.advertise<nav_msgs::Path>("topic_ruta_link_base_desp", 1, true);
  publicadorTopicPathLBD.publish(ruta_link_base_dep);

  ros::Publisher publicadorTopicPathLB = nh.advertise<nav_msgs::Path>("topic_ruta_link_base", 1, true);
  publicadorTopicPathLB.publish(ruta_link_base);

  x_link_base_ini = trayectoria_link_base[0].x; // m
  x_link_base_ant = x_link_base_ini;
  x_link_base = x_link_base_ini;

  y_link_base_ini = trayectoria_link_base[0].y; // m
  y_link_base_ant = y_link_base_ini;
  y_link_base = y_link_base_ini;

  z_link_base_ant = z_link_base_ini; // m
  z_link_base = z_link_base_ini;

  t_link_base_ant = t_link_base_ini; // rad
  t_link_base = t_link_base_ini;

  /*
   tiempoDesplazamiento en ms
   frec = 1 /tiempoDesplazamientoSeg o 1000/tiempoDesplazamientoMSeg
   Si hago que la frecuencia de muestreo de la ubicacion del robot, es decir, la frecuencia a la que
   se van a dar acciones de control sea menor que este valor, es equivalente a permitir que el
   robot se desplace un poco más de tiempo con la accion de control anterior.
   Para hacer que la frec sea un poco menor de lo estricamente necesaria uso 'floorf'.
   */

//float frec = floorf(1000 / algPP.tiempoDesplazamiento);
//float frec = 0.5; // Hz
//ros::Rate tasa(frec);
#ifdef DEBUG

  cout << endl;
  cout << "link_base en MMapa" << endl;
  cout << "(" << x_link_base << " m,  " << y_link_base << " m, " << z_link_base << " m, " << gradRad * t_link_base
      << " grad)" << endl;
//cout << "x_link_base: " << x_link_base << " m " << endl;
//cout << "y_link_base: " << y_link_base << " m" << endl;
//cout << "z_link_base: " << z_link_base << " m " << endl;
//cout << "t_link_base: " << gradRad * t_link_base << " grad " << endl;
//	cout << "Frecuencia refresco: " << frec << " Hz" << endl;

#endif

  char teclaParaContinuar = '0';
  std::cout << "Introduce una tecla para continuar: ";
  std::cin >> teclaParaContinuar;

  rotacionInicial(descriptorDPRAM, trayectoria_link_base, t_link_base, algPP, cuentas_cm_recorrido);

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
      x_link_base = stampedTf.getOrigin().x();
      y_link_base = stampedTf.getOrigin().y();
      z_link_base = stampedTf.getOrigin().z();
      t_link_base = tf::getYaw(stampedTf.getRotation());

#ifdef DEBUG
      cout << endl;
      cout << "link_base en MMapa" << endl;
      cout << "(" << x_link_base << " m,  " << y_link_base << " m, " << gradRad * t_link_base << " grad)" << endl;
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
    x_link_base = xInferiorAreaReposo;
    y_link_base = yInferiorAreaReposo;
  }

  std::cout << "Introduce una tecla para continuar: ";
  std::cin >> teclaParaContinuar;
//char temp = 0;
//scanf("%c", &temp);
  std::cout << std::endl;

  ros::Time timeInicio;

// El robot se encuentra en el indice 0 de trayectoria_link_base.
  int indice_destino = 1;
  int indice_final = trayectoria_link_base.size() - 1;
  int flag_para_emergencia = 0;

  /*while (x_link_base < xInferiorAreaReposo
   || x_link_base > xSuperiorAreaReposo
   || y_link_base < yInferiorAreaReposo
   || y_link_base > ySuperiorAreaReposo)*/

  while (VbF != -1)
  {

    theta_error = 0.0;
    distancia_destino = 0.0;

    buscarPuntoDestino(trayectoria_link_base, indice_destino, x_link_base, y_link_base, t_link_base, L_ah_cm,
                       theta_error, distancia_destino);

// SITUACIÓN NORMAL: Si -90 grad <= theta_error <= 90 grad desplazamiento de frente + giro
// (horario o antihorario).
// NO ESTA PERMITIDO theta_error < -90º
// NO ESTA PERMITIDO theta_error > +90º
// Si 'theta_error < -90 grad' o 'theta_error > 90 grad' ejecutar rotacion horaria o
// antihoraria.

    // Esta condicion se pone porque sing(x) = 0 si x = 0;
    // Por defecto giro míiiinimo antihorario.
    //if (theta_error == 0) {

    //	theta_error = theta_error_min; //
    //}
// theta_error muy pequeño pero no es 0.
//		else
//		if (theta_error > -theta_error_min && theta_error < theta_error_min) {
//			theta_error = sign(theta_error) * theta_error_min;

//		} else if (theta_error < -theta_error_max
//				|| theta_error > theta_error_max) {

//TODO: Rotacion horaria u antihoraria.
//		}

// Para evitar la división por cero sumo 0.1 grados (en radianes).
    algPP.radioConSignoFinal = distancia_destino / (2 * sin(thetaError + theta_error_min));

    if (fabs(algPP.radioConSignoFinal) < algPP.radioBase)
    {

      algPP.radioConSignoFinal = sign(algPP.radioConSignoFinal) * algPP.radioBase;

#ifdef DEBUG
      cout << "\tRsF: " << algPP.radioConSignoFinal << " cm" << endl;
#endif
    }

    VbF = pendiente_VbF * (fabs(algPP.radioConSignoFinal) - algPP.radioBase) + algPP.velRuedaMin; // cm/s

    if (fabs(algPP.radioConSignoFinal) > algPP.radioUmbralLinRecta)
    {

      VbF = algPP.velRuedaMax;

    }

    if (indice_destino >= indice_final)
    {

      break; // se rompe el bucle while para que el robot se pare.

    }
    else
    {

      actualizarVelFinalRuedas(descriptorDPRAM, algPP, VbF, deltaVmax, deltaVmin, cuentas_cm_recorrido);
    }

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

    x_link_base_ant = x_link_base;
    y_link_base_ant = y_link_base;
    t_link_base_ant = t_link_base;

    try
    {

      if (!tfListener.waitForTransform("/map", "/link_base", marca_tiempo, ros::Duration(1.0)))
      {
        throw tf::TransformException("La transformacion no llego a tiempo");
      }
      else
      {
        tfListener.lookupTransform("/map", "/link_base", marca_tiempo, stampedTf);
        x_link_base = stampedTf.getOrigin().x();
        y_link_base = stampedTf.getOrigin().y();
        z_link_base = stampedTf.getOrigin().z();
        t_link_base = tf::getYaw(stampedTf.getRotation());

#ifdef DEBUG
        cout << endl;
        cout << "(" << x_link_base << " m,  " << y_link_base << " m, " << gradRad * t_link_base << " grad)" << endl;
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
      theta_error = 0; // Condición para que el robot se pare en seco.
      break;
    }
  }

  pararRobot(descriptorDPRAM, algPP, theta_error, cuentas_cm_recorrido);

//}

//delete nombreMapa;
  delete img;
  delete gridmap;
  delete fm2pathplanner;
  delete p_init;
  delete p_goal;
  return 0;
}
