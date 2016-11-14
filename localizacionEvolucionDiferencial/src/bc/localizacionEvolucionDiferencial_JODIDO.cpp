#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <Magick++.h>
#include <magick/MagickCore.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

#include "sicks3000/servicioSicks3000.h"
#include "odometria/servicioOdometria.h"

#include "localizacionEvolucionDiferencial/localizacionEvolucionDiferencial.h"

// act --> actual
// ant --> anterior
// c   --> columna
// f   --> fila
// g   --> global
// h   --> hipotenusa
// km1 --> k + 1
// l   --> local
// o   --> odometria
// p   --> posicion
// pst --> posterior
// t   --> theta
// rr  --> robot real
// rv  --> robot virtual

//extern unsigned int num_loc;

using std::cin;
using std::cout;
using std::endl;

// Funcion necesaria para la funcion qsort.
int cmp(const void* a, const void* b)
{

  const double* ptr_1 = *(const double**)a;
  const double* ptr_2 = *(const double**)b;
  if (*ptr_1 < *ptr_2)
  {
    return -1;
  }
  if (*ptr_1 == *ptr_2)
  {
    return 0;
  }
  return 1;
}

LocED::LocED(const float& lado_celda_m, const unsigned int& d, const unsigned int& st, const float& cr, const float& f,
             const float& t_c, const float& dist_conv_m, const unsigned int& num_elem_pob_norm,
             const unsigned int& num_elem_pob_conv, const unsigned int& num_iter_norm,
             const unsigned int& num_iter_conv, const MapaLoc& mapa_a,

#ifdef DOS_MAPAS

             const MapaLoc& mapa_b,

#endif

             const Ubicacion& ub_min_regint_g,
             const Ubicacion& ub_max_regint_g, const float& alcance_laser_m, const double& angulo_max,
             const double& res_angular, const unsigned int& numMedLaser, const float& dist_3sigma_m,
             const unsigned int& coef_incr, const unsigned int& coef_incr_conv) :
    ladoCeldaM(lado_celda_m), numero_componentes_(d), estrategia_mutacion_(st), factor_cruce_(cr), cF(f), dist_traslacion_C_(t_c), tM(dist_traslacion_C_ * ladoCeldaM), dist_convergencia_C_(
        dist_conv_m / ladoCeldaM), bandera_loc_global_exito_(0), num_elem_pob_norm_(num_elem_pob_norm), num_elem_pob_conv_(
        num_elem_pob_conv), num_elem_pob_act_(num_elem_pob_norm_), num_iter_norm_(num_iter_norm), num_iter_conv_(
        num_iter_conv), num_iter_act_(num_iter_norm_), mapa_a_(mapa_a),

#ifdef DOS_MAPAS

        mapa_b_(mapa_b),

#endif

        ub_min_regint_g_(ub_min_regint_g), ub_max_regint_g_(ub_max_regint_g), aptitudUbEstSrg_(0), numLocSinExito_(0), alcance_laser_M_(
            alcance_laser_m), alcance_laser_C_(alcance_laser_M_ / ladoCeldaM), cAnguloMax(scRadGrad * angulo_max), cResAngular(
            scRadGrad * res_angular), cNumMedLaser(numMedLaser), cDist3SigmaM(dist_3sigma_m),
        //cCoefNumMedLaser(coef_incr),
        cNumMedLaserNorm(2 * ((unsigned int)(angulo_max / (coef_incr * res_angular))) + 1), cNumMedLaserConv(
            2 * ((unsigned int)(angulo_max / (coef_incr_conv * res_angular))) + 1), num_med_laser_act_(
            cNumMedLaserNorm), cAptitudLimSup(scCoefAptitudLimSup * (2 * cDist3SigmaM * num_med_laser_act_)), coef_hipotenusa_(
            1.0 * ladoCeldaM / cDist3SigmaM), coef_angulo_(10 * ladoCeldaM / cDist3SigmaM), intervaloTiempoSegLocSinExito_(
            0)
{

  // cAptitudLimSup = 2 * ((2 * dist_3sigma_m) * num_med_laser_act_);
  // double dist_3sigma_c = dist_3sigma_m / cLadoCeldaM;
  // double inv_dist_3sigma_c = 1.0 / dist_3sigma_c;
  // cCoefHipo = 1.0 * inv_dist_3sigma_c;
  // cCoefAng = 10.0 * inv_dist_3sigma_c;

  double angulo = -cAnguloMax;

  ang_.assign(cNumMedLaser, 0);

  for (unsigned int i = 0; i < cNumMedLaser; ++i)
  {
    ang_[i] = angulo;
    //cout << i << " " << ang_[i] << " " << scGradRad * ang_[i] << endl;
    angulo += cResAngular;
  }

  printf("cLadoCeldaM: %f.\n", ladoCeldaM);
  printf("cD: %d.\n", numero_componentes_);
  printf("cST: %d.\n", estrategia_mutacion_);
  printf("cCR: %f.\n", factor_cruce_);
  printf("cF: %f.\n", cF);
  printf("cTC: %f.\n", dist_traslacion_C_);
  printf("cTM: %f.\n", tM);
  printf("cDistConvC: %f.\n", dist_convergencia_C_);
  printf("cCoefHipo: %.8f.\n", coef_hipotenusa_);
  printf("cCoefAng: %.8f.\n", coef_angulo_);
  printf("scUmbralOcupacion: %f.\n", umbral_ocupacion_);
  printf("banderaLocConExito_: %d.\n", bandera_loc_global_exito_);
  printf("cNumElemPobNorm: %d.\n", num_elem_pob_norm_);
  printf("cNumElemPobConv: %d.\n", num_elem_pob_conv_);
  printf("num_elem_pob_act_: %d.\n", num_elem_pob_act_);
  printf("cNumIterNorm: %d.\n", num_iter_norm_);
  printf("cNumIterConv: %d.\n", num_iter_conv_);
  printf("num_iter_act_: %d.\n", num_iter_act_);
  printf("\n");
  printf("Fichero mapa a: %s.\n", mapa_a_.getNombreFichero().c_str());

#ifdef DOS_MAPAS

  printf("Fichero mapa b: %s.\n", mapa_b_.getNombreFichero().c_str());

#endif

  printf("\n");
  printf("Numero filas mapa: %d.\n", mapa_a_.getFilas());
  printf("Numero columnas mapa: %d.\n", mapa_a_.getColumnas());
  printf("\n");
  printf("x_min_regint_g: %.8f.\n", ub_min_regint_g_.getX());
  printf("y_min_regint_g: %.8f.\n", ub_min_regint_g_.getY());
  printf("t_min_regint_g: %.8f.\n", scGradRad * ub_min_regint_g_.getTheta());
  printf("\n");
  printf("x_max_regint_g: %.8f.\n", ub_max_regint_g_.getX());
  printf("y_max_regint_g: %.8f.\n", ub_max_regint_g_.getY());
  printf("t_max_regint_g: %.8f.\n", scGradRad * ub_max_regint_g_.getTheta());
  printf("\n");
  printf("x_est_busq_g: %.8f.\n", ubEstSrg_.getX());
  printf("y_est_busq_g: %.8f.\n", ubEstSrg_.getY());
  printf("t_est_busq_g: %.8f.\n", scGradRad * ubEstSrg_.getTheta());
  printf("aptitudUbEstSrg_: %f.\n", aptitudUbEstSrg_);
  printf("\n");
  printf("Tamanio poblacion_: %d.\n", poblacion_.size());
  printf("Tamanio pobInicioLoc_: %d.\n", pobInicioLoc_.size());
  printf("Tamanio coste_pob_: %d.\n", coste_pob_.size());
  printf("\n");
  printf("cAptitudLimSup: %.8f.\n", cAptitudLimSup);
  printf("\n");
  printf("cAlcanceLaserM: %f.\n", alcance_laser_M_);
  printf("cAlcanceLaserC: %f.\n", alcance_laser_C_);
  printf("cAnguloMax (RAD): %.8f.\n", cAnguloMax);
  printf("cAnguloMax (GRAD): %.8f.\n", scGradRad * cAnguloMax);
  printf("cResAngular (RAD): %.8f.\n", cResAngular);
  printf("cResAngular (GRAD): %.8f.\n", scGradRad * cResAngular);
  printf("cNumMedLaser: %d.\n", cNumMedLaser);
  printf("\n");
  //printf("cCoefNumMedLaser: %d.\n", cCoefNumMedLaser);
  printf("cNumMedLaserNorm: %d.\n", cNumMedLaserNorm);
  printf("cNumMedLaserConv: %d.\n", cNumMedLaserConv);
  printf("num_med_laser_act_: %d.\n", num_med_laser_act_);
}

void LocED::actualizarPoblacion(const double & intervaloTiempoLocSeg, const Ubicacion& ub_ant_rr_o,
                                const Ubicacion& ub_act_rr_o, const std::vector<double>& dist_laser,
                                const std::vector<double>& distLaserAnterior)
{

  cout << endl;
  cout << "DESPLAZAR LA POBLACION Y COMPROBAR LA CONVERGENCIA" << endl;

  // Coordenadas del individuo seleccionado.
  double x_act_g = 0;
  double y_act_g = 0;
  double t_act_g = 0;

  // Region donde se esparce la poblacion;
  double x_min_g = 0;
  double y_min_g = 0;
  double t_min_g = 0;
  poblacion_[0].getXYTheta(x_min_g, y_min_g, t_min_g);
  double x_max_g = x_min_g;
  double y_max_g = y_min_g;
  double t_max_g = t_min_g;

  double dosPi = 2 * M_PI;

  // Aumento del 25% del tamaño del lado: (1+0'25)*(0'5*cDistConvC) = 1'25*0'5*cDistConvC = 0'625*cDistConvC;
  //float mitad_lado_reg_conv = 0.5 * cDistConvC;
  //float coef_incr_lado = (1 + 0.25);
  //float mitad_lado_reg_int = coef_incr_lado * mitad_lado_reg_conv;

  // Averiguar si los individuos de la poblacion se han cocentrado en el
  // interior de una zona cuadrada de 'cDistConvC' celdas de lado.
  for (unsigned int i = 1; i < num_elem_pob_act_; ++i)
  {
    poblacion_[i].getXYTheta(x_act_g, y_act_g, t_act_g);
    if (x_act_g < x_min_g)
    {
      x_min_g = x_act_g;
    }
    else if (x_act_g > x_max_g)
    {
      x_max_g = x_act_g;
    }
    if (y_act_g < y_min_g)
    {
      y_min_g = y_act_g;
    }
    else if (y_act_g > y_max_g)
    {
      y_max_g = y_act_g;
    }
    if (t_act_g < t_min_g)
    {
      t_min_g = t_act_g;
    }
    else if (t_act_g > t_max_g)
    {
      t_max_g = t_act_g;
    }
  }
  double incrX = x_max_g - x_min_g;
  double incrY = y_max_g - y_min_g;
  // Ubicacion estimada buena y poblacion concentrada en un cuadrado de 'cDistConvC' celdas de lado:
  // ALGORITMO ALCANZA LA CONVERGENCIA.
  if (aptitudUbEstSrg_ < cAptitudLimSup && incrX < dist_convergencia_C_ && incrY < dist_convergencia_C_)
  {
    bandera_loc_global_exito_ = 1;
    numLocSinExito_ = 0;
    intervaloTiempoSegLocSinExito_ = 0;
    //estadoFinal_ = 1;

    /*double t_max_g_2 = fmod(t_max_g, dosPi);
     double t_min_g_2 = fmod(t_min_g, dosPi);

     if (fabs(t_max_g_2) > M_PI)
     {
     // Ejemplo: angulo = -869 --> fmod(-869, 360) = -149 --> +360 --> 211
     t_max_g_2 += (-sign(t_max_g_2) * dosPi);
     }
     if (fabs(t_min_g_2) > M_PI)
     {
     // Ejemplo: angulo = -869 --> fmod(-869, 360) = -149 --> +360 --> 211
     t_min_g_2 += (-sign(t_min_g_2) * dosPi);
     }
     double incrT = fmod((t_max_g_2 - t_min_g_2), dosPi);
     if (fabs(incrT) > M_PI)
     {
     incrT += (-sign(incrT) * dosPi);
     }
     */

    double t_max_g_2 = t_max_g;
    Ubicacion::orientacionMenosPiMasPi(t_max_g_2);
    double t_min_g_2 = t_min_g;
    Ubicacion::orientacionMenosPiMasPi(t_min_g_2);
    double incrT = t_max_g_2 - t_min_g_2;
    Ubicacion::orientacionMenosPiMasPi(incrT);

    cout << "\x1B[31;1m";
    cout << endl;
    cout << "CONVERGENCIA ALCANZADA" << endl;
    cout << "(xMinSrg, xMaxSrg): (" << x_min_g << ", " << x_max_g << ")" << endl;
    cout << "(yMinSrg, yMaxSrg): (" << y_min_g << ", " << y_max_g << ")" << endl;
    cout << "(tMinSrg, tMaxSrg): (" << scGradRad * t_min_g_2 << "grad, " << scGradRad * t_max_g_2 << " grad)" << endl;
    cout << endl;
    cout << "incrX: " << incrX << endl;
    cout << "incrY: " << incrY << endl;
    cout << "incrT: " << scGradRad * incrT << " grad" << endl;
    cout << "\x1B[0m";
    cout << "num_elem_pob_act_: " << num_elem_pob_act_ << endl;
    cout << "poblacion_.size(): " << poblacion_.size() << endl;

    num_elem_pob_act_ = num_elem_pob_conv_;
    num_iter_act_ = num_iter_conv_;
    num_med_laser_act_ = cNumMedLaserConv;
    poblacion_.resize(num_elem_pob_act_);

    cout << "num_elem_pob_act_: " << num_elem_pob_act_ << endl;
    cout << "poblacion_.size(): " << poblacion_.size() << endl;
    cout << "poblacion_.size(): " << poblacion_.size() << endl;

    desplazarPoblacion(ub_ant_rr_o, ub_act_rr_o, dist_laser);

    //ub_pred_movil_g_ = poblacion_[0];
    // Region de interes cuadrada centrada en el individuo predicho.
    //ub_min_regint_g_.setXYTheta(ubPreSrg_.getX() - mitad_lado_reg_int, ubPreSrg_.getY() - mitad_lado_reg_int, 0.0);
    //ub_max_regint_g_.setXYTheta(ubPreSrg_.getX() + mitad_lado_reg_int, ubPreSrg_.getY() + mitad_lado_reg_int, dos_pi);
    //ComprobarLimitesRegionInteres();
  }
  // Ubicacion estimada no buena y poblacion no concentrada en un cuadrado de 'cDistConvC' celdas de lado.
  // Ubicacion estimada si buena y poblacion no concentrada en un cuadrado de 'cDistConvC' celdas de lado.
  else
  {
    cout << endl;
    cout << "\x1B[31;1m";
    cout << "EL ALGORITMO NO ALCANZA LA CONVERGENCIA." << endl;
    cout << "Aptitud individuo estimado: " << aptitudUbEstSrg_ << endl;
    cout << "incrX: " << incrX << "incrY: " << incrY << endl;
    cout << "\x1B[0m";
    cout << "num_elem_pob_act_: " << num_elem_pob_act_ << endl;
    cout << "poblacion_.size(): " << poblacion_.size() << endl;
    // El algoritmo alcanzo la convergencia en UN PASO ANTERIOR (UNO CUALQUIERA, NO EL INMEDIATAMENTE
    // ANTERIOR A ESTE).
    if (bandera_loc_global_exito_)
    {
      intervaloTiempoSegLocSinExito_ += intervaloTiempoLocSeg;
      // Si el algoritmo ha alcanzado la convergencia anteriormente me fio de la prediccion,
      // aunque en esta localizacion el algoritmo no haya alcanzado la convergencia.
      ubEstSrg_ = ubPreSrg_;
      // El algoritmo lleva menos de 5 segundos sin localizar con exito al robot.
      if (intervaloTiempoSegLocSinExito_ < 5)
      {
        cout << "FIARSE DE LA PREDICCION" << endl;
        poblacion_.assign(pobInicioLoc_.begin(), pobInicioLoc_.end());
        desplazarPoblacion(ub_ant_rr_o, ub_act_rr_o, dist_laser);
      }
      // El algoritmo lleva mas de 5 segundos sin localizar con exito al robot, localizacion global alrededor
      // de la prediccion en un area considerable.
      else
      {
        cout << "DEMASIADA CONFIANZA EN LA PREDICCION. LOC EXHAUSTA ALREDEDOR DE LA PREDICCION" << endl;
        double ladoRegInt = intervaloTiempoSegLocSinExito_ * 20;
        ub_min_regint_g_.setXYTheta(ubPreSrg_.getX() - ladoRegInt, ubPreSrg_.getY() - ladoRegInt, 0);
        ub_max_regint_g_.setXYTheta(ubPreSrg_.getX() + ladoRegInt, ubPreSrg_.getY() + ladoRegInt, dosPi);
        comprobarLimitesRegionInteres();
        bandera_loc_global_exito_ = 0;
        num_elem_pob_act_ = num_elem_pob_norm_;
        num_iter_act_ = num_iter_norm_;
        num_med_laser_act_ = cNumMedLaserNorm;
        poblacion_.clear();
        coste_pob_.clear();
        crearPoblacion(dist_laser, num_elem_pob_norm_);
        ub_min_regint_g_.setXYTheta(0, 0, 0);
        ub_max_regint_g_.setXYTheta(mapa_a_.getColumnas(), mapa_a_.getFilas() - scRound, dosPi);
      }
    }
    else
    {
      cout << "LOC GLOBAL EN TODO EL MAPA" << endl;
      numLocSinExito_++;
      // Nueva localizacion global en to_do el mapa.
      if (numLocSinExito_ < 3)
      {
        desplazarPoblacion(ub_ant_rr_o, ub_act_rr_o, dist_laser);
      }
      // Localizacion global en to_do el mapa.
      else
      {
        printf("\x1B[31;1m");
        cout << "RE-LOC GLOBAL EN TODO EL MAPA" << endl;
        printf("\x1B[0m");
        poblacion_.clear();
        coste_pob_.clear();
        crearPoblacion(dist_laser, num_elem_pob_norm_);
      }
      cout << "num_elem_pob_act_: " << num_elem_pob_act_ << endl;
      cout << "poblacion_.size(): " << poblacion_.size() << endl;
    }
  }
  // Al finalizar el proceso de localizacion 'poblacion_[0]' es la ubicacion estimada del robot.
  // Esa ubicacion se desplaza de acuerdo a la odometria y genera la prediccion de la ubicacion del robot.
  ubPreSrg_ = poblacion_[0];
  pobInicioLoc_.assign(poblacion_.begin(), poblacion_.end());
}

double LocED::trazadoRayos(const MapaLoc& mapa, const Ubicacion& ub_g, const double& incr_x_ex_g,
                           const double& incr_y_ex_g, const double& dist_inic)
{
// Coordenadas 'enteras' de la posicion alcanzada.
  int c_x_lleg_g = 0;
  int f_y_lleg_g = 0;

// Probabilidad de que la ubicacion alcanzada este libre en el mapa pasado como argumento.
  double prob_p_libre = 0.0;

// Distancia desde la 'posicion' origen hasta el primer obstaculo encontrado en la
// orientacion de exploracion 't_ex_g'.
// Se usa como valor de inicializacion la distancia pasada como argumento.
  double dist = dist_inic;

// Coordenadas maximas del entorno donde se realiza la localizacion.
  double x_max_mapa = mapa.getColumnas() - scRound;
  double y_max_mapa = mapa.getFilas() - scRound;

// Coordenadas minimas del entorno donde se realiza la localizacion.
  double x_min_mapa = 0;
  double y_min_mapa = 0;

// Coordenadas de la posicion alcanzada.
  double x_lleg_g = 0;
  double y_lleg_g = 0;
// Orientacion con la que se explora.
  double t_ex_g = 0;
  ub_g.getXYTheta(x_lleg_g, y_lleg_g, t_ex_g);

// cout << endl;
// cout << "CDO -- UB_G: (" << x_g << ", " << y_g << ", " << scGradRad * ub_g.getTheta() << ")" << endl;
// cout << "CDO -- UB_LLEG_G: (" << x_lleg_g << ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;
// cout << "CDO -- INC_X_EX_G: " << incr_x_ex_g << endl ;
// cout << "CDO -- INC_Y_EX_G: " << incr_y_ex_g << endl;

// Explorar con una determinada orientacion, 't_ex_g', hasta que:
// 1.- Se encuentre una posicion ocupada por un obstaculo.
// 2.- Se recorra una distancia igual al alcance maximo medible por el laser.
// 3.- Se excedan los limites del mapa.
  do
  {

    // La ubicacion de partida, 'ub_g', no puede estar ocupada por un obstaculo.
    // De este hecho ya me he asegurado fuera de esta funcion. Por este motivo, la primera
    // ubicacion que puedo evaluar en la orientacion 't_ex_g' es
    x_lleg_g += incr_x_ex_g;
    y_lleg_g += incr_y_ex_g;
    dist += tM;
    //cout << endl;
    //cout <<"CDO2 -- UB_LLEG_G: (" x_lleg_g << ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;

    // Comprobar si la ubicacion alcanzada esta fuera de los limites del mapa.
    if (x_lleg_g < x_min_mapa || x_lleg_g >= x_max_mapa || y_lleg_g < y_min_mapa || y_lleg_g >= y_max_mapa)
    {
      //cout << endl;
      //cout << "CDO -- UB_LLEG_G FUERA MAPA" << endl;

      // La ubicacion alcanzada esta fuera de los limites del mapa y por lo tanto se recupera la ubicacion
      // anterior, que SI que estaba dentro del mapa, porque esta ubicacion fue evaluada en la iteracion
      // anterior.
      x_lleg_g -= incr_x_ex_g;
      y_lleg_g -= incr_y_ex_g;
      dist -= tM;
      //cout << "CDO -- UB_LLEG_G: (" x_lleg_g ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;

      // Marco la ubicacion alcanzada como ocupada y de este modo cuando la instruccion 'continue'
      // interrumpa la iteracion actual del bucle, y me lleve a la expresion
      // while(prob_p_libre > umbral_p_ocupada), este se detenga porque la condicion
      // no se satisface.
      prob_p_libre = 0;
      continue;
    }

    // Averiguar si la posicion de llegada esta ocupada por un obstaculo.
    roundLoc(x_lleg_g, y_lleg_g, c_x_lleg_g, f_y_lleg_g);
    prob_p_libre = mapa.obtenerProbabilidadOcupacion(c_x_lleg_g, f_y_lleg_g);

    // cout << "CDO -- UB_LLEG_G (ROUND): (" << c_x_lleg_g << ", " << f_y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;
    // cout << "CDO -- PROB: " << prob_p_libre << endl;

  } while (dist < alcance_laser_M_ && prob_p_libre > umbral_ocupacion_);

// cout << endl;
// cout << "CDO -- UB_G: (" << x_g << ", " << y_g << ", " << scGradRad * t_ex_g << ")" << endl;
// cout << "CDO -- UB_LLEG_G: (" << x_lleg_g << ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;
// cout << "CDO -- DIST: " << dist << endl;

// Comprobar que la distancia desde la posicion de partida a la posicion del obstaculo
// es menor, o igual, que el alcance maximo del laser (expresado en metros).
// Si no se conoce el estado de la posicion de llegada, prob_pos_libre = 0'5, se marca la distancia
// como 'distancia maxima'.
  if (dist > alcance_laser_M_)
  {
    dist = alcance_laser_M_;
  }

// cout << endl;
// cout << "CDO -- DIST: " << dist << endl;

  return dist;
}

#ifdef DOS_MAPAS

// LOS METODOS QUE SE HAYAN DENTRO DE ESTE IFDEF DEBEN SER REPASADOS A CONCIENCIA
// PUES NO SE HAN USADO EN MUCHO TIEMPO. SE HAN CODIFICADO A PARTIR
// DE LOS METODOS QUE USA EL ALGORITMO CUANDO SOLO HAY UN MAPA Y PUEDE QUE ALGUNA
// CONDICION SE ME HAYA PASADO POR ALTO.

void LocED::trazadoRayos2(const Ubicacion& ub_g, const double& incr_x_ex_g, const double& incr_y_ex_g, double& dista,
    double& distb)
{
// Coordenadas 'enteras' de la posicion alcanzada.
  int c_x_lleg_g = 0;
  int f_y_lleg_g = 0;

// Probabilidad de que la ubicacion alcanzada este libre en el mapa_a_ y en el mapa_b_.
  double prob_p_libre_a = 0.0;
  double prob_p_libre_b = 0.0;
  double dist = 0.0;
// Coordenadas maximas del entorno donde se realiza la localizacion.
  double x_max_mapa = mapa_a_.getColumnas() - scRound;
  double y_max_mapa = mapa_a_.getFilas() - scRound;
// Coordenadas minimas del entorno donde se realiza la localizacion.
  double x_min_mapa = 0;
  double y_min_mapa = 0;
// Coordenadas de la posicion a alcanzada.
  double x_lleg_g = 0;
  double y_lleg_g = 0;
// Orientacion con la que explorar.
  double t_ex_g = 0;
  ub_g.getXYTheta(x_lleg_g, y_lleg_g, t_ex_g);

// cout << endl;
// cout << "CDO2 -- UB_G: (" << x_g << ", " << y_g << ", " <<  scGradRad * ub_g.getTheta() << ")" << endl;
// cout << "CDO2 -- UB_LLEG_G: (" << x_lleg_g << ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;
// cout << "CDO2 -- INC_X_EX_G: " << incr_x_ex_g << endl;
// cout << "CDO2 -- INC_Y_EX_G: " << incr_y_ex_g << endl;

// Explorar en una determinada orientacion, 't_ex_g', hasta que:
// 1.- Se encuentre una posicion ocupada por un obstaculo.
// 2.- Se recorra una distancia igual al alcance maximo medible por el laser.
// 3.- Se excedan los limites del mapa.

  do
  {
    // La ubicacion de partida no puede estar ocupada por un obstaculo. De este hecho ya me
    // he asegurado fuera de esta funcion. Por este motivo, la primera ubicacion que puedo
    // evaluar es la siguiente ubicacion a la ubicacion de partida en la orientacion indicada por
    // la variable 't_ex_g'.
    x_lleg_g += incr_x_ex_g;
    y_lleg_g += incr_y_ex_g;
    dist += tM;

    // cout << endl;
    // cout <<"CDO2 -- UB_LLEG_G: (" x_lleg_g << ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;

    // Comprobar si la ubicacion alcanzada se sale fuera de los limites del mapa.
    if (x_lleg_g < x_min_mapa || x_lleg_g >= x_max_mapa || y_lleg_g < y_min_mapa || y_lleg_g >= y_max_mapa)
    {
      // cout << endl;
      // cout << "CDO2 -- UB_LLEG_G FUERA MAPA" << endl;

      // La ubicacion alcanzada esta fuera de los limites del mapa y por lo tanto se recupera la ubicacion
      // anterior, que SI que estaba dentro del mapa, porque esta ubicacion fue evaluada en la iteracion
      // anterior.
      x_lleg_g -= incr_x_ex_g;
      y_lleg_g -= incr_y_ex_g;
      dist -= tM;

      // cout << "CDO2 -- UB_LLEG_G: (" x_lleg_g ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;

      // Marco la ubicacion actual como ocupada y de este modo cuando la instruccion 'continue' interrumpa
      // la iteracion actual del bucle, y me lleve a la expresion while(prob_pos_libre > umbral_pos_ocupada),
      // este se detenga porque la condicion no se satisface.
      prob_p_libre_a = 0;
      prob_p_libre_b = 0;
      continue;
    }

    // Averiguar si la posicion seleccionada esta ocupada.
    roundLoc(x_lleg_g, y_lleg_g, c_x_lleg_g, f_y_lleg_g);
    prob_p_libre_a = mapa_a_.obtenerProbabilidadOcupacion(c_x_lleg_g, f_y_lleg_g);
    prob_p_libre_b = mapa_b_.obtenerProbabilidadOcupacion(c_x_lleg_g, f_y_lleg_g);

    // cout << "CDO2-- UB_LLEG_G (ROUND): (" << c_x_lleg_g << ", " << f_y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;
    // cout << "CDO2 -- PROB_A: " << prob_p_libre_a << endl;
    // cout << "CDO2 -- PROB_B: " << prob_p_libre_b << endl;

    // El flujo de ejecucion se sale de este bucle si el haz de exploracion se ha salido
    // del mapa, o si el haz de exploracion ha recorrido una distancia mayor de la que el laser
    // es capaz de medir, o si alguna de las probabilidades hayadas es inferior a
    // 'scUmbralOcupacion'
  }while (dist < alcance_laser_M_ && prob_p_libre_a > umbral_ocupacion_ && prob_p_libre_b > umbral_ocupacion_);

// cout << endl;
// cout << "CDO2 -- UB_G: (" << x_g << ", " << y_g << ", " << scGradRad * t_ex_g << ")" << endl;
// cout << "CDO2 -- UB_LLEG_G: (" << x_lleg_g << ", " << y_lleg_g << ", " << scGradRad * t_ex_g << ")" << endl;
// cout << "CDO2 -- DIST: " << dist << endl;

// Porque el ¿=?. Si no uso el = y 'dist' es igual a alcance_laser_m_,
// entonces esta condicion no se cumple y el flujo de ejecucion seguiria
// relizando las siguientes comprobaciones, cosa que es innecesaria porque
// dist debe acabar valiendo alcance_laser_m_. Usando el = me ahorro
// comprobaciones y se gana velocidad de ejecucion.
  if (dist >= alcance_laser_M_)
  {
    // cout << endl;
    // cout << "CDO2 -- DIST MAX ALCANZADA: " << cAlcanceLaserM << endl;

    dista = alcance_laser_M_;
    distb = alcance_laser_M_;
  }

// Obstaculo encontrado en AMBOS mapas.

  else if (prob_p_libre_a <= umbral_ocupacion_ && prob_p_libre_b <= umbral_ocupacion_)
  {
    // cout << endl;
    // cout << "CDO2 -- MA y MB: UB_LLEG_G OCUPADA" << endl;

    dista = dist;
    distb = dist;
  }

// Obstaculo NO encontrado en el mapa_a_ y SI encontrado en el mapa_b_.

  else if (prob_p_libre_a > umbral_ocupacion_ && prob_p_libre_b <= umbral_ocupacion_)
  {
    // cout << endl;
    // cout << "CDO2 -- MA: UB_LLEG_G LIBRE -- MB: UB_LLEG_G OCUPADA" << endl;

    dista = trazadoRayos(mapa_a_, Ubicacion(x_lleg_g, y_lleg_g, t_ex_g), incr_x_ex_g, incr_y_ex_g, dist);
    distb = dist;
  }
// No hace falta usar el bloque if-else de abajo, ya que si me he salido del bucle do-while, es
// por 4 motivos:
// 1. Distancia maxima medible por el laser alcanzada.
// 2. Obstculo hayado en dos mapas.
// 3. Obstaculo hayado en uno de los mapas.
// 4. Obstaculo hayado en el otro de los mapas.

// else if (prob_p_libre_a <= umbral_pos_ocupada_ && prob_p_libre_b > umbral_pos_ocupada_)

  else
  {
    // cout << endl;
    // cout << "CDO2 -- MA: UB_LLEG_G OCUPADA -- MB: UB_LLEG_G LIBRE" << endl;

    dista = dist;
    distb = trazadoRayos(mapa_b_, Ubicacion(x_lleg_g, y_lleg_g, t_ex_g), incr_x_ex_g, incr_y_ex_g, dist);
  }

// cout << endl;
// cout << "CDO -- DIST_A: " << dist_a[i] << endl;
// cout << "CDO -- DIST_B: " << dist_b[i] << endl;

}

double LocED::aptitudIndividuo(const Ubicacion& ub_g, const std::vector<double>& dist_laser)
{

  ind_med_laser_.assign(cNumMedLaser, 0);

  unsigned int i = 0;
  unsigned int j = 0;

// Permutacion de indices de medidas laser.
  for (i = 0; i < cNumMedLaser; ++i)
  {
    j = rand() % (i + 1);
    ind_med_laser_[i] = ind_med_laser_[j];
    ind_med_laser_[j] = i;
  }

  ind_med_laser_.resize(num_med_laser_act_);

// for (i = 0; i < num_med_laser_act_; ++i) {
//    cout << i << " -- " << ind_med_laser_[i] << " -- " << scGradRad * ang_[ind_med_laser_[i]] << endl;
//}

// Coordenadas de la ubicacion seleccionada, donde comienza la exploracion.
  double x_g = 0;
  double y_g = 0;
  double t_g = 0;
  ub_g.getXYTheta(x_g, y_g, t_g);

  double x_max_mapa = mapa_a_.getColumnas() - scRound;
  double y_max_mapa = mapa_a_.getFilas() - scRound;

// Si la ubicacion seleccionada esta FUERA de la region de interes se descarta haciendo su coste
// infinito.
//if (x_g < ub_min_regint_g_.getX() || x_g >= ub_max_regint_g_.getX() || y_g < ub_min_regint_g_.getY() || y_g
//    >= ub_max_regint_g_.getY())
//{
//  return DBL_MAX;
//}
  if (x_g < 0 || x_g >= x_max_mapa || y_g < 0 || y_g >= y_max_mapa)
  {
    return DBL_MAX;
  }

// A PARTIR DE AQUI LA UBICACION SELECCIONADA ESTA DENTRO DE LA REGION DE INTERES.

  int c_x_g = 0;
  int f_y_g = 0;
  double prob_p_libre_a = 0.0;
  double prob_p_libre_b = 0.0;

// Comprobar si la posicion seleccionada esta ocupada.
  roundLoc(x_g, y_g, c_x_g, f_y_g);
  prob_p_libre_a = mapa_a_.obtenerProbabilidadOcupacion(c_x_g, f_y_g);
  prob_p_libre_b = mapa_b_.obtenerProbabilidadOcupacion(c_x_g, f_y_g);

// Ubicacion seleccionada ocupada en ambos mapas.
  if (prob_p_libre_a <= umbral_ocupacion_ && prob_p_libre_b <= umbral_ocupacion_)
  {
    return DBL_MAX;
  }

  std::vector<double> dist_a(num_med_laser_act_, 0);
  std::vector<double> dist_b(num_med_laser_act_, 0);

// Orientacion con la que explorar.
  double t_ex_g = 0;
// Incrementos en 'x' e 'y' en la orientacion de exploracion.
  double incr_x_ex_g = 0;
  double incr_y_ex_g = 0;

  double dif_abs_a = 0;
  double dif_abs_b = 0;
  double difAbs = 0;
  double dist_max = 0;
  double fraccion = 0.50;

  unsigned int num_med_util = 0;

// Coste de la ubicacion seleccionada.
  double coste = 0;

  for (i = 0; i < num_med_laser_act_ && num_med_util < cNumMedLaserNorm; ++i)
  {
    // Explorar en la orientacion 't_ex_g', hasta encontrar un obstaculo o hasta llegar al alcance maximo
    // del laser.
    t_ex_g = t_g - ang_[ind_med_laser_[i]];
    incr_x_ex_g = dist_traslacion_C_ * cos(t_ex_g);
    incr_y_ex_g = dist_traslacion_C_ * sin(t_ex_g);

    // cout << endl;
    // cout << "T_EX_G: " << scGradRad * t_ex_g << endl;
    // cout << "INC_X_EX_G: " << incr_x_ex_g << endl;
    // cout << "INC_Y_EX_G: " << incr_y_ex_g << endl;

    if (prob_p_libre_a > umbral_ocupacion_ && prob_p_libre_b > umbral_ocupacion_)
    {
      // cout << "CDO2 -- MA: UB_LLEG_G LIBRE -- MB: UB_LLEG_G LIBRE" << endl;
      trazadoRayos2(ub_g, incr_x_ex_g, incr_y_ex_g, dist_a[i], dist_b[i]);
    }
    else if (prob_p_libre_a > umbral_ocupacion_ && prob_p_libre_b <= umbral_ocupacion_)
    {
      // cout << "CDO2 -- MA: UB_LLEG_G LIBRE -- MB: UB_LLEG_G OCUPADA" << endl;
      dist_a[i] = trazadoRayos(mapa_a_, ub_g, incr_x_ex_g, incr_y_ex_g, 0);
    }
    else
    {
      // cout << "CDO2 -- MA: UB_LLEG_G OCUPADA -- MB: UB_LLEG_G LIBRE" << endl;
      dist_b[i] = trazadoRayos(mapa_b_, ub_g, incr_x_ex_g, incr_y_ex_g, 0);
    }

    dif_abs_a = fabs(dist_laser[ind_med_laser_[i]] - dist_a[i]);
    dif_abs_b = fabs(dist_laser[ind_med_laser_[i]] - dist_b[i]);

    if (dif_abs_a > dif_abs_b)
    {
      difAbs = dif_abs_b;
      dist_max = std::max(dist_laser[ind_med_laser_[i]], dist_b[i]);
    }
    else
    {
      difAbs = dif_abs_a;
      dist_max = std::max(dist_laser[ind_med_laser_[i]], dist_a[i]);
    }
    // Explicacion arriba.
    if (bandera_loc_global_exito_ && difAbs >= (dist_max * fraccion))
    {
      // printf("\n");
      // printf("CC -- i: %d -- iml: %d -- ang: %f.\n", i, ind_med_laser_[i], scGradRad * ang_[ind_med_laser_[i]]);
      // printf("CC -- DIST_A[%d]: %.8f\n", i, dist_a[i]);
      // printf("CC -- DIST_B[%d]: %.8f\n", i, dist_b[i]);
      // printf("CC -- DIST_L[%d]: %.8f\n", i, dist_laser[ind_med_laser_[i]]);
      // printf("CC -- DIST_MAX: %.8f\n", dist_max);
      // printf("CC -- DIST_MAX * %f: %.8f\n", porcentaje, dist_max * fraccion);
      // printf("CC -- DIF_ABS_MIN: %.8f\n", difAbs);
      difAbs = 0;
      --num_med_util;
    }
    coste += difAbs;
    ++num_med_util;
  }

// cout << endl;
// cout << "CF -- COSTE: " << coste << endl;

  if (bandera_loc_global_exito_)
  {
    double incr_x_g = 0.0;
    double incr_y_g = 0.0;
    double incr_t_g = 0.0;
    // Calcular la diferencia entre las coordenadas x, y, theta de la particula evaluada y la particula estimada.
    ubEstSrg_.getXYTheta(incr_x_g, incr_y_g, incr_t_g);
    incr_x_g -= x_g;
    incr_y_g -= y_g;
    incr_t_g -= t_g;

    coste += (coef_hipotenusa_ * (ladoCeldaM * hypot(incr_x_g, incr_y_g))) + (coef_angulo_ * fabs(incr_t_g));
  }

  return coste;
}

#else

double LocED::aptitudIndividuo(const Ubicacion& ub_g, const std::vector<double>& dist_laser)
{

  //cout << "aptitudIndividuo" << endl;

  unsigned int i = 0;
  unsigned int j = 0;

// Coordenadas de la ubicacion seleccionada, donde comienza la exploracion.
  double x_g = 0;
  double y_g = 0;
  double t_g = 0;
  ub_g.getXYTheta(x_g, y_g, t_g);

//double x_max_mapa = mapa_a_.columnas() - scRound;
//double y_max_mapa = mapa_a_.filas() - scRound;
//if (x_g < 0 || x_g >= x_max_mapa || y_g < 0 || y_g >= y_max_mapa)
//{
//return DBL_MAX ;
//}

// Si la ubicacion seleccionada esta FUERA de la region de interes se descarta haciendo su coste
// infinito.
  if (x_g < ub_min_regint_g_.getX() || x_g >= ub_max_regint_g_.getX() || y_g < ub_min_regint_g_.getY()
      || y_g >= ub_max_regint_g_.getY())
  {
    //cout << "Fuera de la region de interes" << endl;
    return DBL_MAX ;
  }
// A PARTIR DE AQUI LA UBICACION SELECCIONADA ESTA DENTRO DE LA REGION DE INTERES.
  int c_x_g = 0;
  int f_y_g = 0;
  double prob_p_libre_a = 0.0;

// Comprobar si la posicion seleccionada esta ocupada.
  roundLoc(x_g, y_g, c_x_g, f_y_g);
  prob_p_libre_a = mapa_a_.obtenerProbabilidadOcupacion(c_x_g, f_y_g);
  if (prob_p_libre_a <= umbral_ocupacion_)
  {
    //cout << "Individuo cae en ubicacion ocupada" << endl;
    return DBL_MAX ;
  }

// Permutacion de indices de medidas laser.
  ind_med_laser_.assign(cNumMedLaser, 0);
  for (i = 0; i < cNumMedLaser; ++i)
  {
    j = rand() % (i + 1);
    ind_med_laser_[i] = ind_med_laser_[j];
    ind_med_laser_[j] = i;
  }
  //if (banderaLocConExito_)
  //{
  //for (i = 0; i < num_med_laser_act_; ++i)
  //{
  //cout << i << " " << ind_med_laser_[i] << endl;
  //}
  //}
  //double fraccion = 0.50;
  // Coste de la ubicacion seleccionada.
  //double aptitud = 0;
  // Orientacion con la que explorar.
  //double t_ex_g = 0;
  // Incrementos en 'x' y en 'y' en la orientacion de exploracion.
  //double incr_x_ex_g = 0;
  //double incr_y_ex_g = 0;
  //double difAbs = 0;
  //double dist_max = 0;
  if (bandera_loc_global_exito_)
  {
    return aptitudIndividuoConConvergencia(ub_g, dist_laser);
  }
  else
  {
    return aptitudIndividuoSinConvergencia(ub_g, dist_laser);
  }

}
/* Calcular las distancias a los obstaculos situados alrededor de la ubicacion
 * seleccionada en el 'mapa_a_'.
 */
double LocED::aptitudIndividuoConConvergencia(const Ubicacion & ub_g, const std::vector<double> & dist_laser)
{

  //cout << "aptitudIndividuoConConvergencia" << endl;

  // Coordenadas de la ubicacion seleccionada, donde comienza la exploracion.
  double x_g = 0;
  double y_g = 0;
  double t_g = 0;
  ub_g.getXYTheta(x_g, y_g, t_g);
  // Orientacion con la que explorar.
  double t_ex_g = 0;
  // Incrementos en 'x' y en 'y' en la orientacion de exploracion.
  double incr_x_ex_g = 0;
  double incr_y_ex_g = 0;
  // Primera medida laser disponible, que esta vinculada al barrido
  // 'ind_med_laser_[j]', y por tanto al angulo 'ang_[ind_med_laser_[j]]'.
  unsigned int j = num_med_laser_act_;
  double difAbs = 0.0;
  double aptitud = 0.0;
  unsigned int indInsertar = 0;
  std::vector<double> dist_a(num_med_laser_act_, 0);
  //int k = 0;
  //double difAbsGuardada = 0;
  std::vector<double> difAbsGuardada;
  double difAbsOrig = 0;

  cout.precision(5);

  // Procesar 'num_med_laser_act_' medidas laser.
  for (unsigned int i = 0; i < num_med_laser_act_; ++i)
  {
    // Explorar, en el mapa, en la orientacion 't_ex_g', hasta encontrar un obstaculo o hasta
    // llegar al alcance maximo del laser.
    t_ex_g = t_g - ang_[ind_med_laser_[i]];
    incr_x_ex_g = dist_traslacion_C_ * cos(t_ex_g);
    incr_y_ex_g = dist_traslacion_C_ * sin(t_ex_g);
    // cout << endl;
    // cout << "T_EX_G: " << scGradRad * t_ex_g << endl;
    // cout << "INC_X_EX_G: " << incr_x_ex_g << endl;
    // cout << "INC_Y_EX_G: " << incr_y_ex_g << endl;
    dist_a[i] = trazadoRayos(mapa_a_, Ubicacion(x_g, y_g, t_ex_g), incr_x_ex_g, incr_y_ex_g, 0);
    // Diferencia absoluta entre la distancia real (obtenida por el laser) y la distancia
    // calculada en el mapa, en la orientacion 't_ex_g'.
    difAbs = fabs(dist_laser[ind_med_laser_[i]] - dist_a[i]);
    // Si el algoritmo alcanzo la convergencia al finalizar el estado anterior se realiza descarte
    // de medidas si la medida real y la simulada en la orientacion 't_ex_g' son muy dispares, y
    // quedan orientaciones libres que explorar.
    //cout << "i " << i << " indMedLaser " << ind_med_laser_[i] << " orientacion " << ang_[ind_med_laser_[i]] << " difAbs " << difAbs << endl;

    //k = i;
    difAbsOrig = difAbs;
    //difAbsGuardada = difAbs;
    while (difAbs > (scCoefAptitudLimSup * 2 * cDist3SigmaM) && j < cNumMedLaser)
    {

      indInsertar = obtenerIndicePorBusquedaBinaria(difAbs, difAbsGuardada, 0, difAbsGuardada.size() - 1);
      difAbsGuardada.insert(difAbsGuardada.begin() + indInsertar, difAbs);

      t_ex_g = t_g - ang_[ind_med_laser_[j]];
      incr_x_ex_g = dist_traslacion_C_ * cos(t_ex_g);
      incr_y_ex_g = dist_traslacion_C_ * sin(t_ex_g);
      dist_a[i] = trazadoRayos(mapa_a_, Ubicacion(x_g, y_g, t_ex_g), incr_x_ex_g, incr_y_ex_g, 0);
      difAbs = fabs(dist_laser[ind_med_laser_[j]] - dist_a[i]);
      //cout << "j " << j << " difAbs " << difAbs;
      // Este 'if' sirve para que en el caso de que todas las orientaciones disponibles
      // proporcionen malos resultados, quedarme con la 'difAbs' mas pequeña (la menos mala).

      //if (difAbs < difAbsGuardada) {
      //k = j;
      //difAbsGuardada = difAbs;
      //cout << " Guardo" << endl;
      //} else {
      //cout << endl;
      //}
      ++j;
    }
    // Todas las orientaciones disponibles (desde 'num_med_laser_act_' hasta 'cNumMedLaser-1')
    // proporcionan malos resultados.
    if (j == cNumMedLaser && difAbs > (scCoefAptitudLimSup * 2 * cDist3SigmaM))
    {
      cout << "j == cNumMedLaser" << endl;
      //difAbs = difAbsGuardada;
      indInsertar = obtenerIndicePorBusquedaBinaria(difAbs, difAbsGuardada, 0, difAbsGuardada.size() - 1);
      difAbsGuardada.insert(difAbsGuardada.begin() + indInsertar, difAbs);
      difAbs = difAbsGuardada[0];
      cout << "i " << i << " indMedLaser " << ind_med_laser_[i] << " orientacion " << ang_[ind_med_laser_[i]]
          << " difAbsOrig " << difAbsOrig << " difAbs " << difAbs << endl;
      difAbsGuardada.erase(difAbsGuardada.begin());
      cout << " difAbs " << difAbs << endl;
    }

    aptitud += difAbs;
  }
  // cout << endl;
  // cout << "AptitudIndividuo -- aptitud: " << aptitud << endl;

  double incr_x_g = 0.0;
  double incr_y_g = 0.0;
  double incr_t_g = 0.0;
  // Calcular la diferencia entre las coordenadas x, y, theta de la ubicacion seleccionada y
  // de la ubicacion predicha.
  ubPreSrg_.getXYTheta(incr_x_g, incr_y_g, incr_t_g);
  incr_x_g -= x_g;
  incr_y_g -= y_g;
  incr_t_g -= t_g;

  // Expresiones originales comentadas.
  // d_norm = sqrt((pow(incr_x, 2) + pow(incr_y, 2)) / sigma_cuadrado);
  // d_ang = sqrt(pow(incr_theta, 2) / sigma_cuadrado);
  // aptitud += (k_mult * d_norm) + (k_ang * d_ang);
  aptitud += (coef_hipotenusa_ * (ladoCeldaM * hypot(incr_x_g, incr_y_g))) + (coef_angulo_ * fabs(incr_t_g));
  // cout << "aptitud: " << aptitud << endl;
  return aptitud;
}

double LocED::aptitudIndividuoSinConvergencia(const Ubicacion & ub_g, const std::vector<double> & dist_laser)
{
  //cout << "aptitudIndividuoSinConvergencia" << endl;
  // Coordenadas de la ubicacion seleccionada, donde comienza la exploracion.
  double x_g = 0;
  double y_g = 0;
  double t_g = 0;
  ub_g.getXYTheta(x_g, y_g, t_g);
  // Orientacion con la que explorar.
  double t_ex_g = 0;
  // Incrementos en 'x' y en 'y' en la orientacion de exploracion.
  double incr_x_ex_g = 0;
  double incr_y_ex_g = 0;
  double difAbs = 0.0;
  double aptitud = 0.0;
  std::vector<double> dist_a(num_med_laser_act_, 0);
  // Procesar 'num_med_laser_act_' medidas laser.
  for (unsigned int i = 0; i < num_med_laser_act_; ++i)
  {
    // Explorar en la orientacion 't_ex_g', hasta encontrar un obstaculo o hasta llegar al alcance maximo
    // del laser.
    t_ex_g = t_g - ang_[ind_med_laser_[i]];
    incr_x_ex_g = dist_traslacion_C_ * cos(t_ex_g);
    incr_y_ex_g = dist_traslacion_C_ * sin(t_ex_g);
    // cout << endl;
    // cout << "T_EX_G: " << scGradRad * t_ex_g << endl;
    // cout << "INC_X_EX_G: " << incr_x_ex_g << endl;
    // cout << "INC_Y_EX_G: " << incr_y_ex_g << endl;
    dist_a[i] = trazadoRayos(mapa_a_, Ubicacion(x_g, y_g, t_ex_g), incr_x_ex_g, incr_y_ex_g, 0);
    // Diferencia absoluta entre la distancia real y la distancia simulada, en la orientacion 't_ex_g'
    difAbs = fabs(dist_laser[ind_med_laser_[i]] - dist_a[i]);
    // Si el algoritmo alcanzo la convergencia al finalizar el estado anterior se realiza descarte
    // de medidas si la medida real y la simulada en la orientacion 't_ex_g' son muy dispares, y
    // quedan orientaciones libres que explorar.
    //cout << "i " << i << " indMedLaser " << ind_med_laser_[i] << " orientacion " << ang_[ind_med_laser_[i]] << " difAbs " << difAbs << endl;
    aptitud += difAbs;
  }
  // cout << "aptitud: " << aptitud << endl;
  return aptitud;
}

#endif

void LocED::crearPoblacion(const std::vector<double>& dist_laser, const unsigned int& num_elem_pob)
{

// Region de interes
  double x_min_regint_g = 0;
  double y_min_regint_g = 0;
  double t_min_regint_g = 0;
  double incr_x_g = 0;
  double incr_y_g = 0;
  double incr_t_g = 0;
  ub_min_regint_g_.getXYTheta(x_min_regint_g, y_min_regint_g, t_min_regint_g);
  ub_max_regint_g_.getXYTheta(incr_x_g, incr_y_g, incr_t_g);
  incr_x_g -= x_min_regint_g;
  incr_y_g -= y_min_regint_g;
  incr_t_g -= t_min_regint_g;
  Ubicacion ub_g;
  double coste_ub_g = 0;
  unsigned int indInsertar = 0;
  for (unsigned int i = 0; i < num_elem_pob; ++i)
  {
    ub_g.setXYTheta(x_min_regint_g + (drand48() * incr_x_g), y_min_regint_g + (drand48() * incr_y_g),
                    t_min_regint_g + (drand48() * incr_t_g));
    coste_ub_g = aptitudIndividuo(ub_g, dist_laser);
    indInsertar = obtenerIndicePorBusquedaBinaria(coste_ub_g, coste_pob_, 0, coste_pob_.size() - 1);
    coste_pob_.insert(coste_pob_.begin() + indInsertar, coste_ub_g);
    poblacion_.insert(poblacion_.begin() + indInsertar, ub_g);
    /*cout << "Elementos" << endl;
     for(int i = 0; i < coste_pob_.size(); i++){
     cout << coste_pob_[i] << endl;
     }
     cout<<endl;*/
  }
}

void LocED::comprobarLimitesRegionInteres()
{

  double x_max_mapa = mapa_a_.getColumnas() - scRound;
  double y_max_mapa = mapa_a_.getFilas() - scRound;

  double x_min_regint_g = 0;
  double y_min_regint_g = 0;
  ub_min_regint_g_.getXY(x_min_regint_g, y_min_regint_g);

  double x_max_regint_g = 0;
  double y_max_regint_g = 0;
  ub_max_regint_g_.getXY(x_max_regint_g, y_max_regint_g);

// Comprobar que la region de interes no excede los limites del mapa.
  if (x_min_regint_g < 0)
  {
    x_min_regint_g = 0;
  }
  if (x_max_regint_g > x_max_mapa)
  {
    x_max_regint_g = x_max_mapa;
  }
  if (y_min_regint_g < 0)
  {
    y_min_regint_g = 0;
  }
  if (y_max_regint_g > y_max_mapa)
  {
    y_max_regint_g = y_max_mapa;
  }
  ub_min_regint_g_.setXYTheta(x_min_regint_g, y_min_regint_g, 0.0);
  ub_max_regint_g_.setXYTheta(x_max_regint_g, y_max_regint_g, 2 * M_PI);
}

std::vector<std::vector<unsigned int> > LocED::crearPermutaciones()
{
  std::vector<std::vector<unsigned int> > rotaciones(numero_componentes_, std::vector<unsigned int>(num_elem_pob_act_, 0));
  unsigned int i = 0;
  unsigned int j = 0;
  for (i = 0; i < num_elem_pob_act_; ++i)
  {
    // NO OLVIDAR PONER SRAND(TIME(NULL)) EN EL MAIN!!
    // El operador '%' solo se usa con operandos que sean enteros.
    j = rand() % (i + 1);
    rotaciones[0][i] = rotaciones[0][j];
    rotaciones[0][j] = i;
  }

  for (i = 1; i < numero_componentes_; ++i)
  {
    // El operador '%' solo se usa con operandos que sean enteros.
    // j tiene un valor comprendido entre 0 y num_elem_pob_act - 1.
    j = rand() % (num_elem_pob_act_);
    rotaciones[i].assign(rotaciones[0].begin() + j, rotaciones[0].end());
    rotaciones[i].insert(rotaciones[i].end(), rotaciones[0].begin(), rotaciones[0].begin() + j);
  }

  return rotaciones;
}

double LocED::desplazarPoblacion(const Ubicacion& ub_ant_rr_o, const Ubicacion& ub_act_rr_o,
                                 const std::vector<double>& dist_laser)
{

  // Duplicar la poblacion obtenida al finalizar el proceso de localizacion.
  // Limpiar la poblacion y el coste.
  std::vector<Ubicacion> pobCopia(poblacion_.begin(), poblacion_.end());
  poblacion_.clear();
  coste_pob_.clear();
// ANTERIOR ubicacion ocupada por el robot expresada en el sistema de referencia odometrico.
  double x_ant_rr_o = 0;
  double y_ant_rr_o = 0;
  double t_ant_rr_o = 0;
  ub_ant_rr_o.getXYTheta(x_ant_rr_o, y_ant_rr_o, t_ant_rr_o);
// ACTUAL ubicacion ocupada por el robot expresada en el sistema de referencia odometrico.
  double x_act_rr_o = 0;
  double y_act_rr_o = 0;
  double t_act_rr_o = 0;
  ub_act_rr_o.getXYTheta(x_act_rr_o, y_act_rr_o, t_act_rr_o);
// Incremento entre dos posiciones consecutivas ocupadas por el robot en el entorno.
// (Incrementos expresados en el sistema de referencia odometrico, en celdas)
  double incr_x_rr_o = x_act_rr_o - x_ant_rr_o;
  double incr_y_rr_o = y_act_rr_o - y_ant_rr_o;
  double incr_t_rr_o = t_act_rr_o - t_ant_rr_o;
  cout << endl;
  cout << "INCR_X_RR_O: " << incr_x_rr_o << endl;
  cout << "INCR_Y_RR_O: " << incr_y_rr_o << endl;
  cout << "INCR_T_RR_O: " << scGradRad * incr_t_rr_o << endl;
// Operaciones giro-desplazamiento-giro que conducen desde la ubicacion 'x_ant_rr_o' a la
// ubicacion 'x_act_rr_o', en el sistema de coordenadas odometrico.
// Para entender estos calculos mirar mi tesis de master.
  double D2 = hypot(incr_x_rr_o, incr_y_rr_o);
// Para evitar divisiones 0/0 o algo/0 sumo al segundo parametro una cantidad
// insignificante: 0.0000000000001.
  double gamma_o = atan2(incr_y_rr_o, incr_x_rr_o + 0.0000000000001);
  double theta1 = gamma_o - t_ant_rr_o;
  double gamma_indiv = 0;
// Ubicacion seleccionada actualizada.
  double x_pst_g = 0;
  double y_pst_g = 0;
  double t_pst_g = 0;
  double incr_x_g = 0;
  double incr_y_g = 0;
  unsigned int i = 0;
  unsigned int indInsertar = 0;
  double aptitud = 0;
  for (i = 0; i < num_elem_pob_act_; ++i)
  {
    // Actualizar el individuo seleccionado con las operaciones giro-desplazamiento_giro.
    // Giro --> Para entender el signo '-' mirar mi tesis de master.
    gamma_indiv = pobCopia[i].getTheta() - theta1;
    // Desplazamienzo.
    // Se añade un pequeño ruido en posicion.
    incr_x_g = D2 * cos(gamma_indiv);
    incr_y_g = D2 * sin(gamma_indiv);
    x_pst_g = pobCopia[i].getX() + incr_x_g + ((2 * drand48()) - 1);
    y_pst_g = pobCopia[i].getY() + incr_y_g + ((2 * drand48()) - 1);
    // Giro --> Para entender el signo '-' mirar mi tesis de master.
    // Se añade un pequeño ruido en orientacion: 2 grados.
    t_pst_g = pobCopia[i].getTheta() - incr_t_rr_o + ((drand48() - 0.5) * 0.07);
    pobCopia[i].setXYTheta(x_pst_g, y_pst_g, t_pst_g);
    aptitud = aptitudIndividuo(pobCopia[i], dist_laser);
    indInsertar = obtenerIndicePorBusquedaBinaria(aptitud, coste_pob_, 0, coste_pob_.size() - 1);
    coste_pob_.insert(coste_pob_.begin() + indInsertar, aptitud);
    poblacion_.insert(poblacion_.begin() + indInsertar, pobCopia[i]);
  }
//cout << i << "Paso" << endl;
  return D2;
}

void LocED::mutarCruzarSeleccionarDescarte(const std::vector<double>& dist_laser)
{

  cout << "Mutar cruzar seleccionar descarte" << endl;

// Numero de iteraciones completadas.
  unsigned int k = 0;
// Puesto en la poblacion del individuo con el que se trabaja.
  unsigned int i = 0;
// Numero de descartes completados.
  unsigned int j = 0;

// Vector de tres permutaciones usado en la operacion de mutacion.
  std::vector<std::vector<unsigned int> > perm;

// Coordenadas de los individuos usados en la operacion de mutacion.
  double x_r0_g = 0;
  double y_r0_g = 0;
  double t_r0_g = 0;

  double x_r1_g = 0;
  double y_r1_g = 0;
  double t_r1_g = 0;

  double x_r2_g = 0;
  double y_r2_g = 0;
  double t_r2_g = 0;

// Coordenadas del individuo obtenido en la operacion de mutacion.
  double x_mut_g = 0;
  double y_mut_g = 0;
  double t_mut_g = 0;

// Coordenadas del individuo obtenido en la operacion de cruce.
  double x_cruce_g = 0;
  double y_cruce_g = 0;
  double t_cruce_g = 0;

  Ubicacion q_cruce_g;

// Aptitud del individuo obtenido por cruce.
  double aptitud_q = 0;

// Numero de descartes: el 5 % de los individuos de la poblacion resultante
// tras la operacion de seleccion.
  unsigned int num_descartes = (unsigned int)(0.05 * num_elem_pob_act_);

// Puesto en la poblacion del individuo usado en la operacion de descarte.
  unsigned int ind_desc = 0;

  double x_desc_g = 0;
  double y_desc_g = 0;
  double t_desc_g = 0;

  Ubicacion d_desc_g;

  double aptitud_d = 0;
  unsigned int indInsertar = 0;
// Individuos ancestros.
  std::vector<Ubicacion> pobCopia;
  do
  {
    // Guardar una copia de los individuos ancestros.
    pobCopia.assign(poblacion_.begin(), poblacion_.end());
    // Permutaciones de puestos.
    perm = crearPermutaciones();
    // MUTACION Y CRUCE UNIFORME
    // Crear ubicaciones descendientes de la poblacion actual.
    for (i = 0; i < num_elem_pob_act_; ++i)
    {
      // La ubicacion descendiente tiene por defecto las coordenadas
      // de uno de sus progenitores.
      // pobCopia[i].getXYTheta(x_cruce_g, y_cruce_g, t_cruce_g);

      // El segundo progenitor se obtiene a partir de tres ubicaciones.
      pobCopia[perm[0][i]].getXYTheta(x_r0_g, y_r0_g, t_r0_g);
      pobCopia[perm[1][i]].getXYTheta(x_r1_g, y_r1_g, t_r1_g);
      pobCopia[perm[2][i]].getXYTheta(x_r2_g, y_r2_g, t_r2_g);
      // MUTACION
      x_mut_g = x_r0_g + (cF * (x_r1_g - x_r2_g));
      y_mut_g = y_r0_g + (cF * (y_r1_g - y_r2_g));
      t_mut_g = t_r0_g + (cF * (t_r1_g - t_r2_g));
      // CRUCE UNIFORME O BINOMIAL
      // Valores por defecto
      x_cruce_g = x_mut_g;
      y_cruce_g = y_mut_g;
      t_cruce_g = t_mut_g;
      // La componente seleccionada de 'ub_mut_g' se propaga directamente al descendiente 'ub_cruce_g'.
      unsigned int jr = rand() % numero_componentes_;
      // Si se propaga la coordenada x (jr = 0)
      if (jr == 0)
      {
        if (drand48() > factor_cruce_)
        {
          y_cruce_g = pobCopia[i].getY();
        }
        if (drand48() > factor_cruce_)
        {
          t_cruce_g = pobCopia[i].getTheta();
        }
      }
      // Si se propaga la coordenada y (jr = 1)
      else if (jr == 1)
      {
        if (drand48() > factor_cruce_)
        {
          x_cruce_g = pobCopia[i].getX();
        }
        if (drand48() > factor_cruce_)
        {
          t_cruce_g = pobCopia[i].getTheta();
        }
      }
      // Si se propaga la coordenada t (jr = 2)
      else
      {
        t_cruce_g = t_mut_g;
        if (drand48() > factor_cruce_)
        {
          x_cruce_g = pobCopia[i].getX();
        }
        if (drand48() > factor_cruce_)
        {
          y_cruce_g = pobCopia[i].getY();
        }
      }
      // printf("\n");
      // printf("CSM -- I: %d.\n", i);
      // printf("CSM -- X_CRUCE_G: %.8f.\n", x_cruce_g);
      // printf("CSM -- Y_CRUCE_G: %.8f.\n", y_cruce_g);
      // printf("CSM -- T_CRUCE_G: %.8f.\n", scGradRad * t_cruce_g);
      q_cruce_g.setXYTheta(x_cruce_g, y_cruce_g, t_cruce_g);
      // Calcular el coste de la ubicacion descendiente e insertarla de forma ordenada dentro de la poblacion
      // de descendientes.
      aptitud_q = aptitudIndividuo(q_cruce_g, dist_laser);
      indInsertar = obtenerIndicePorBusquedaBinaria(aptitud_q, coste_pob_, 0, coste_pob_.size() - 1);
      coste_pob_.insert(coste_pob_.begin() + indInsertar, aptitud_q);
      poblacion_.insert(poblacion_.begin() + indInsertar, q_cruce_g);
    }
    // SELECCION (lambda + mu)-ES --> Los mejores M individuos entre la poblacion X y Q
    poblacion_.resize(num_elem_pob_act_);
    coste_pob_.resize(num_elem_pob_act_);
    // DESCARTE
    // Operacion de descarte aplicado a unos pocos individuos mientras no se haya logrado la convergencia.
    j = 0;
    while (!bandera_loc_global_exito_ && j < num_descartes)
    {
      // Escoger un individuo que pertenezca a la primera mitad de la poblacion.
      // num_elem_pob_act_ >> 1 equivale a parte_entera(num_elem_pob_act_/2)
      // La instruccion 'rand() % parte_entera(num_elem_pob_act_/2)' devuelve un numero comprendido
      // entre 0 y 'parte_entera(num_elem_pob_act_/2) - 1'

      ind_desc = rand() % (num_elem_pob_act_ >> 1);
      poblacion_[ind_desc].getXYTheta(x_desc_g, y_desc_g, t_desc_g);
      // printf("\n");
      // printf("CSM -- T_DESC_G: %.8f.\n", scGradRad * t_desc_g);

      // Leve perturbacion: 2 * drand48() - 1 --> [-1, +1)
      x_desc_g += ((2 * drand48()) - 1);
      y_desc_g += ((2 * drand48()) - 1);
      t_desc_g += ((drand48() - 0.5) * 0.07);		// +- 2º
      // printf("\n");
      // printf("CSM -- T_DESC_G: %.8f.\n", scGradRad * t_desc_g);
      d_desc_g.setXYTheta(x_desc_g, y_desc_g, t_desc_g);

      // Calcular el coste de la ubicacion perturbada.
      aptitud_d = aptitudIndividuo(d_desc_g, dist_laser);
      indInsertar = obtenerIndicePorBusquedaBinaria(aptitud_d, coste_pob_, 0, coste_pob_.size() - 1);
      coste_pob_.insert(coste_pob_.begin() + indInsertar, aptitud_d);
      poblacion_.insert(poblacion_.begin() + indInsertar, d_desc_g);
      ++j;
    }
    poblacion_.resize(num_elem_pob_act_);
    coste_pob_.resize(num_elem_pob_act_);
    //LimpiarImagenes();
    //dibujarPoblacion(1, poblacion_, Magick::Color("#0000FF"));
    // Nombre para guardar imagenes.
    //std::stringstream nombre_fich_pob_a;
    //nombre_fich_pob_a.str("");
    //nombre_fich_pob_a << "MCSD_" << num_loc + 1 << "_iter_" << k << ".png";
    //printf("\n"); printf("MCSD_NOMBRE_FICH: %s\n", nombre_fich_pob_a.str().c_str());
    //GuardarImagenes(nombre_fich_pob_a.str());

    ++k;
  } while (k < num_iter_act_);

// Ubicacion estimada al final de todas las generaciones.
  ubEstSrg_ = poblacion_[0];
  aptitudUbEstSrg_ = coste_pob_[0];
}

void LocED::dibujarLaserReal(const unsigned int& a_b, const std::vector<double>& dist_laser, const Magick::Color& color)
{

  MapaLoc* mapa = &mapa_a_;

#ifdef DOS_MAPAS

  if (a_b)
  {
    mapa = &mapa_b_;
  }

#endif

  unsigned int i = 0;

  int c_x_lleg_g = 0;
  int f_y_lleg_g = 0;
// Coordenadas de la posicion a alcanzada.
  double x_lleg_g = 0;
  double y_lleg_g = 0;
// Ubicacion estimada por el algoritmo.
  double x_est_g = 0;
  double y_est_g = 0;
  double t_est_g = 0;
  ubEstSrg_.getXYTheta(x_est_g, y_est_g, t_est_g);
// Orientacion con la que explorar.
  double t_ex_g = 0;
  double x_max_mapa = mapa->getColumnas() - scRound;
  double y_max_mapa = mapa->getFilas() - scRound;
// La ubicacioon alrededor de la cual se pintan las medidas laser esta fuera del mapa de trabajo.
  if (x_est_g < 0 || x_est_g >= x_max_mapa || y_est_g < 0 || y_est_g >= y_max_mapa)
  {
    return;
  }
// SE PINTAN TODAS LAS MEDIDAS DEL SENSOR LASER REAL.
  for (i = 0; i < cNumMedLaser; ++i)
  {
    // La primera orientacion a explorar es 'angulo_max_loc_' radianes a la derecha del eje x del
    // sistema de referencia local del robot. (giro horario)
    // Con el sistema de coordenadas global usado (esquina superior izquierda), el giro horario es
    // positivo.
    t_ex_g = t_est_g - ang_[i];
    // printf("\n");
    // printf("dist_laser[%d] = %f.\n", i, dist_laser[i]);
    // printf("t_est_g = %f\n", scGradRad * t_est_g);
    // printf("t_ex_g = %f\n", scGradRad * t_ex_g);
    // printf("PEX = %f\n", dist_laser[i] * cos(t_ex_g));
    // printf("PEY = %f\n", dist_laser[i] * sin(t_ex_g));
    // printf("CEX = %f\n", (dist_laser[i] * cos(t_ex_g)) / cLadoCeldaM);
    // printf("CEY = %f\n", (dist_laser[i] * sin(t_ex_g)) / cLadoCeldaM);
    x_lleg_g = x_est_g + ((dist_laser[i] * cos(t_ex_g)) / ladoCeldaM);
    y_lleg_g = y_est_g + ((dist_laser[i] * sin(t_ex_g)) / ladoCeldaM);
    if (x_lleg_g >= 0 && x_lleg_g < x_max_mapa && y_lleg_g >= 0 && y_lleg_g < y_max_mapa)
    {
      roundLoc(x_lleg_g, y_lleg_g, c_x_lleg_g, f_y_lleg_g);
      mapa->pintarPixel(c_x_lleg_g, f_y_lleg_g, color);
    }
  }
  return;
}

void LocED::dibujarPoblacion(const unsigned int& a_b, const std::vector<Ubicacion>& pob, const Magick::Color& color)
{

  MapaLoc* mapa = &mapa_a_;

#ifdef DOS_MAPAS

  if (a_b)
  {
    mapa = mapa_b_;
  }

#endif

  unsigned int i = 0;
  unsigned int num_elem_pob = pob.size();

//cout << endl;
//cout << "DP - num_elem_pob: " << num_elem_pob << endl;

  double x_g = 0;
  double y_g = 0;
  double t_g = 0;

  int c_x_g = 0;
  int f_y_g = 0;

  double x_max_mapa = mapa->getColumnas() - scRound;
  double y_max_mapa = mapa->getFilas() - scRound;

  for (i = 0; i < num_elem_pob; ++i)
  {
    pob[i].getXYTheta(x_g, y_g, t_g);

    if (x_g >= 0 && x_g < x_max_mapa && y_g >= 0 && y_g < y_max_mapa)
    {
      roundLoc(x_g, y_g, c_x_g, f_y_g);
      mapa->pintarPixel(c_x_g, f_y_g, color);
    }
  }
}

void LocED::dibujarPosicion(const unsigned int& a_b, const Ubicacion& ub_g, const Magick::Color& color)
{

  MapaLoc* mapa = &mapa_a_;

#ifdef DOS_MAPAS

  if (a_b)
  {
    mapa = mapa_b_;
  }

#endif

  double x_g = 0;
  double y_g = 0;
  double t_g = 0;

  int c_x_g = 0;
  int f_y_g = 0;

  double x_max_mapa = mapa->getColumnas() - scRound;
  double y_max_mapa = mapa->getFilas() - scRound;

  ub_g.getXYTheta(x_g, y_g, t_g);

  if (x_g >= 0 && x_g < x_max_mapa && y_g >= 0 && y_g < y_max_mapa)
  {
    roundLoc(x_g, y_g, c_x_g, f_y_g);
    mapa->pintarPixel(c_x_g, f_y_g, color);
  }
}

void LocED::dibujarPosicionEstPred(const unsigned int& a_b, Ubicacion ub_est_rr_g, const Magick::Color& color_or,
                                   const Magick::Color& color_pos)
{

  MapaLoc* mapa = &mapa_a_;

#ifdef DOS_MAPAS

  if (a_b)
  {
    mapa = mapa_b_;
  }

#endif

  double x_max_mapa = mapa->getColumnas() - scRound;
  double y_max_mapa = mapa->getFilas() - scRound;

  double x_i = ub_est_rr_g.getX();
  double y_i = ub_est_rr_g.getY();
  double t = ub_est_rr_g.getTheta();

// NO se pinta si la ubicacion inicial esta fuera del mapa.
  if (x_i < 0 || x_i >= x_max_mapa || y_i < 0 || y_i >= y_max_mapa)
  {
    return;
  }

  int longitud = 5;

  int x_f = 0;
  int y_f = 0;

  do
  {
    x_f = x_i + (longitud * cos(t));
    y_f = y_i + (longitud * sin(t));
    --longitud;
  } while ((x_f < 0 || x_f >= x_max_mapa || y_f < 0 || y_f >= y_max_mapa) && longitud != -1);

  int c_x_i = 0;
  int f_y_i = 0;
  roundLoc(x_i, y_i, c_x_i, f_y_i);

  int c_x_f = 0;
  int f_y_f = 0;
  roundLoc(x_f, y_f, c_x_f, f_y_f);

  mapa->dibujarOrientacion(c_x_i, f_y_i, c_x_f, f_y_f, color_or);
  dibujarPosicion(a_b, ub_est_rr_g, color_pos);
}

void LocED::dibujarRegionInteres(const unsigned int& a_b, const Magick::Color& color)
{

  MapaLoc* mapa = &mapa_a_;

#ifdef DOS_MAPAS

  if (a_b)
  {
    mapa = mapa_b_;
  }

#endif

  double x_max_mapa = mapa->getColumnas() - scRound;
  double y_max_mapa = mapa->getFilas() - scRound;

  double x_min = ub_min_regint_g_.getX();
  double y_min = ub_min_regint_g_.getY();

  double x_max = ub_max_regint_g_.getX();
  double y_max = ub_max_regint_g_.getY();

// Si alguna de las coordenadas que define la region cuadrada de interes se sale fuera de los limites del
// mapa la region no se pinta.
  if (x_min < 0)
  {
    x_min = 0;
  }
  if (x_max >= x_max_mapa)
  {
    x_max = mapa->getColumnas() - scRound;
    if (!scRound)
    {
      x_max -= 0.1;
    }
  }
  if (y_min < 0)
  {
    y_min = 0;
  }
  if (y_max >= y_max_mapa)
  {
    y_max = mapa->getFilas() - scRound;
    if (!scRound)
    {
      y_max -= 0.1;
    }
  }
  int c_x_min = 0;
  int f_y_min = 0;
  roundLoc(x_min, y_min, c_x_min, f_y_min);
  int c_x_max = 0;
  int f_y_max = 0;
  roundLoc(x_max, y_max, c_x_max, f_y_max);
  mapa->dibujarAreaCuadrada(c_x_min, f_y_min, c_x_max, f_y_max, color);
}

/* El vector 'vectorElementos' esta ordenado de menor a mayor valor de sus elementos.
 * Buscar en el vector 'vectorElementos', entre los indices 'indInicial' e 'indFinal', la posicion
 * en dicho vector en la que hay que insertar el elemento 'elemento'.
 * */
unsigned int LocED::obtenerIndicePorBusquedaBinaria(const double & elemento,
                                                    const std::vector<double> & vectorElementos, const int & indInicial,
                                                    const int & indFinal)
{
  int indIni = indInicial;
  int indFin = indFinal;
  int indMed = 0;
  // Si el vector de elementos esta vacio ==> insertar 'elemento' en la primera posicion. Obvio!
  if (!vectorElementos.size())
  {
    return 0;
  }
  /*
   cout << "Elemento: " << elemento << endl;
   cout << "indIni: " << indIni << " indFin: " << indFin << endl;
   */
  // Buscar en el vector 'vectorElementos' la posicion donde insertar el elemento
  // 'elemento' usando busqueda binaria.
  while (indIni <= indFin)
  {
    indMed = (indIni + indFin) / 2;
    // cout << "indIni: " << indIni << " indFin: " << indFin << endl;
    // cout << "indMed: " << indMed << endl;
    if (elemento > vectorElementos[indMed])
    {
      indIni = indMed + 1;
      //cout << "elemento (" << elemento << ") > v[iMed] (" << vectorElementos[indMed] << ") ==> indIni: " << indIni
      //    << " indFin: " << indFin << endl;
    }
    else if (elemento < vectorElementos[indMed])
    {
      indFin = indMed - 1;
      //cout << "elemento (" << elemento << ") < v[iMed] (" << vectorElementos[indMed] << ") ==> indIni: " << indIni
      //    << " indFin: " << indFin << endl;
    }
    else
    {
      return indMed;
    }
  }
  //int ind_insertar = indIni;
  //if (elemento > vectorElementos[indIni])
  //{
  //  ind_insertar = indIni + 1;
  //}
  //cout << "ind_insertar: " << ind_insertar << endl;
  return indIni;
}
