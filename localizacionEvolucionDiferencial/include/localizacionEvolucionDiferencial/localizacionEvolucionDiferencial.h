#ifndef LOCED_H_

#define LOCED_H_

#include <string>
#include <vector>

#include <Magick++.h>
#include <magick/MagickCore.h>

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include "mapa.h"
#include "ubicacion.h"

//#define DEBUG
//#define DOS_MAPAS

class LocED
{

public:

  LocED(const unsigned int& d, const unsigned int& st, const float& cr, const float& f, const float& t_c,
        const float& dist_conv_m, const unsigned int& num_elem_pob_norm, const unsigned int& num_elem_pob_conv,
        const unsigned int& num_iter_norm, const unsigned int& num_iter_conv, const std::string& nomb_fich_a,

#ifdef DOS_MAPAS

        const std::string& nomb_fich_b,

#endif

        const float& alcance_laser_m,
        const double& angulo_max, const double& res_angular, const unsigned int& numMedLaser,
        const float& dist_3sigma_m, const unsigned int& coef_incr, const unsigned int& coef_incr_conv);

  ~LocED()
  {
  }

  //private:
  // Constante de conversión de grados a radianes.
  static const double scRadGrad = 0.017453292519943295769236907684886;
  // Constante de conversión de radianes a grados.
  static const double scGradRad = 57.295779513082322864647721871734;
  static const float scRound = 0; //0.25;

  const unsigned int numero_componentes_;
  const unsigned int estrategia_mutacion_;
  const float factor_cruce_;
  const float factor_atenuacion_; // Entre 0 y 1.
  const float dist_traslacion_C_;
  float dist_traslacion_M_;
  // El algoritmo ha alcanzado la convergencia cuando toda la poblacion de individuos
  // se ha concentrado en un cuadrado de 'X' m de lado.
  float dist_convergencia_C_;
  const float dist_convergencia_M_;
  float lado_celda_M_;
  // Probabilidad por encima de la cual se considera que la celdilla
  // esta ocupada.
  // MIRAR EL FICHERO YAML DE UN MAPA.
  static const float umbral_ocupacion_ = 0.650;
  static const float umbral_no_ocupacion_ = 0.196;
  unsigned int bandera_loc_global_exito_;
  // Numero de elementos usados por el algoritmo mientras no ha alcanzado la
  // convergencia.
  const unsigned int num_elem_pob_norm_;
  // Numero de elementos usados por el algoritmo cuando ha alcanzado la convergencia.
  const unsigned int num_elem_pob_conv_;
  unsigned int num_elem_pob_act_;
  // Numero de iteraciones usadas por el algoritmo mientras no ha alcanzado la convergencia.
  const unsigned int num_iter_norm_;
  // Numero de iteraciones usadas por el algoritmo cuando ha alcanzado la convergencia.
  const unsigned int num_iter_conv_;
  // Numero de iteraciones a usar por el algoritmo.
  unsigned int num_iter_act_;

  bool mapa_recibido_;

  // Mapas usados para la localizacion.
  MapaLoc mapa_a_;

#ifdef DOS_MAPAS

  MapaLoc mapa_b_;

#endif

  // Region de interes.
  Ubicacion ub_min_regint_g_;
  Ubicacion ub_max_regint_g_;

  Ubicacion ubEstSrg_;
  double aptitudUbEstSrg_;

  Ubicacion ubPreSrg_;
  //Ubicacion ub_pred_movil_g_;

  // Coste de las posiciones que constituyen la poblacion.
  std::vector<double> coste_pob_;

  // Poblacion de candidatos a solucion.
  std::vector<Ubicacion> poblacion_;
  std::vector<Ubicacion> pobInicioLoc_;

  // Numero de localizaciones sin exito.
  int numLocSinExito_;

  // Parametros del laser usado.
  // Alcance maximo del laser expresado en metros.
  const float alcance_laser_M_;
  float alcance_laser_C_;
  const double cAnguloMax;
  const double cResAngular;
  const unsigned int cNumMedLaser;
  const float cDist3SigmaM;

  // Variables relacionadas con el numero de medidas laser usadas por el algoritmo.
  // const unsigned int cCoefNumMedLaser;
  // double angulo_max_loc_;
  // double res_angular_loc_;
  const unsigned int cNumMedLaserNorm;
  const unsigned int cNumMedLaserConv;
  unsigned int num_med_laser_act_;

  // Umbral convergencia
  static const float scCoefAptitudLimSup = 4;
  const float cAptitudLimSup;

  //coef_hipotenusa_(1.0 * lado_celda_M_ / cDist3SigmaM), coef_angulo_(
  //        10 * lado_celda_M_ / cDist3SigmaM)
  double coef_hipotenusa_;
  double coef_angulo_;

  std::vector<unsigned int> ind_med_laser_;
  std::vector<double> ang_;

  //unsigned int estadoFinalAnterior_;
  //unsigned int estadoFinal_;
  double intervaloTiempoSegLocSinExito_;

  void actualizarPoblacion(const double & intervaloTiempoLocSeg, const Ubicacion& ub_ant_rr_o,
                           const Ubicacion& ub_act_rr_o, const std::vector<double>& dist_laser,
                           const std::vector<double>& distLaserAnterior);

  double trazadoRayos(const MapaLoc& mapa, const Ubicacion& ub_g, const double& incr_x_ex_g, const double& incr_y_ex_g,
                      const double& dist_inic);

#ifdef DOS_MAPAS
  void trazadoRayos2(const Ubicacion& ub_g, const double& incr_x_ex_g,
      const double& incr_y_ex_g, double& dista, double& distb);
#endif

  double aptitudIndividuo(const Ubicacion& ub_g, const std::vector<double>& dist_laser);
  double aptitudIndividuoConConvergencia(const Ubicacion & ub_g, const std::vector<double> & dist_laser);
  double aptitudIndividuoSinConvergencia(const Ubicacion & ub_g, const std::vector<double> & dist_laser);
  void comprobarLimitesRegionInteres();
  std::vector<std::vector<unsigned int> > crearPermutaciones();
  void crearPoblacion(const std::vector<double>& dist_laser, const unsigned int& num_elem_pob);
  double desplazarPoblacion(const Ubicacion& ub_ant_rr_o, const Ubicacion& ub_act_rr_o,
                            const std::vector<double>& dist_laser);
  void dibujarLaserReal(const unsigned int& a_b, const std::vector<double>& dist_laser, const Magick::Color& color);
  void dibujarPoblacion(const unsigned int& a_b, const std::vector<Ubicacion>& pob, const Magick::Color& color);
  void dibujarPosicion(const unsigned int& a_b, const Ubicacion& ub_g, const Magick::Color& color);
  void dibujarPosicionEstPred(const unsigned int& a_b, Ubicacion ub_est_rr_g, const Magick::Color& color_or,
                              const Magick::Color& color_pos);
  void dibujarRegionInteres(const unsigned int& a_b, const Magick::Color& color);

  inline bool getBanderaLocConExito(void) const
  {
    return bandera_loc_global_exito_;
  }

  inline bool getMapaRecibido(void) const
  {
    return mapa_recibido_;
  }

  inline int getNumeroColumnasMapa(void) const
  {
    return mapa_a_.getColumnas();
  }

  inline int getNumeroFilasMapa(void) const
  {

    return mapa_a_.getFilas();

  }

  inline float getTamanioCeldaM(void) const
  {

    return lado_celda_M_;

  }

  inline Ubicacion getUbicacionEstRobotSRG(void) const
  {
    return ubEstSrg_;
  }

  inline Ubicacion getUbicacionPredRobotSRG(void) const
  {
    return ubPreSrg_;
  }

#ifndef DOS_MAPAS

  inline void guardarImagenes(const std::string& nomb_fich)
  {
    mapa_a_.guardarImagen(nomb_fich);
  }

  inline void limpiarImagenes()
  {
    mapa_a_.limpiarImagen();
  }

#else

  inline void guardarImagenes(const unsigned int& a_b, const std::string& nomb_fich)
  {
    MapaLoc* mapa = &mapa_a_;
    if(a_b)
    {
      mapa = &mapa_b_;
    }
    mapa->guardarImagen(nomb_fich);
  }

  inline void guardarImagenes(const std::string& nomb_fich_a, const std::string& nomb_fich_b)
  {
    mapa_a_.guardarImagen(nomb_fich_a);
    mapa_b_.guardarImagen(nomb_fich_b);
  }

  inline void limpiarImagenes()
  {
    mapa_a_.limpiarImagen();
    mapa_b_.limpiarImagen();
  }

#endif

  void mutarCruzarSeleccionarDescarte(const std::vector<double>& dist_laser);
  unsigned int obtenerIndicePorBusquedaBinaria(const double & elemento, const std::vector<double> & vectorElementos,
                                               const int & ind_inicial, const int & ind_final);

  void obtenerMapa(const nav_msgs::OccupancyGrid::ConstPtr& rp_msj_rejilla_ocupacion_rx);

  inline void roundLoc(const double& x, const double& y, int& x_r, int& y_r)
  {
    x_r = (int)(x + scRound);
    y_r = (int)(y + scRound);
  }

  //inline int sign(float numero)
  //{
  // numero < 0 --> return -1
  // numero = 0 --> numero > 0 es false, false equivale a 0, por tanto return 0.
  // return > 0 --> numero > 0
  //return numero < 0 ? -1 : numero > 0; // # -1, 0, +1.
  //}

};

#endif
