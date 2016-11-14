#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <Magick++.h>

#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/PoseStamped.h"
#include "localizacionEvolucionDiferencial/localizacionEvolucionDiferencial.h"
#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialAction.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "odometria/servicioOdometria.h"
#include <ros/callback_queue.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sicks3000/servicioSicks3000.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

using std::string;
using std::cout;
using std::cin;
using std::endl;

bool flag_topic_odometria_recibido = false;
bool flag_topic_sicks3000_recibido = false;
bool flag_topic_map_recibido = false;
unsigned int num_loc = 0;
unsigned int num_filas = 0;
unsigned int num_columnas = 0;
nav_msgs::Odometry *p_msj_odometria = NULL;
sensor_msgs::LaserScan *p_msj_barrido_laser = NULL;
nav_msgs::OccupancyGrid *p_msj_rejilla_ocupacion = NULL;
std::vector<std::vector<double> > mapa;

void obtenerBarridoLaser(const sensor_msgs::LaserScan::ConstPtr& rp_msj_barrido_laser_rx)
{
  *p_msj_barrido_laser = *rp_msj_barrido_laser_rx;
  flag_topic_sicks3000_recibido = true;
}

/*
 void obtenerMapa(const nav_msgs::OccupancyGrid::ConstPtr& rp_msj_rejilla_ocupacion_rx)
 {

 num_filas = static_cast<unsigned int>(rp_msj_rejilla_ocupacion_rx->info.height);
 num_columnas = static_cast<unsigned int>(rp_msj_rejilla_ocupacion_rx->info.width);
 std::vector<double> fila_imagen(num_columnas, 0);
 mapa.resize(num_filas, fila_imagen);
 unsigned int inicio = 0;
 cout << "Filas: " << num_filas << endl;
 cout << "Columnas: " << num_columnas << endl;

 for (unsigned int i = 0; i < num_filas; i++)
 {
 for (unsigned int j = 0; j < num_columnas; j++)
 {
 // En ROS el color de gris de cada pixel representa la probabilidad de ocupacion
 // de dicho pixel. Asi, un pixel con RGB (0, 0, 0) tiene probabilidad de ocupacion 1.
 // Asi, un pixel negro, con RGB (0, 0, 0) tiene probabilidad de ocupacion 1.
 // Asi, un pixel blanco, con RGB (255, 255, 255) tiene probabilidad de ocupacion 0.
 // En cambio yo represento la probabilidad de que un pixel este libre. De modo
 // que para mi un pixel negro, con RGB (0, 0, 0) tiene probabilidad de estar libre 0,
 // y un pixel blanco, con RGB (255, 255, 255) tiene probabilidad de estar libre 1.
 // Como se puede ver interpretamos la info de manera distinta. Para ello
 // tengo que restarle a 1 la probabilidad del mapa en ROS, asi paso
 // los datos a mi interpretacion.

 // OJO EL CAMPO NEGATE DE LOS FICHEROS YAML DEJARLO A VALOR 0. NO PONER A 1 NUNCA.
 fila_imagen[j] = 1 - (rp_msj_rejilla_ocupacion_rx->data[inicio + j] / 100.0);
 }
 inicio += num_columnas;
 mapa[i].assign(fila_imagen.begin(), fila_imagen.end());
 }

 tamanio_celda_M = rp_msj_rejilla_ocupacion_rx->info.resolution;
 flag_topic_map_recibido = true;


 for (unsigned i = 0; i < num_filas; i++)
 {
 for (unsigned j = 0; j < num_columnas; j++)
 {
 cout << mapa[i][j] << " ";
 }
 cout << endl;
 }


 }*/

void obtenerOdometria(const nav_msgs::Odometry::ConstPtr& rp_msj_odometria_rx)
{

  *p_msj_odometria = *rp_msj_odometria_rx;
  flag_topic_odometria_recibido = true;

}

bool ubicacionRobotSisRefGlobal(
    actionlib::SimpleActionServer<manfred_arm_msgs::LocalizacionEvolucionDiferencialAction> * servidorAccion,
    manfred_arm_msgs::LocalizacionEvolucionDiferencialResult * resultado)
{

  servidorAccion->setSucceeded((*resultado), "Localizacion global con ED servida");

#ifdef DEBUG

  cout << endl;
  cout << "Localizacion global con ED servida" << endl;

#endif

  return true;

}

int main(int argc, char* argv[])
{

  // Semilla para numeros aleatorios.
  srand(time(NULL));
  srand48(time(NULL));
  MagickCore::InitializeMagick(*argv);
  ros::init(argc, argv, "nodoLocalizacionEvolucionDiferencial");
  ros::NodeHandle nodoManejador;

  ros::Subscriber suscriptorTopicOdometria = nodoManejador.subscribe("topic_odometria_OK", 1, obtenerOdometria);
  nav_msgs::Odometry msj_odometria;
  p_msj_odometria = &msj_odometria;

  ros::Subscriber suscriptorSicks3000 = nodoManejador.subscribe("topic_sicks3000_OK", 1, obtenerBarridoLaser);
  sensor_msgs::LaserScan msj_barrido_laser;
  p_msj_barrido_laser = &msj_barrido_laser;

  tf::TransformListener escuchadorTransformacion;
  tf::StampedTransform transformacion;

  // Numero de componentes de cada individuo.
  const unsigned int cEvDifD = 3;
  // Estrategia de mutacion.
  const unsigned int cEvDifST = 1;

  string parametro;

  if (!nodoManejador.hasParam("/TasaCruce"))
  {
    return -1;
  }
  nodoManejador.getParam("/TasaCruce", parametro);
  const float cEvDifCR = atof(parametro.c_str());

  if (!nodoManejador.hasParam("/FactorAtenuacion"))
  {
    return -1;
  }
  nodoManejador.getParam("/FactorAtenuacion", parametro);
  const float cEvDifF = atof(parametro.c_str());

  if (!nodoManejador.hasParam("/DistanciaTraslacionC"))
  {
    return -1;
  }
  nodoManejador.getParam("/DistanciaTraslacionC", parametro);
  const float cDistanciaTraslacionC = atof(parametro.c_str());

  if (!nodoManejador.hasParam("/LadoConvergenciaM"))
  {
    return -1;
  }
  nodoManejador.getParam("/LadoConvergenciaM", parametro);
  const float cDistConvM = atof(parametro.c_str());

  if (!nodoManejador.hasParam("/NumeroElementosPoblacion"))
  {
    return -1;
  }
  nodoManejador.getParam("/NumeroElementosPoblacion", parametro);
  const unsigned int cNumElemPobNorm = static_cast<unsigned int>(atoi(parametro.c_str()));

  if (!nodoManejador.hasParam("/NumeroElementosPoblacionConvergencia"))
  {
    return -1;
  }
  nodoManejador.getParam("/NumeroElementosPoblacionConvergencia", parametro);
  const unsigned int cNumElemPobConv = static_cast<unsigned int>(atoi(parametro.c_str()));

  if (!nodoManejador.hasParam("/NumeroIteraciones"))
  {
    return -1;
  }
  nodoManejador.getParam("/NumeroIteraciones", parametro);
  const unsigned int cNumIterNorm = static_cast<unsigned int>(atoi(parametro.c_str()));

  if (!nodoManejador.hasParam("/NumeroIteracionesConvergencia"))
  {
    return -1;
  }
  nodoManejador.getParam("/NumeroIteracionesConvergencia", parametro);
  const unsigned int cNumIterConv = static_cast<unsigned int>(atoi(parametro.c_str()));

  // DATOS DEL LASER SICKS3000
  if (!nodoManejador.hasParam("/DistanciaMedibleMaximaM"))
  {
    return -1;
  }
  nodoManejador.getParam("/DistanciaMedibleMaximaM", parametro);
  const float cAlcanceLaserM = atof(parametro.c_str());

  if (!nodoManejador.hasParam("/AnguloBarridoMaximoG"))
  {
    return -1;
  }
  nodoManejador.getParam("/AnguloBarridoMaximoG", parametro);
  const float cAnguloMax = atof(parametro.c_str()) / 2;

  if (!nodoManejador.hasParam("/ResolucionAngularG"))
  {
    return -1;
  }
  nodoManejador.getParam("/ResolucionAngularG", parametro);
  const float cResAngular = atof(parametro.c_str());

  if (!nodoManejador.hasParam("/DistanciaTresSigmaM"))
  {
    return -1;
  }
  nodoManejador.getParam("/DistanciaTresSigmaM", parametro);
  const float cDist3SigmaM = atof(parametro.c_str());

  // El laser Sick S3000 proporciona muchas medidas, en concreto, si la resolucion usada es de
  // 0'25 grados, dado que este laser abarca un angulo 190 grados, se dispone de 761 medidas
  const unsigned int num_med_laser_tot = static_cast<unsigned int>(((2 * cAnguloMax) / cResAngular) + 1);

  // En el algoritmo no necesitamos tanta resolucion angular. Es suficiente con usar una medida
  // laser cada 2'5 grados, o 3 grados. Por lo tanto se hace necesario conocer aquellas medidas
  // que nos interesan de entre las 761 que proporciona el Sick S3000.
  // La variable 'coef_incremento' nos indica el numero de sectores angulares de 0'25 grados
  // que conforman la resolucion angular del algoritmo. Por ejemplo si 'coef_incr = 12'
  // la resolucion angular del algoritmo es 'res_angular_loc_ = 12 * 0'25 = 3 grados'.
  // 0.25 grad -->> coef_incr = 1 -->> 761 med_laser  -->> 190º
  // 0.50 grad -->> coef_incr = 2 -->> 381 med_laser  -->> 190º
  // 0.75 grad -->> coef_incr = 3 -->> 253 med_laser  -->> 189º
  // 1.00 grad -->> coef_incr = 4 -->> 191 med_laser  -->> 190º
  // 1.25 grad -->> coef_incr = 5 -->> 153 med_laser  -->> 190º
  // 1.50 grad -->> coef_incr = 6 -->> 127 med_laser  -->> 189º
  // 1.75 grad -->> coef_incr = 7 -->> 109 med_laser  -->> 189º
  // 2.00 grad -->> coef_incr = 8 -->>  95 med_laser  -->> 188º
  // 2.25 grad -->> coef_incr = 9 -->>  85 med_laser  -->> 189º
  // 2.50 grad -->> coef_incr = 10 -->> 77 med_laser  -->> 190º
  // 2.75 grad -->> coef_incr = 11 -->> 69 med_laser  -->> 187º
  // 3.00 grad -->> coef_incr = 12 -->> 63 med_laser  -->> 186º
  if (!nodoManejador.hasParam("/CoeficienteIncrementoHazLaser"))
  {
    return -1;
  }
  nodoManejador.getParam("/CoeficienteIncrementoHazLaser", parametro);
  const unsigned int coef_incr = static_cast<unsigned int>(atoi(parametro.c_str()));

  if (!nodoManejador.hasParam("/CoeficienteIncrementoHazLaserConvergencia"))
  {
    return -1;
  }
  nodoManejador.getParam("/CoeficienteIncrementoHazLaserConvergencia", parametro);
  const unsigned int coef_incr_conv = static_cast<unsigned int>(atoi(parametro.c_str()));

  // El algoritmo de localizacion original trabaja con dos mapas, uno
  // con las puertas abiertas y otro con las puertas cerradas. Sin embargo
  // como considero que no siempre se va a contar con ambos mapas, solamente con
  // uno, el que construyas haciendo SLAM, he modificado el algoritmo para
  // que trabaje con un solo mapa.
  // Aun asi he dejado en el dichero 'localizacionEvolucionDiferencial.cpp' el
  // codigo necesario para que si alguien lo necesite, use el algoritmo con
  // dos mapas. Para conseguir esto descomentar la instruccion #define DOS_MAPAS

  // Los dos mapas pueden ser iguales o distintos.
  // Si se trabaja con dos mapas iguales, ambos seran 'abiertos' o 'cerrados'.
  // Si se trabaja con dos mapas distintos, el mapa abierto debe guardarse
  // en 'mapa_a_' y el mapa cerrado debe guardarse en 'mapa_b_'.
  if (!nodoManejador.hasParam("/NombreMapa"))
  {
    return -1;
  }
  nodoManejador.getParam("/NombreMapa", parametro);
  const std::string nombre_mapa = parametro;

  // ************************************
  // * SIEMPRE SE TRABAJA CON DOS MAPAS *
  // ************************************
  // Los dos mapas pueden ser iguales o distintos.
  // Si se trabaja con dos mapas iguales, ambos seran 'abiertos' o 'cerrados'.
  // Si se trabaja con dos mapas distintos, el mapa abierto debe guardarse
  // en 'mapa_a_' y el mapa cerrado debe guardarse en 'mapa_b_'.

#ifndef DOS_MAPAS

  if (!nodoManejador.hasParam("/NombreMapa"))
  {
    return -1;
  }
  nodoManejador.getParam("/NombreMapa", parametro);
  const std::string nomb_fich_a = parametro;
  const std::string nomb_fich_b = nomb_fich_a;

#else

  const std::string nomb_fich_a = "Un_mapa_abierto.bmp";
  const std::string nomb_fich_b = "Un_mapa_cerrado.bmp";

#endif

  Magick::Color laserRojo("#FF0000");
  Magick::Color colorPobIniLoc("#D580ED");
  Magick::Color colorPobFinLoc("#00FF00");
  Magick::Color colorPobFinLocDesp("#0000FF");
  Magick::Color colorRegInt("#FF000044");
  //Magick::Color color_reg_busq_fin_loc_despl(0, 0, MaxRGB, 0.8 * MaxRGB);
  Magick::Color colorUbEstSrg("#FF8000");
  Magick::Color colorUbPreSrg("#FFFF00");

  std::vector<Ubicacion> pobFinLoc;
  Ubicacion ubEstCopia;
  ros::Time tiempo_1;
  ros::Time tiempo_2;
  double intervaloTiempoSeg = 0;

  bool transformacionEscuchada = false;
  while (!transformacionEscuchada)
  {
    try
    {
      escuchadorTransformacion.lookupTransform("/link_laser", "/link_base", ros::Time(0), transformacion);
      transformacionEscuchada = true;

#ifdef DEBUG
      cout << "x_M.Base_en_M.Laser " << transformacion.getOrigin().x() << " m " << endl;
      cout << "y_M.Base_en_M.Laser " << transformacion.getOrigin().y() << " m " << endl;
      cout << "z_M.Base_en_M.Laser " << transformacion.getOrigin().z() << " m " << endl;
      cout << "t_M.Base_en_M.Laser " << LocED::scGradRad * tf::getYaw(transformacion.getRotation()) << " grad " << endl;
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

  // Region de interes en cuyo interior se haya el robot inicialmente, aunque se
  // desconoce su ubicacion exacta.
  Ubicacion ub_min_regint_g(0.0, 0.0, 0.0);
  Ubicacion ub_max_regint_g(num_columnas, num_filas, 2 * M_PI);

  // Nombre para guardar imagenes que pinto.
  std::stringstream nombre_fich_pob_a;

  // Crear el objeto 'algoritmo differential evolution'
  LocED loced(cEvDifD, cEvDifST, cEvDifCR, cEvDifF, cDistanciaTraslacionC, cDistConvM, cNumElemPobNorm, cNumElemPobConv,
              cNumIterNorm, cNumIterConv, nomb_fich_a, nomb_fich_b, cAlcanceLaserM, cAnguloMax, cResAngular,
              num_med_laser_tot, cDist3SigmaM, coef_incr, coef_incr_conv);

  ros::Subscriber suscriptorMapa = nodoManejador.subscribe("map", 1, &LocED::obtenerMapa, &loced);
  nav_msgs::OccupancyGrid msj_rejilla_ocupacion;
  p_msj_rejilla_ocupacion = &msj_rejilla_ocupacion;

  // Recibir el mapa.
  do
  {
    ros::spinOnce();
  } while (!loced.getMapaRecibido());

  float tamanio_celda_M = loced.getTamanioCeldaM();

  manfred_arm_msgs::LocalizacionEvolucionDiferencialResult resultado;

  actionlib::SimpleActionServer<manfred_arm_msgs::LocalizacionEvolucionDiferencialAction> servidorAccion(
      nodoManejador, "servidorAccionLocalizacionDE",
      boost::bind(&ubicacionRobotSisRefGlobal, &servidorAccion, &resultado), false);

  servidorAccion.start();

  // El laser de Manfred hace un barrio en el entorno.
  // Se obtiene la ubicacion del M.Base en el M.Odom.
  do
  {
    ros::spinOnce();
  } while (!flag_topic_odometria_recibido || !flag_topic_sicks3000_recibido);
  flag_topic_odometria_recibido = false;
  flag_topic_sicks3000_recibido = false;

  Ubicacion ub_ant_mbase_modom(-1, -1, -1);
  Ubicacion ub_act_mbase_modom(-1, -1, -1);

  ub_act_mbase_modom.setXYTheta(msj_odometria.pose.pose.position.x / tamanio_celda_M,
                                msj_odometria.pose.pose.position.y / tamanio_celda_M,
                                tf::getYaw(msj_odometria.pose.pose.orientation));

  cout << "Odometria recibida" << endl;
  cout << "x_act_M.Base_en_M.Odom " << ub_act_mbase_modom.getX() << " m " << endl;
  cout << "y_act_M.Base_en_M.Odom " << ub_act_mbase_modom.getY() << " m " << endl;
  cout << "t_act_M.Base_en_M.Odom " << LocED::scGradRad * ub_act_mbase_modom.getTheta() << " grad " << endl;

  std::vector<double> dist_laser(num_med_laser_tot, 0);
  dist_laser.assign(msj_barrido_laser.ranges.begin(), msj_barrido_laser.ranges.end());

  cout << "Barrido del Sick S3000 recibido" << endl;
  cout << msj_barrido_laser.header.frame_id << endl;
  cout << LocED::scGradRad * msj_barrido_laser.angle_min << " grad " << endl;
  cout << LocED::scGradRad * msj_barrido_laser.angle_max << " grad " << endl;
  cout << LocED::scGradRad * msj_barrido_laser.angle_increment << " grad " << endl;
  cout << msj_barrido_laser.range_min << " m " << endl;
  cout << msj_barrido_laser.range_max << " m " << endl;

  // for (unsigned int i = 0; i < dist_laser.size(); i++)
  // {
  //  cout << i << "  " << dist_laser[i] << " m " << endl;
  // }

  loced.crearPoblacion(dist_laser, cNumElemPobNorm);

  // Escoger como ubicacion predicha un individuo cualquiera de la poblacion.
  // La siguiente operacion proporciona un numero comprendido entre '0' y 'loced.num_elem_pob_act_' - 1.
  unsigned int i_indiv = rand() % loced.num_elem_pob_act_;
  loced.ubPreSrg_.setXYTheta(loced.poblacion_[i_indiv].getX(), loced.poblacion_[i_indiv].getY(),
                             loced.poblacion_[i_indiv].getTheta());

  loced.limpiarImagenes();
  loced.dibujarRegionInteres(0, colorRegInt);
  loced.dibujarPoblacion(0, loced.poblacion_, colorPobFinLoc);
  loced.dibujarPosicionEstPred(0, loced.ubPreSrg_, colorUbPreSrg, colorUbPreSrg);
  nombre_fich_pob_a.str("");
  nombre_fich_pob_a << "INICIO_POBLACION.png";

#ifndef DOS_MAPAS

  loced.guardarImagenes(nombre_fich_pob_a.str());

#else

  loced.guardarImagenes(0, nombre_fich_pob_a.str());

#endif

  double anguloSRG = 0.0;
  Ubicacion ub;

  do
  {
    tiempo_1 = ros::Time::now();
    // EVOLUCION DIFERENCIAL.
    loced.mutarCruzarSeleccionarDescarte(dist_laser);
    tiempo_2 = ros::Time::now();
    intervaloTiempoSeg = (tiempo_2 - tiempo_1).toSec();
    ++num_loc;

#ifdef DEBUG

    cout << endl;
    cout << "\x1B[31;1m";
    cout << "Evolucion diferencial " << num_loc << " [" << intervaloTiempoSeg << " seg] " << endl;
    cout << "\x1B[0m";

    ub = loced.getUbicacionEstRobotSRG();
    // El signo menos:
    // El sistema de referencia del algoritmo de localizacion es la esquina superior izquierda,
    // el habitual al trabajar con imagenes. El sistema de referencia global es el habitual
    // al trabajar con funciones, es decir, la esquina inferior izquierda. Para transformar
    // los angulos del primer sistema de referencia al segundo hay que usar un signo menos.
    anguloSRG = -ub.getTheta();
    Ubicacion::orientacionMenosPiMasPi(anguloSRG);

    cout << endl;
    cout << "\x1B[32;1m";
    cout << "x_est_M.Base_en_M.Mapa: " << tamanio_celda_M * ub.getX() << " m " << endl;
    cout << "y_est_M.Base_en_M.Mapa: " << tamanio_celda_M * (loced.getNumeroFilasMapa() - ub.getY()) << " m " << endl;
    cout << "t_est_M.Base_en_M.Mapa: " << LocED::scGradRad * anguloSRG << " grad " << endl;
    cout << "Coste minimo: " << loced.aptitudUbEstSrg_ << endl;
    cout << "\x1B[0m";

    loced.limpiarImagenes();
    loced.dibujarRegionInteres(0, colorRegInt);
    loced.dibujarPoblacion(0, loced.pobInicioLoc_, colorPobIniLoc);
    loced.dibujarPoblacion(0, loced.poblacion_, colorPobFinLoc);
    loced.dibujarPosicionEstPred(0, loced.getUbicacionPredRobotSRG(), colorUbPreSrg, colorUbPreSrg);
    loced.dibujarPosicionEstPred(0, loced.getUbicacionEstRobotSRG(), colorUbEstSrg, colorUbEstSrg);
    loced.dibujarLaserReal(0, dist_laser, laserRojo);
    nombre_fich_pob_a.str("");
    nombre_fich_pob_a << "RESULT_LOC_AAP_" << num_loc << ".png";

    // El signo menos:
    // El sistema de referencia del algoritmo de localizacion es la esquina superior izquierda, el habitual
    // al trabajar con imagenes.
    // El sistema de referencia global es el habitual al trabajar con funciones, es decir, la esquina inferior
    // izquierda. Para transformar los angulos del primer sistema de referencia al segundo hay
    // que usar un signo menos.
    ub.setTheta(-ub.getTheta());
#endif

#ifndef DOS_MAPAS

    loced.guardarImagenes(nombre_fich_pob_a.str());

#else

    loced.guardarImagenes(0, nombre_fich_pob_a.str());

#endif

    // Averiguar cuanto se ha desplazado/girado el robot real desde la
    // anterior ubicacion, expresada en el sistema de
    // ref.odom, hasta la nueva posicion, expresada tambien
    // en dicho sist.de referencia.
    // Salvar la ubicacion de interes anterior que ocupo el robot real
    // expresada en el sistema de referencia odometrico.
    ub_ant_mbase_modom = ub_act_mbase_modom;

    // Desde la nueva ubicacion de interes, expresada en el sist.ref.odom,
    // el robot real toma un barrido laser.
    std::vector<double> distLaserAnterior = dist_laser;

    do
    {
      ros::spinOnce();
    } while (!flag_topic_odometria_recibido || !flag_topic_sicks3000_recibido);

    flag_topic_odometria_recibido = false;
    flag_topic_sicks3000_recibido = false;

    ub_act_mbase_modom.setXYTheta(msj_odometria.pose.pose.position.x / tamanio_celda_M,
                                  msj_odometria.pose.pose.position.y / tamanio_celda_M,
                                  tf::getYaw(msj_odometria.pose.pose.orientation));
    /*
     cout << "Odometria recibida" << endl;
     cout << "x_act_M.Base_en_M.Odom " << ub_act_mbase_modom.getX() << " m "
     << endl;
     cout << "y_act_M.Base_en_M.Odom " << ub_act_mbase_modom.getY() << " m "
     << endl;
     cout << "t_act_M.Base_en_M.Odom "
     << LocED::scGradRad * ub_act_mbase_modom.getTheta() << " grad "
     << endl;
     */

    dist_laser.assign(msj_barrido_laser.ranges.begin(), msj_barrido_laser.ranges.end());

    pobFinLoc.assign(loced.poblacion_.begin(), loced.poblacion_.end());
    ubEstCopia = loced.getUbicacionEstRobotSRG();
    // Actualizar la poblacion de acuerdo al desplazamiento realizado por el robot para
    // llegar desde la ubicacion de interes anterior a la nueva ubicacion de interes.
    loced.actualizarPoblacion(intervaloTiempoSeg, ub_ant_mbase_modom, ub_act_mbase_modom, dist_laser,
                              distLaserAnterior);
    // El sistema de referencia global en el algoritmo de localizacion esta situado en la esquina superior
    // izquierda, como es habitual al trabajar con imágenes, siguiendo el convenio de la regla de la mano derecha.
    // Por contrario, el sistema de referencia global del mapa esta situado en la esquina inferior izquierda, como
    // es habitual cuando se trabajan con funciones o planos, siguiendo tambien el convenio de la regla
    // de la mano derecha. Por tanto hay que transformar las coordenadas de la ubicacion del robot, del primero de
    // los sistemas de referencia al segundo.
    resultado.ubicacionRobotSRG.pose.position.x = tamanio_celda_M * loced.getUbicacionPredRobotSRG().getX();
    resultado.ubicacionRobotSRG.pose.position.y = tamanio_celda_M
        * (loced.getNumeroFilasMapa() - loced.getUbicacionPredRobotSRG().getY());
    anguloSRG = -loced.getUbicacionPredRobotSRG().getTheta();
    Ubicacion::orientacionMenosPiMasPi(anguloSRG);
    resultado.ubicacionRobotSRG.pose.orientation.w = tf::createQuaternionFromYaw(anguloSRG).getW();
    resultado.ubicacionRobotSRG.pose.orientation.x = tf::createQuaternionFromYaw(anguloSRG).getX();
    resultado.ubicacionRobotSRG.pose.orientation.y = tf::createQuaternionFromYaw(anguloSRG).getY();
    resultado.ubicacionRobotSRG.pose.orientation.z = tf::createQuaternionFromYaw(anguloSRG).getZ();

    cout << endl;
    cout << "\x1B[35;1m";
    cout << "x_pred_M.Laser_en_M.Mapa: " << resultado.ubicacionRobotSRG.pose.position.x << " m" << endl;
    cout << "y_pred_M.Laser_en_M.Mapa: " << resultado.ubicacionRobotSRG.pose.position.y << " m" << endl;
    cout << "t_pred_M.Laser_en_M.Mapa (GRAD): " << LocED::scGradRad * anguloSRG << " grad" << endl;
    cout << "\x1B[0m";
    /*
     int bandera_stop = 0;
     cout << "INTRODUCE UN NUMERO PARA SEGUIR: ";
     std::cin >> bandera_stop;
     */
    ros::spinOnce();

    loced.limpiarImagenes();
    loced.dibujarRegionInteres(0, colorRegInt);
    loced.dibujarPoblacion(0, pobFinLoc, colorPobFinLoc);
    loced.dibujarPoblacion(0, loced.poblacion_, colorPobFinLocDesp);
    loced.dibujarPosicionEstPred(0, ubEstCopia, colorUbEstSrg, colorUbEstSrg);
    loced.dibujarPosicionEstPred(0, loced.getUbicacionPredRobotSRG(), colorUbPreSrg, colorUbPreSrg);
    nombre_fich_pob_a.str("");
    nombre_fich_pob_a << "RESULT_LOC_DAP_" << num_loc << ".png";

#ifndef DOS_MAPAS

    loced.guardarImagenes(nombre_fich_pob_a.str());

#else

    loced.guardarImagenes(0, nombre_fich_pob_a.str());

#endif

    //int bandera_stop = 0;
    //cout << "INTRODUCE UN NUMERO PARA SEGUIR: ";
    //std::cin >> bandera_stop;

    cout << endl;
    cout << "===== ===== ===== ===== =====" << endl;

  } while (1);

  return 0;
}
