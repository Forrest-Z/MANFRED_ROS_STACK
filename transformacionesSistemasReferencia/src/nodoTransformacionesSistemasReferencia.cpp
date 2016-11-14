#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define DEBUG

int main(int argc, char** argv)
{

  ros::init(argc, argv, "nodoTransformacionesSistemasReferencia");
  ros::NodeHandle nodoManejador;
  tf::TransformBroadcaster publicadorTransformacion;
  tf::Transform transformacionBaseSickS3000;
  tf::Transform transformacionBaseHokuyo;
  tf::Transform transformacionBaseHombro;

#ifdef DEBUG
  ros::Rate frec(5); // Periodo 1/5 = 200 ms por ejemplo.
  double velocidadAngular = (20.0f / 180.0f) * M_PI; // 20 grad/s por ejemplo
  double radioGiro = 1.00; // 1 m de radio de giro por ejemplo
  double distanciaCentroRobotLaser = 0.252;
  ros::Time tiempoInicial = ros::Time::now();

  double angulo = 0.0;
  double dosPI = 2 * M_PI;

  double xb_SRG = radioGiro;
  double yb_SRG = 0;
  double tb_SRG = M_PI / 2;

  double xl_SRG = radioGiro;
  double yl_SRG = 0.252;
  double tl_SRG = M_PI / 2;
#else
  ros::Rate frec(100);
#endif

  std::string x, y, z, qx, qy, qz, qw;

  // TRANSFORMACIONES ESTATICAS

  // Transformacion /sr_base ==> /sr_sicks3000: Se tienen las coordenadas de sist.ref.sicks3000 en el
  // sist.ref.base.
  if (!nodoManejador.hasParam("/xSRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/xSRSicks3000EnSRBase", x);
  if (!nodoManejador.hasParam("/ySRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/ySRSicks3000EnSRBase", y);
  if (!nodoManejador.hasParam("/zSRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/zSRSicks3000EnSRBase", z);
  if (!nodoManejador.hasParam("/qxSRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qxSRSicks3000EnSRBase", qx);
  if (!nodoManejador.hasParam("/qySRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qySRSicks3000EnSRBase", qy);
  if (!nodoManejador.hasParam("/qzSRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qzSRSicks3000EnSRBase", qz);
  if (!nodoManejador.hasParam("/qwSRSicks3000EnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qwSRSicks3000EnSRBase", qw);
  transformacionBaseSickS3000.setRotation(
      tf::Quaternion(atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()), atof(qw.c_str())));
  transformacionBaseSickS3000.setOrigin(tf::Vector3(atof(x.c_str()), atof(y.c_str()), atof(z.c_str())));

  // Transformacion base ==> hokuyo: Se tienen las coordenadas de sist.ref.hokuyo en el
  // sist.ref.base
  if (!nodoManejador.hasParam("/xSRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/xSRHokuyoEnSRBase", x);
  if (!nodoManejador.hasParam("/ySRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/ySRHokuyoEnSRBase", y);
  if (!nodoManejador.hasParam("/zSRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/zSRHokuyoEnSRBase", z);
  if (!nodoManejador.hasParam("/qxSRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qxSRHokuyoEnSRBase", qx);
  if (!nodoManejador.hasParam("/qySRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qySRHokuyoEnSRBase", qy);
  if (!nodoManejador.hasParam("/qzSRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qzSRHokuyoEnSRBase", qz);
  if (!nodoManejador.hasParam("/qwSRHokuyoEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qwSRHokuyoEnSRBase", qw);
  transformacionBaseHokuyo.setRotation(
      tf::Quaternion(atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()), atof(qw.c_str())));
  transformacionBaseHokuyo.setOrigin(tf::Vector3(atof(x.c_str()), atof(y.c_str()), atof(z.c_str())));

  // Transformacion base ==> hombro

  if (!nodoManejador.hasParam("/xSRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/xSRHombroEnSRBase", x);
  if (!nodoManejador.hasParam("/ySRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/ySRHombroEnSRBase", y);
  if (!nodoManejador.hasParam("/zSRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/zSRHombroEnSRBase", z);
  if (!nodoManejador.hasParam("/qxSRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qxSRHombroEnSRBase", qx);
  if (!nodoManejador.hasParam("/qySRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qySRHombroEnSRBase", qy);
  if (!nodoManejador.hasParam("/qzSRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qzSRHombroEnSRBase", qz);
  if (!nodoManejador.hasParam("/qwSRHombroEnSRBase"))
  {
    return -1;
  }
  nodoManejador.getParam("/qwSRHombroEnSRBase", qw);
  transformacionBaseHombro.setRotation(
      tf::Quaternion(atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()), atof(qw.c_str())));
  transformacionBaseHombro.setOrigin(tf::Vector3(atof(x.c_str()), atof(y.c_str()), atof(z.c_str())));

#ifdef DEBUG
  ros::Time marcaTemporal;
#endif

  while (nodoManejador.ok())
  {

#ifdef DEBUG

    marcaTemporal = ros::Time::now();
    angulo = velocidadAngular * (marcaTemporal - tiempoInicial).toSec();
    tb_SRG = angulo + M_PI / 2;
    // Finalmente hacer que el valor de 'tb_SRG' este entre -pi y pi.
    double signoThetaDosPI = (tb_SRG < 0 ? -1 : tb_SRG > 0) * dosPI;
    if (fabs(tb_SRG) > dosPI)
    {
      tb_SRG = fmod(tb_SRG, signoThetaDosPI);
    }
    if (fabs(tb_SRG) > M_PI)
    {
      tb_SRG -= signoThetaDosPI;
    }
    // Ejemplo: El /sr_base y el /sr_sicks3000 dan vueltas alrededor del /sr_mundo con un radio
    // de 'radioGiro' m. Ejecuta en la consola el comando 'rqt', visualiza el arbol TF y veras
    // un divertido movimiento similar al 'corro de la patata'. El ejemplo se completa ejecutando el
    // nodo 'nodoEscuchador' y observando como puedes obtener las coordenadas del /sr_base en el /sr_mundo
    // (/sr_mundo ==> /sr_base) gracias a las transformaciones
    // /sr_mundo ==> /sr_sicks3000 y /sr_sicks3000 ==> /sr_base.
    xb_SRG = radioGiro * cos(angulo);
    yb_SRG = radioGiro * sin(angulo);
    tl_SRG = tb_SRG;
    xl_SRG = xb_SRG + distanciaCentroRobotLaser * cos(tl_SRG);
    yl_SRG = yb_SRG + distanciaCentroRobotLaser * sin(tl_SRG);
    printf("\n");
    printf("%.4f \n", marcaTemporal.toSec());
    printf("Ubicacion  base: (%.3f m, %.3f m, %.3f rad) \n", xb_SRG, yb_SRG, tb_SRG);
    printf("Ubicacion laser: (%.3f m, %.3f m, %.3f rad) \n", xl_SRG, yl_SRG, tl_SRG);
    publicadorTransformacion.sendTransform(
        tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(tb_SRG), tf::Vector3(xb_SRG, yb_SRG, 0.0)),
                             marcaTemporal, "sr_mapa", "sr_base"));
#endif

    // Transformacion base -> sicks3000
    publicadorTransformacion.sendTransform(
        tf::StampedTransform(transformacionBaseSickS3000, ros::Time::now(), "sr_base", "sr_sicks3000"));
    publicadorTransformacion.sendTransform(
        tf::StampedTransform(transformacionBaseHokuyo, ros::Time::now(), "sr_base", "sr_hokuyo"));
    publicadorTransformacion.sendTransform(
        tf::StampedTransform(transformacionBaseHombro, ros::Time::now(), "sr_base", "sr_hombro"));

    frec.sleep();

  }
  return 0;
}
