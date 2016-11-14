#include "ros/ros.h"
#include <tinyxml.h>

#define DEBUG

void guardarParametrosEnMaster(ros::NodeHandle* nh, TiXmlNode* pParent)
{
  if (!pParent)
  {
    return;
  }
  TiXmlNode* pChild = NULL;
  TiXmlText* pText = NULL;
  int tipo = pParent->Type();
  char* elemento = NULL;
  if (tipo == TiXmlNode::TINYXML_ELEMENT)
  {
    elemento = (char*)calloc(1 + strlen(pParent->Value()) + 1, sizeof(char));
    strcat(elemento, "/");
    strcat(elemento, pParent->Value());
#ifdef DEBUG
    printf("%s: %s \n", elemento, pParent->FirstChild()->ToText()->Value());
#endif
    nh->setParam(elemento, pParent->FirstChild()->ToText()->Value());
    free(elemento);
  }
  else
  {
    for (pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
      guardarParametrosEnMaster(nh, pChild);
    }
  }
}

int main(int argc, char* argv[])
{
  // NO HAY QUE LIBERAR ESTE PUNTERO!
  char* rutaROS = getenv("ROS_WORKSPACE");
  if (rutaROS == NULL)
  {
    printf("\x1B[31;1m");
    printf("La ruta hasta el sistema de ficheros de ROS (personal) no existe \n");
    printf("\x1B[0m");
    return -1;
  }
  ros::init(argc, argv, "nodoConfiguracionRobot");
  ros::NodeHandle nodoManejador;
  char* nombreFicheroConfiguracion =
      new char[strlen(rutaROS) + strlen("/configuracionRobot/configuracionRobot.xml") + 1]();
  strcat(nombreFicheroConfiguracion, rutaROS);
  strcat(nombreFicheroConfiguracion, "/configuracionRobot/configuracionRobot.xml");
#ifdef DEBUG
  printf("Ruta del sistema de ficheros de ROS (personal): %s \n", rutaROS);
  printf("Ruta del fichero de configuracion del robot:    %s \n", nombreFicheroConfiguracion);
#endif
  TiXmlDocument ficheroConfiguracion(nombreFicheroConfiguracion);
  bool exitoLectura = ficheroConfiguracion.LoadFile();
  if (!exitoLectura)
  {
    printf("\x1B[31;1m");
    printf("Error al leer el fichero: %s \n", nombreFicheroConfiguracion);
    printf("\x1B[0m");
    return -1;
  }
  guardarParametrosEnMaster(&nodoManejador, &ficheroConfiguracion);
  delete[] nombreFicheroConfiguracion;
  return 0;
}
