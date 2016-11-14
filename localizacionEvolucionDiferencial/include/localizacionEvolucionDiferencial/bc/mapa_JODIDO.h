/*
 * mapaLoc.h
 *
 *  Created on: Jul 22, 2011
 *      Author: jfrascon
 */

#ifndef MAPALOC_H_
#define MAPALOC_H_

#include <cmath>
#include <string>
#include <vector>
#include <Magick++.h>
#include "ubicacion.h"

class MapaLoc
{

public:

  MapaLoc(const unsigned int& num_filas, const unsigned int& num_columnas, const std::string& nombre_fich,
                   const std::vector<std::vector<double> >& matriz_probabilidad);

  inline MapaLoc(const MapaLoc& mapa) :
      filas_(mapa.filas_), columnas_(mapa.columnas_), nombre_fich_(mapa.nombre_fich_), img_mapa_orig_(
          mapa.img_mapa_orig_), img_mapa_modif_(mapa.img_mapa_orig_), matriz_probabilidad_ocupacion_(mapa.matriz_probabilidad_ocupacion_)
  {
  }

  ~MapaLoc()
  {
  }

  inline unsigned int getFilas() const
  {
    return filas_;
  }
  inline unsigned int getColumnas() const
  {
    return columnas_;
  }
  inline std::string getNombreFichero() const
  {
    return nombre_fich_;
  }
  inline std::vector<std::vector<double> > getMatrizProbabilidadOcupacion() const
  {
    return matriz_probabilidad_ocupacion_;
  }
  inline void setFilas(const unsigned int & filas)
  {
    filas_ = filas;
  }
  inline void setColumnas(const unsigned int & columnas)
  {
    columnas_ = columnas;
  }
  inline void setNombreFichero(const std::string & nombre_fich)
  {
    nombre_fich_ = nombre_fich;
  }
  inline void setMatrizProbabilidadOcupacion(const std::vector<std::vector<double> >& matriz)
  {
    matriz_probabilidad_ocupacion_ = matriz;
  }
  void dibujarAreaCuadrada(const unsigned int & x_min, const unsigned int & y_min, const unsigned int & x_max,
                           const unsigned int & y_max, const Magick::Color & color);

  void dibujarOrientacion(const unsigned int & x_i, const unsigned int & y_i, const unsigned int & x_f,
                          const unsigned int & y_f, const Magick::Color & color);
  void guardarImagen(const std::string& nombre_fich);
  void limpiarImagen();
  inline void obtenerDimensiones(unsigned int & filas, unsigned int & columnas) const
  {
    filas = filas_;
    columnas = columnas_;
  }
  inline void obtenerDimensiones(double & filas, double & columnas) const
  {
    filas = filas_;
    columnas = columnas_;
  }
  void obtenerImagesParaDibujar();
  inline float obtenerProbabilidadOcupacion(const unsigned int & x, const unsigned int & y) const
  {
    // No hay que comprobar que el valor de 'x' e 'y' sea mayor o igual que
    // cero y menor o igual que el numero de columnas y filas, respectivamente, del mapa,
    // siempre y cuando se garantice, fuera de esta funcion, que estas
    // coordenadas cumplen estos criterios.
    return matriz_probabilidad_ocupacion_[y][x];
  }
  void pintarPixel(const unsigned int & x, const unsigned int & y, const Magick::Color & color);
private:
  // Numero de filas de cada una de las matrices de probabilidad.
  unsigned int filas_;
  // Numero de columnas de cada una de las matrices de probabilidad.
  unsigned int columnas_;
  // Nombre del fichero del que se obtiene la matriz de probabilidad
  std::string nombre_fich_;
  Magick::Image img_mapa_orig_;
  Magick::Image img_mapa_modif_;
  // Matriz que contiene en cada posicion la probabilidad de que esa celda en el
  // mapa abierto este libre.
  std::vector<std::vector<double> > matriz_probabilidad_ocupacion_;
};

#endif /* MAPALOC_H_ */
