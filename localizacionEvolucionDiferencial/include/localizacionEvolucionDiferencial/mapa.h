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

  MapaLoc(const std::string& nombre_fich);

  MapaLoc(const MapaLoc& mapa) :
      filas_(mapa.filas_), columnas_(mapa.columnas_), nombre_fich_(mapa.nombre_fich_), img_mapa_orig_(
          mapa.img_mapa_orig_), img_mapa_modif_(mapa.img_mapa_modif_), matriz_probabilidad_ocupacion_(
          mapa.matriz_probabilidad_ocupacion_)
  {

  }

  inline void dibujarAreaCuadrada(const unsigned int& x_min, const unsigned int& y_min, const unsigned int& x_max,
                                  const unsigned int& y_max, const Magick::Color& color)
  {
    img_mapa_modif_.strokeColor(color);
    img_mapa_modif_.fillColor(color);
    Magick::DrawableRectangle rect(x_min, y_min, x_max, y_max);
    img_mapa_modif_.draw(rect);

  }

  inline void dibujarOrientacion(const unsigned int& x_i, const unsigned int& y_i, const unsigned int& x_f,
                                 const unsigned int& y_f, const Magick::Color& color)
  {
    img_mapa_modif_.strokeColor(color);
    img_mapa_modif_.draw(Magick::DrawableLine(x_i, y_i, x_f, y_f));
  }

  inline unsigned int getFilas() const
  {
    return filas_;
  }

  inline unsigned int getColumnas() const
  {
    return columnas_;
  }

  inline std::string getNombreFich() const
  {
    return nombre_fich_;
  }

  inline std::vector<std::vector<double> > getMatrizProbabilidadOcupacion() const
  {
    return matriz_probabilidad_ocupacion_;
  }

  inline void guardarImagen(const std::string& nombre_fich)
  {
    img_mapa_modif_.write(nombre_fich);

  }

  inline void limpiarImagen()
  {
    // Resetear la imagen que podemos modificar.
    img_mapa_modif_ = Magick::Image(img_mapa_orig_);

  }

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

  bool obtenerImagenMapa();

  inline float obtenerProbabilidadOcupacion(const unsigned int & x, const unsigned int & y) const
  {
    // No hay que comprobar que el valor de 'x' e 'y' sea mayor o igual que
    // cero y menor o igual que el numero de columnas y filas, respectivamente, del mapa,
    // siempre y cuando se garantice, fuera de esta funcion, que estas
    // coordenadas cumplen estos criterios.
    return matriz_probabilidad_ocupacion_[y][x];
  }

  inline void setFilas(const unsigned int & filas)
  {
    filas_ = filas;
  }

  inline void setColumnas(const unsigned int & columnas)
  {
    columnas_ = columnas;
  }

  inline void setMatrizProbabilidadOcupacion(const std::vector<std::vector<double> > & matriz)
  {
    matriz_probabilidad_ocupacion_ = matriz;
  }

  inline void setNombreFich(const std::string & nombre_fich)
  {
    nombre_fich_ = nombre_fich;
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
