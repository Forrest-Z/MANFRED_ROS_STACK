/*
 * mapaLoc.cpp
 *
 *  Created on: Jul 29, 2011
 *      Author: jfrascon
 */

#include <cmath>

#include <iostream>
#include <string>
#include <vector>

#include "localizacionEvolucionDiferencial/mapa.h"

MapaLoc::MapaLoc(const std::string& nombre_fich) :
    filas_(0), columnas_(0), nombre_fich_(nombre_fich)
{
  std::cout << nombre_fich_ << std::endl;
  obtenerImagenMapa();
}

bool MapaLoc::obtenerImagenMapa()
{
  /*
   Magick::Quantum azul = 0;
   Magick::Quantum rojo = 0;
   Magick::Quantum verde = 0;
   // Una columna de la matriz representa una coordenada x.
   // Una fila de la matriz representa una coordenada y.
   unsigned int colCordX = 0;
   unsigned int filCordY = 0;
   */
  // MaxRGB = 65535 --> Si se duda de esto comprobar con printf("%d.\n", MaxRGB).
  // El valor de la componente R, G y B, normalmente suele estar comprendido entre
  // 0 y 255. Por lo tanto el  valor de cada coordenada (R, G, N) puede expresarse con 8 bits.
  // De este modo cualquier color puede expresarse con 24 bits (8 bits adicionales si se
  // quiere expresar tambien el canal alfa --> transparencia). Con 24 bits se pueden obtener
  // 2^24 colores distintos.
  // La libreria Magick++ usa 16 bits para cada componente (R, G y B). Por lo tanto
  // cada componente tiene un valor comprendido entre 0 y 2^16 - 1= 65535.
  // El valor 127 en la escala 0~255 se convierte en 32639 en la escala 0~65535
  // 127/255 * 65535 = 32639
  //Magick::Quantum colorMedio = Magick::Color::scaleDoubleToQuantum(0.5);
  // El valor 128 en la escala 0~255 se convierte en 32896 en la escala 0~65535
  // 128/255 * 65535 = 32896
  //unsigned int lim_sup = (128.0 / 255.0) * 65535; //MagickCore::Quantum;
  // printf("%d - %d.\n", lim_inf, lim_sup);
  //Magick::Quantum colorAux = 0;
  //Magick::Color colorPixel;
  try
  {
    img_mapa_orig_.read(nombre_fich_);
    img_mapa_modif_ = img_mapa_orig_;
  }
  catch (Magick::WarningCoder& warning)
  {
    // Process coder warning while loading file (e.g. TIFF warning)
    // Maybe the user will be interested in these warnings (or not).
    // If a warning is produced while loading an image, the image
    // can normally still be used (but not if the warning was about
    // something important!)
    printf("Coder Warning: %s.\n", warning.what());
    return false;
  }
  catch (Magick::Warning &warning)
  {
    // Handle any other Magick++ warning.
    printf("Warning: %s.\n", warning.what());
    return false;
  }
  catch (Magick::ErrorFileOpen &error)
  {
    // Process Magick++ file open error
    printf("Error: %s.\n", error.what());
    return false;
  }
  /*
   filas_ = img_mapa_orig_.rows();
   columnas_ = img_mapa_orig_.columns();
   std::cout << "Dimensiones del mapa: fil: " << filas_ << " col: " << columnas_ << std::endl;
   matriz_probabilidad_ocupacion_.assign(filas_, std::vector<double>(columnas_, 0));
   // Leer el mapa de trabajo y construir la matriz de probabilidad.
   for (filCordY = 0; filCordY < filas_; filCordY++)
   {
   for (colCordX = 0; colCordX < columnas_; colCordX++)
   {
   colorPixel = img_mapa_orig_.pixelColor(colCordX, filCordY);
   azul = colorPixel.blueQuantum();
   rojo = colorPixel.redQuantum();
   verde = colorPixel.greenQuantum();
   // Supongo que el pixel esta expresado en escala de grises.
   colorAux = azul;
   // printf("(%d, %d) - %d, %d, %d.\n", filCordY, colCordX, rojo, verde, azul);
   // Si el pixel no esta expresado en escala de grises se hace una media
   // entre el valor de las coordenadas RGB
   if (azul != rojo || azul != verde)
   {
   //colorAux = (unsigned int)((((float)(azul + rojo + verde)) / 3.0) + 0.5);
   colorAux = Magick::Color::scaleDoubleToQuantum(
   (Magick::Color::scaleQuantumToDouble(rojo) + Magick::Color::scaleQuantumToDouble(verde)
   + Magick::Color::scaleQuantumToDouble(azul)) / 3);
   //printf("(%d, %d) - %d, %d, %d --> %d.\n", filCordY, colCordX, rojo, verde, azul, colorAux);
   }
   // Probabilidad de ocupacion.
   // Pixel blanco (255, 255, 255) ==> prob.ocupacion = 0.
   // Pixel negro (0, 0, 0) ==> prob.ocupacion = 1.
   matriz_probabilidad_ocupacion_[filCordY][colCordX] = 1 - Magick::Color::scaleQuantumToDouble(colorAux);
   //if (colorAux >= lim_inf && colorAux <= lim_sup) {
   //	matriz_probabilidad_[filCordY][colCordX] = 0.5;
   //}
   //printf("(%d, %d) - %f.\n", filCordY, colCordX, matriz_probabilidad_[filCordY][colCordX]);
   //printf("----------------------------------\n");
   }
   }
   */
  return true;
}

void MapaLoc::pintarPixel(const unsigned int& x, const unsigned int& y, const Magick::Color& color)
{
  img_mapa_modif_.pixelColor(x, y, color);
}
