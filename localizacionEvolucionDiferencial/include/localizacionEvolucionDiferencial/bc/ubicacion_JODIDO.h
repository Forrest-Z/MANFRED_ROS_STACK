/*
 * posicion.h
 *
 *  Created on: Jul 26, 2011
 *      Author: jfrascon
 */

#ifndef UBICACION_H_
#define UBICACION_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

class Ubicacion
{

public:

  // Constructor por defecto.
  Ubicacion(const double & x = 0, const double & y = 0, const double & theta = 0) :
      x_(x), y_(y), theta_(theta)
  {
  }

  // Constructor copia.
  Ubicacion(const Ubicacion & po) :
      x_(po.x_), y_(po.y_), theta_(po.theta_)
  {
  }

  ~Ubicacion()
  {
  }

  inline double getX() const
  {
    return x_;
  }

  inline double getY() const
  {
    return y_;
  }

  inline double getTheta() const
  {
    return theta_;
  }

  inline void getXY(double & x, double & y) const
  {
    x = x_;
    y = y_;
  }

  inline void getXYTheta(double & x, double & y, double & theta) const
  {
    x = x_;
    y = y_;
    theta = theta_;
  }

  inline void setX(const double& x)
  {
    x_ = x;
  }

  inline void setY(const double& y)
  {
    y_ = y;
  }

  inline void setTheta(const double & theta)
  {
    theta_ = theta;
  }

  inline void setXYTheta(const double & x, const double & y, const double & theta)
  {
    x_ = x;
    y_ = y;
    theta_ = theta;
  }

  inline float rotarUbicacion(const float& anguloRotarRad){

	  theta_ += anguloRotarRad;
	  return theta_;
  }

  inline void trasladarUbicacion(const float& orientacionRad, const float& distancia){

	  // La distancia tiene que ser un valor positivo.
	  // La orientacion puede tener cualquier valor, no es necesario que su valor
	  // se haye entre -pi y pi.
	  x_ += fabs(distancia)*cos(orientacionRad);
	  y_ += fabs(distancia)*sin(orientacionRad);

  }

  inline static void orientacionMenosPiMasPi(double& angulo){

	  double dosPI = 2 * M_PI;
	  double signoThetaDosPI = (angulo < 0 ? -1 : angulo > 0) * dosPI;
	  if (fabs(angulo) > dosPI)
	  {
	    // Si el valor fabs(deltaTheta) esta entre 0 y 2*pi el valor retornado por la operacion
	    // fmodf(deltaTheta, signoDeltaThetaDosPi) es el valor de su primer argumento, es decir,
	    // deltaTheta.
	    // Si el valor de fabs(deltaTheta) es mayor que 2*pi, entonces la operacion fmod retorna
	    // el angulo equivalente entre 0 y 2*pi.
	    angulo = fmod(angulo, signoThetaDosPI);
	  }
	  // Finalmente hacer que el valor de deltaTheta este entre -pi y pi.
	  if (fabs(angulo) > M_PI)
	  {
	    angulo -= signoThetaDosPI;
	  }
  }

private:
  // 'x' e 'y' en celdillas.
  double x_;
  double y_;
  // 'theta' en radianes.
  double theta_;
};

#endif

