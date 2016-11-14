#ifndef BARRIDO_LASER_H_
#define BARRIDO_LASER_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>

class BarridoLaser
{

public:

  // Constructor por defecto
  BarridoLaser(const double& incr_ang = 0.0, const Posicion& pos_orig = Posicion(0, 0, 0)) :
      incr_ang_(incr_ang), pos_orig_(pos_orig)
  {
  }

  BarridoLaser(const double& incr_ang, const Posicion& pos_orig, const std::vector<double>& dist,
                 const std::vector<std::vector<Posicion> >& coord) :
      incr_ang_(incr_ang), pos_orig_(pos_orig), dist_(dist), coord_(coord)
  {
  }

  BarridoLaser(const double& incr_ang, const Posicion& pos_orig, const unsigned int& num_haces) :
      incr_ang_(incr_ang), pos_orig_(pos_orig), dist_(std::vector<double>(num_haces, 0)), coord_(
          std::vector<std::vector<Posicion> >(num_haces))
  {
  }

  // Constructor copia
  BarridoLaser(const BarridoLaser& barrido_laser) :
      incr_ang_(barrido_laser.incr_ang_), pos_orig_(barrido_laser.pos_orig_), dist_(barrido_laser.dist_), coord_(
          barrido_laser.coord_)
  {
  }

  ~BarridoLaser()
  {
  }

  double incr_ang() const
  {
    return incr_ang_;
  }

  Posicion pos_orig() const
  {
    return pos_orig_;
  }

  std::vector<std::vector<Posicion> > coord() const
  {
    return coord_;
  }

  std::vector<Posicion> CoordHaz(unsigned int& num_haz) const
  {
    return coord_[num_haz];
  }

  Posicion CoordHazPosicion(unsigned int& num_haz, unsigned int& num_pos)
  {
    return coord_[num_haz][num_pos];
  }

  std::vector<double> dist() const
  {
    return dist_;
  }

  double ObtenerDistancia(unsigned int& num_haz)
  {
    return dist_[num_haz];
  }

  void set_pos_orig(const Posicion& pos_orig)
  {
    pos_orig_ = pos_orig;
  }

  void set_dist(std::vector<double> dist)
  {
    dist_ = dist;
  }

  void GuardarDist(const unsigned int& num_haz, const double& dist)
  {
    dist_[num_haz] = dist;
  }

  void GuardarPosicion(const unsigned int& num_haz, const Posicion& pos)
  {
    coord_[num_haz].push_back(pos);
  }

  void NumeroHaces(const unsigned int& num_haces)
  {
    dist_.resize(num_haces, 0);
    coord_.resize(num_haces);
  }

  void LimpiarBarrido()
  {
    dist_.erase(dist_.begin(), dist_.end());
    coord_.erase(coord_.begin(), coord_.end());
  }

private:
  double incr_ang_;
  Posicion pos_orig_;
  // Distancias.
  std::vector<double> dist_;
  // Coordenadas.
  std::vector<std::vector<Posicion> > coord_;

};

#endif
