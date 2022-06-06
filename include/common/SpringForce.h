#pragma once

#include "Particle.h"
#include "Force.h"

class SpringForce: public Force {
 public:
  SpringForce(std::vector<Particle*> p, int p1, int p2, double dist, double ks, double kd, float col1, float col2, float col3);

  void draw() override;
  void apply() override;

 private:
	 double const m_dist; //Normal length between particles
	 double const m_ks, m_kd; //Spring constants.
	 int const m_p1, m_p2; //Positions of particles in the vector of all particles.
	 float const col1, col2, col3;
};
