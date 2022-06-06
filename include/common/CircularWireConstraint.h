#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);
  
  float constraint() override;
  float constraintDerivative() override;
  std::vector<Vec2f> J() override;
  std::vector<Vec2f> JDerivative() override;
  void draw() override;
  void apply() override;

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
