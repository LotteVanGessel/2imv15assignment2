#pragma once
#include "Particle.h"
#include <vector>
#include <math.h>

class Constraint {
public:
   Constraint(std::vector<Particle*> particles): particles(particles) {};

   virtual float constraint();
   virtual void derCon();
   virtual float constraintDerivative();
   virtual std::vector<Vec2f> J();
   virtual std::vector<Vec2f> JDerivative();\
   virtual void draw();
   virtual void apply();

   std::vector<Particle*> particles;
};
