#pragma once
#include "Particle.h"
#include "Force.h"

class RadialForce : public Force
{
public:
	RadialForce(std::vector<Particle*> particles, int p1);
	int index; 
	void draw() override;
	void apply() override;
};