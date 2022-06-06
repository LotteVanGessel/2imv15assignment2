#pragma once
#include "Particle.h"
#include "Force.h"

class GravityForce : public Force 
{
	public:
		GravityForce(std::vector<Particle*> particles);

		void draw() override;
		void apply() override;
};