#include "GravityForce.h"

GravityForce::GravityForce(std::vector<Particle*> particles)
{
	this->particles = particles;
}

void GravityForce::draw()
{
}

void GravityForce::apply()
{
	for (Particle* part : particles)
	{
		part->m_Force += Vec2f(0,-0.02 * part->m_Mass);
	}
}