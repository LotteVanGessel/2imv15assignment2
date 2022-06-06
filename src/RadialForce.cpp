#include "RadialForce.h"

RadialForce::RadialForce(std::vector<Particle*> particles, int p1)
{
	this->particles = particles;
	this->index = p1;
}
void RadialForce::draw()
{
}


void RadialForce::apply()
{
	Particle* radialPar = particles[index];
	for (Particle* par : particles)
	{
		Vec2f dist = par->m_Position - radialPar->m_Position;

		float Edist = pow(abs(radialPar->m_Position[0] - par->m_Position[0]) + abs(radialPar->m_Position[1] - par->m_Position[1]), 0.5);
		par->m_Force += dist / norm(dist) * 1 / Edist * 0.01;
	}
}