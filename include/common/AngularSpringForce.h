#include "Particle.h"
#include "Force.h"
#include "SpringForce.h"
#include <GL/glut.h>


class AngularSpringForce : public Force {

	public:
		AngularSpringForce(std::vector<Particle*> p, int p1, int p2, int p3, double ang, double ks, double kd);

		void draw() override;
		void apply() override;

	private:
		double const m_dist; //Normal length between particles
		double const m_ks, m_kd; //Spring constants.
		int const m_p1, m_p2, m_p3; //Positions of particles in the vector of all particles.


};