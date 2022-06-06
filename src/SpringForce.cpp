#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(std::vector<Particle*> p, int p1, int p2, double dist, double ks, double kd, float col1, float col2, float col3) :
	m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd), col1(col1), col2(col2), col3(col3) {
	this->particles = p; //For easier access of the particlex in the springforce
}
void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(col1, col2, col3);
  glVertex2f(particles[m_p1]->m_Position[0], particles[m_p1]->m_Position[1] );
  glColor3f(col1, col2, col3);
  glVertex2f(particles[m_p2]->m_Position[0], particles[m_p2]->m_Position[1] );
  glEnd();
  // 0.6, 0.7, 0.6
}

void SpringForce::apply()
{

	Vec2f length = particles[m_p1]->m_Position - particles[m_p2]->m_Position;
	Vec2f speed = particles[m_p1]->m_Velocity - particles[m_p2]->m_Velocity;
	Vec2f result = -((m_ks * (norm(length) - m_dist) + m_kd * ((length * speed) / norm(length))) * (length / norm(length)));
	particles[m_p1]->m_Force += result;
	particles[m_p2]->m_Force -= result;
	//std::cout << result << endl;
}
