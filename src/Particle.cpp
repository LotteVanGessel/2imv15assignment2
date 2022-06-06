#include "Particle.h"
#include <GL/glut.h>
#define PI 3.1415926535897932384626433832795

Particle::Particle(const Vec2f & ConstructPos , float mass) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0)), m_Force(Vec2f(0.0, 0.0)), m_Mass(mass), anchored(FALSE)
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_Force = Vec2f(0.0, 0.0);
}
void Particle::draw()
{
	const double h = 0.02;
	glBegin(GL_TRIANGLE_FAN);
	glColor3f(1.f, 1.f, 1.f);
	for (int i = 0; i < 360; i = i + 18)
	{
		float degInRad = i * PI / 180;
		glVertex2f(m_Position[0] + cos(degInRad) * h, m_Position[1] + sin(degInRad) * h);
	}
	glEnd();
}

void Particle::setAnchor()
{
	anchored = TRUE;
}

