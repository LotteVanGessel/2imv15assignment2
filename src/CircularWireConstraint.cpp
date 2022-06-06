#include "CircularWireConstraint.h"
#include <GL/glut.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius) :
	Constraint({p}), m_p(p), m_center(center), m_radius(radius) {}

float CircularWireConstraint::constraint()
{
    Vec2f positionVector = m_p->m_Position - m_center;
    return positionVector * positionVector - m_radius * m_radius;
}

float CircularWireConstraint::constraintDerivative()
{
    Vec2f positionVectorDerivative = m_p->m_Position - m_center;
    return 2 * (positionVectorDerivative * m_p->m_Velocity);
}

std::vector<Vec2f> CircularWireConstraint::J()
{
    Vec2f derivativePosition = (m_p->m_Position - m_center) * 2;
    return std::vector<Vec2f>{derivativePosition};
}

std::vector<Vec2f> CircularWireConstraint::JDerivative()
{
    Vec2f derivativePosition = m_p->m_Velocity * 2;
    return std::vector<Vec2f>{derivativePosition};
}
	
void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

void CircularWireConstraint::apply()
{
	m_p->m_Force += JDerivative()[0];
}
