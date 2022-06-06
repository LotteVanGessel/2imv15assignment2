#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  Constraint({p1, p2}), m_p1(p1), m_p2(p2), m_dist(dist) {}
  
float RodConstraint::constraint()
{
    Vec2f positionVector = m_p1->m_Position - m_p2->m_Position;
    return positionVector * positionVector - pow(m_dist, 2);
}

float RodConstraint::constraintDerivative()
{
    Vec2f positionVectorDerivative = 2 * (m_p1->m_Position - m_p2->m_Position);
    Vec2f velocityVectorDerivative = 2 * (m_p1->m_Velocity - m_p2->m_Velocity);
    return positionVectorDerivative * velocityVectorDerivative;
}

std::vector<Vec2f> RodConstraint::J()
{
    Vec2f derivativePosition1 = (m_p1->m_Position - m_p2->m_Position) * 2;
    Vec2f derivativePosition2 = (m_p2->m_Position - m_p1->m_Position) * 2;

    return std::vector<Vec2f>{derivativePosition1, derivativePosition2};
}

std::vector<Vec2f> RodConstraint::JDerivative()
{
    Vec2f derivativePosition1 = (m_p1->m_Velocity - m_p2->m_Velocity) * 2;
    Vec2f derivativePosition2 = (m_p2->m_Velocity - m_p1->m_Velocity) * 2;

    return std::vector<Vec2f>{derivativePosition1, derivativePosition2};
}

void RodConstraint::apply()
{
    m_p1->m_Force += JDerivative()[0];
    m_p2->m_Force += JDerivative()[1];
}
  
void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}
