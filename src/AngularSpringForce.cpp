#include "SpringForce.h"
#include "AngularSpringForce.h"

using namespace std;

AngularSpringForce::AngularSpringForce(std::vector<Particle*> p, int p1, int p2, int p3, double dis, double ks, double kd) :
	m_p1(p1), m_p2(p2), m_p3(p3), m_dist(dis), m_ks(ks), m_kd(kd) 
{
	this->particles = p;
}


void AngularSpringForce::apply() {
    //To calculate the angle the distance between the pairs of the particles. 
    Vec2f dis1 = particles[m_p1]->m_Position - particles[m_p2]->m_Position;
    Vec2f dis2 = particles[m_p2]->m_Position - particles[m_p3]->m_Position;
    Vec2f dis3 = particles[m_p1]->m_Position - particles[m_p3]->m_Position;

    // We will use the angle between p1p2p3. So the angle between dist1 and dist2 
    // Therefore the force will apply on particle 1 and particle 2 
    
    // Speed difference between p1 and p2
    Vec2f speed = particles[m_p1]->m_Velocity - particles[m_p3]->m_Velocity;

    // Normalized distances 
    double adis1 = norm(dis1);
    double adis2 = norm(dis2);
    double adis3 = norm(dis3);

    // We fix the angle by calculating the distance between par 1 and par 2 by law of cosines
    // And substracting that from the desired distance. And then just normal spring force
    Vec2f result = -((m_ks * (adis3 - sqrt(adis1 * adis1 + adis2 * adis2 - 2 * adis1 * adis2 * cos(m_dist))) + m_kd * ((dis3 * speed) / adis3)) * (dis3 / adis3));

    particles[m_p1]->m_Force += result;
    particles[m_p3]->m_Force -= result;
}


void AngularSpringForce::draw() 
{
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex2f(particles[m_p1]->m_Position[0], particles[m_p1]->m_Position[1]);
    glVertex2f(particles[m_p2]->m_Position[0], particles[m_p2]->m_Position[1]);
    glVertex2f(particles[m_p3]->m_Position[0], particles[m_p3]->m_Position[1]);
    glEnd();
}