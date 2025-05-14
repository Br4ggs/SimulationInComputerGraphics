#include "RodConstraint.h"
#include <GL/glut.h>
#include <math.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

float RodConstraint::C()
{
    float x_delt = powf(m_p1->m_Position[0] - m_p2->m_Position[0], 2);
    float y_delt = powf(m_p1->m_Position[1] - m_p2->m_Position[1], 2);
    float r = powf(float(dist), 2);
    return x_delt + y_delt - r;
}

float RodConstraint::C_prime()
{
    //derivative of C w.r.t t
}