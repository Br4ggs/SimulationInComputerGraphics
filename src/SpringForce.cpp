#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::apply_force()
{
  Vec2f l = m_p1->m_Position - m_p2->m_Position;
  Vec2f l_deriv = m_p1->m_Velocity - m_p2->m_Velocity;
  float magnitude_l = std::sqrt(l[0] * l[0] + l[1] * l[1]);
  m_p1->m_Force += -(m_ks * (magnitude_l - m_dist) + m_kd * ((l_deriv * l) / magnitude_l)) * (l / magnitude_l);
  m_p2->m_Force += - (m_p1->m_Force);
}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}
