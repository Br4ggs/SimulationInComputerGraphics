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
    float r = powf(float(m_dist), 2);
    return x_delt + y_delt - r;
}

float RodConstraint::C_prim()
{
    Vec2f x_delt = m_p1->m_Position - m_p2->m_Position;
    Vec2f v_delt = m_p1->m_Velocity - m_p2->m_Velocity;
    float mag = sqrt(pow(x_delt[0], 2) + pow(x_delt[1], 2));
    float dot = (x_delt[0] * v_delt[0]) + (x_delt[1] * v_delt[1]);

    return dot / mag;
}

void RodConstraint::jacob(Matrix* J)
{
    //encode these with a particle
    //get p1 index
    //get p2 index
    Vec2f x_delt = m_p1->m_Position - m_p2->m_Position;
    float mag = sqrt(pow(x_delt[0], 2) + pow(x_delt[1], 2));

    float p1x = x_delt[0] / mag;
    float p1y = x_delt[1] / mag;

    float p2x = -1 * p1x;
    float p2y = -1 * p1y;
}