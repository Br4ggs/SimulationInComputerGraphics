#include "RodConstraint.h"
#include <GL/glut.h>
#include <math.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist, unsigned int index) :
  m_p1(p1), m_p2(p2), m_dist(dist), index(index) {}

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
    Vec2f x_delt = m_p1->m_Position - m_p2->m_Position;
    float mag = sqrt(pow(x_delt[0], 2) + pow(x_delt[1], 2));

    float p1x = x_delt[0] / mag;
    float p1y = x_delt[1] / mag;

    float p2x = -1 * p1x;
    float p2y = -1 * p1y;

    // (*J)[2 * m_p1->index][index] = p1x;
    // (*J)[(2 * m_p1->index) + 1][index] = p1y;
    (*J)[index][2 * m_p1->index] = p1x;
    (*J)[index][2 * m_p1->index + 1] = p1y;

    // (*J)[2 * m_p2->index][index] = p2x;
    // (*J)[(2 * m_p2->index) + 1][index] = p2y;

    (*J)[index][2 * m_p2->index] = p2x;
    (*J)[index][2 * m_p2->index + 1] = p2y;
}

void RodConstraint::jacob_prim(Matrix* J_prim)
{
    Vec2f x_delt = m_p1->m_Position - m_p2->m_Position;
    Vec2f v_delt = m_p1->m_Velocity - m_p2->m_Velocity;
    float mag_sqrt = pow(x_delt[0], 2) + pow(x_delt[1], 2);

    float p1x = (v_delt[0] - ((x_delt[0] * v_delt[0] + x_delt[1] * v_delt[1]) / mag_sqrt) * x_delt[0]) / sqrt(mag_sqrt);
    float p1y = (v_delt[1] - ((x_delt[0] * v_delt[0] + x_delt[1] * v_delt[1]) / mag_sqrt) * x_delt[1]) / sqrt(mag_sqrt);

    float p2x = -1 * p1x;
    float p2y = -1 * p1y;

    (*J_prim)[index][2 * m_p1->index] = p1x;
    (*J_prim)[index][2 * m_p1->index + 1] = p1y;

    (*J_prim)[index][2 * m_p2->index] = p2x;
    (*J_prim)[index][2 * m_p2->index + 1] = p2y;
}
