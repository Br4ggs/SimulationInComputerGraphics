#include "CircularWireConstraint.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <math.h>

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

CircularWireConstraint::CircularWireConstraint(Particle *p, const Vec2f & center, const double radius, unsigned int index) :
	m_p(p), m_center(center), m_radius(radius), index(index) {}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

float CircularWireConstraint::C()
{
    Vec2f x_delt = m_p->m_Position - m_center;

    return pow(x_delt[0], 2) + pow(x_delt[1], 2) - pow(m_radius, 2);
}

float CircularWireConstraint::C_prim()
{
    Vec2f x_delt = m_p->m_Position - m_center;

    return 2 * x_delt[0] * m_p->m_Velocity[0] + 2 * x_delt[1] * m_p->m_Velocity[1];
}

void CircularWireConstraint::jacob(Matrix* J)
{
    Vec2f x_delt = m_p->m_Position - m_center;

    float p1x = 2 * x_delt[0];
    float p1y = 2 * x_delt[1];

    (*J)[index][2 * m_p->index] = p1x;
    (*J)[index][2 * m_p->index + 1] = p1y;
}

void CircularWireConstraint::jacob_prim(Matrix* J_prim)
{
    float p1x = 2 * m_p->m_Velocity[0];
    float p1y = 2 * m_p->m_Velocity[1];

    (*J_prim)[index][2 * m_p->index] = p1x;
    (*J_prim)[index][2 * m_p->index + 1] = p1y;
}