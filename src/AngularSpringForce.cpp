#include "AngularSpringForce.h"
#ifdef __APPLE__
#include <GLUT/glut.h>
#include <cmath>
#else
#include <GL/glut.h>
#endif

// Particle p2 is the corner particle
AngularSpringForce::AngularSpringForce(Particle *p1, Particle * p2, Particle * p3, double rest_angle, double stiffness_constant) :
  m_p1(p1), m_p2(p2), m_p3(p3), m_rest_angle(rest_angle), m_stiffness_constant(stiffness_constant) {}

Vec2f perpendicular_vec_counterclockwise(Vec2f vec)
{
    return Vec2f(-vec[1], vec[0]);
}

Vec2f perpendicular_vec_clockwise(Vec2f vec)
{
    return Vec2f(vec[1], -vec[0]);
}

void AngularSpringForce::apply_force()
{
    float rest_angle_rad = m_rest_angle * M_PI / 180.0f; // Convert the rest angle from degrees to radians
    
    Vec2f b = m_p1->m_Position - m_p2->m_Position;
    Vec2f a = m_p3->m_Position - m_p2->m_Position;
    float cross_product = (b[0] * a[1] - b[1] * a[0]);

    // Vec2f b_norm = norm(b);
    // Vec2f a_norm = norm(a);
    //float dot_product = a_norm[0] * b_norm[0] + a_norm[1] * b_norm[1];
    // dot_product = std::max(-1.0f, std::min(1.0f, dot_product)); // Clamp the dot product to avoid NaN from acos
    float angle_a = std::atan2(a[1], a[0]);
    float angle_b = std::atan2(b[1], b[0]); 
    float current_angle = angle_b - angle_a; // this angle is in radians
    // Normalize to [0, 2π]
    if (current_angle < 0) 
    {
        current_angle += 2 * M_PI;
    }
    float angle_diff = current_angle - rest_angle_rad;
    float force_multiplier = std::abs(angle_diff * m_stiffness_constant);
    Vec2f b_perp;
    Vec2f a_perp;
    if (angle_diff < 0) // current_angle < rest_angle_rad: angle needs to be increased
    {
        if (cross_product > 0) // b is counterclockwise to a 
        {
            b_perp = perpendicular_vec_counterclockwise(b);
            a_perp = perpendicular_vec_clockwise(a);
        } else
        // cross_product < 0: b is clockwise to a
        // cross_product == 0: // be is parallel to a
        {
            b_perp = perpendicular_vec_clockwise(b);
            a_perp = perpendicular_vec_counterclockwise(a);
        }
    } else if (angle_diff > 0) // current_angle > rest_angle_rad: angle needs to be decreased
    {
        if (cross_product > 0) // b is counterclockwise to a 
        {
            b_perp = perpendicular_vec_clockwise(b);
            a_perp = perpendicular_vec_counterclockwise(a);
        } else
        // cross_product < 0: b is clockwise to a
        // cross_product == 0: // be is parallel to a
        {
            b_perp = perpendicular_vec_counterclockwise(b);
            a_perp = perpendicular_vec_clockwise(a);
        }
    } else // angle_diff == 0: angle is equal to rest_angle
    {
        // No force is applied
        b_perp = Vec2f(0.0, 0.0);
        a_perp = Vec2f(0.0, 0.0);
    }

    Vec2f force_on_b = norm(b_perp) * force_multiplier;
    Vec2f force_on_a = norm(a_perp) * force_multiplier;

    m_p1->m_Force += force_on_b;
    m_p3->m_Force += force_on_a;
    m_p2->m_Force -= force_on_b + force_on_a; // The corner particle gets negation of the sum of the forces
}

void AngularSpringForce::draw()
{
/*
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
  */
}
