// ParticleToy.cpp : Defines the entry point for the console application.
//
#include "System.h"
#include "ExplicitEulerSolver.h"
#include "Force.h"
#include "GravityForce.h"
#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "Matrix.h"
#include "imageio.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>

/* macros */

/* external definitions (from solver) */
extern void simulation_step( std::vector<Particle*> pVector, float dt );

/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

static Matrix M(0, 0); //mass matrix
static Matrix W(0, 0); //mass matrix inverse

static Matrix J(0, 0);  //jacobian
static Matrix Jt(0, 0); //jacobian transpose

// static Particle *pList;
static std::vector<Particle*> pVector; // Vector of particles
static std::vector<Force*> fVector; // Vector of forces
static std::vector<Constraint*> cVector; // Vector of constraints

//TODO: constraints
// Vector of constraints


// System interface

int get_dim()
{
	//our system is 2D
	return 4 * pVector.size();
}

std::vector<float> get_state()
{
	std::vector<float> sVector;

	for (int i = 0; i < pVector.size(); i++)
	{
		sVector.push_back(pVector[i]->m_Position[0]); //x position
		sVector.push_back(pVector[i]->m_Position[1]); //y position
		sVector.push_back(pVector[i]->m_Velocity[0]); //x velocity
		sVector.push_back(pVector[i]->m_Velocity[1]); //y velocity
	}

	return sVector;
}

void set_state(std::vector<float> sVector)
{
	//TODO: check for size mismatch between pVector and sVector: sVector.size == pVector.size * 4

	for (int i = 0; i < pVector.size(); i++)
	{
		int ii = i * 4;
		pVector[i]->m_Position[0] = sVector[ii];     //x position
		pVector[i]->m_Position[1] = sVector[ii + 1]; //y position
		pVector[i]->m_Velocity[0] = sVector[ii + 2]; //x velocity
		pVector[i]->m_Velocity[1] = sVector[ii + 3]; //y velocity
	}
}

std::vector<float> deriv_eval()
{
	std::vector<float> dVector;
	for (int i = 0; i < pVector.size(); i++)
	{
		dVector.push_back(pVector[i]->m_Velocity[0]); //dx/dt 1st derivative
		dVector.push_back(pVector[i]->m_Velocity[1]); //dy/dt
		dVector.push_back(pVector[i]->m_Force[0] / pVector[i]->f_Mass); //dv/dt (x) 2nd derivative
		dVector.push_back(pVector[i]->m_Force[1] / pVector[i]->f_Mass); //dv/dt (y)
	}

	return dVector;
}

// System helper functions

static void clear_force_accumulators()
{
    for (int i = 0; i < pVector.size(); i++)
    {
        pVector[i]->m_Force = Vec2f(0.0, 0.0);
    }
}

static void apply_force_accumulators()
{
    for (int i = 0; i < fVector.size(); i++)
    {
        fVector[i]->apply_force();
    }
}

// Constraint helper functions

static std::vector<float> get_q()
{
    std::vector<float> q;
    for (int i = 0; i < pVector.size(); i++)
    {
        q.push_back(pVector[i]->m_Position[0]); //x position
		q.push_back(pVector[i]->m_Position[1]); //y position
    }

    return q;
}

static std::vector<float> get_q_prim()
{
    std::vector<float> q_prim;
    for (int i = 0; i < pVector.size(); i++)
    {
        q_prim.push_back(pVector[i]->m_Velocity[0]);
        q_prim.push_back(pVector[i]->m_Velocity[1]);
    }

    return q_prim;
}

static std::vector<float> get_Q()
{
    std::vector<float> Q;
    for (int i = 0; i < pVector.size(); i++)
    {
        Q.push_back(pVector[i]->m_Force[0]);
        Q.push_back(pVector[i]->m_Force[1]);
    }

    return Q;
}

static std::vector<float> get_C()
{
    std::vector<float> C;
    for (int i = 0; i < cVector.size(); i++)
    {
        //populate C with constraint functions
        C.push_back(cVector[i]->C());
    }
}

static std::vector<float> get_C_prim()
{
    std::vector<float> C_prim;
    for (int i = 0; i < cVector.size(); i++)
    {
        C_prim.push_back(cVector[i]->C_prim());
    }
}

static void calculate_jacobians()
{
    for (int i = 0; i < cVector.size(); i++)
    {
        cVector[i]->jacob(&J);
    }
}

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

//static SpringForce * delete_this_dummy_spring = NULL;
static RodConstraint * delete_this_dummy_rod = NULL;
static CircularWireConstraint * delete_this_dummy_wire = NULL;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

// TODO: Update later
static void free_data ( void )
{
	pVector.clear();
	if (delete_this_dummy_rod) {
		delete delete_this_dummy_rod;
		delete_this_dummy_rod = NULL;
	}
	// if (delete_this_dummy_spring) {
	// 	delete delete_this_dummy_spring;
	// 	delete_this_dummy_spring = NULL;
	//}
	if (delete_this_dummy_wire) {
		delete delete_this_dummy_wire;
		delete_this_dummy_wire = NULL;
	}
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	pVector.push_back(new Particle(center + offset, 0));
	pVector.push_back(new Particle(center + offset + offset, 1));
	pVector.push_back(new Particle(center + offset + offset + offset, 2));

	// Forces
	fVector.push_back(new GravityForce(pVector, Vec2f(0, -1)));
    fVector.push_back(new SpringForce(pVector[0], pVector[1], dist, 1.0, 1.0));
    
    //create M
    M = Matrix(2 * pVector.size(), 2 * pVector.size());

    //populate the diagonal of M
    for (int i = 0; i < pVector.size(); i++)
    {
        M[i * 2][i * 2] = pVector[i]->f_Mass;
        M[(i * 2) + 1][(i * 2) + 1] = pVector[i]->f_Mass;
    }

    //create W
    W = Matrix(2 * pVector.size(), 2 * pVector.size());

    for (int i = 0; i < pVector.size(); i++)
    {
        W[i * 2][i * 2] = 1 / pVector[i]->f_Mass;
        W[(i * 2) + 1][(i * 2) + 1] = 1 / pVector[i]->f_Mass;
    }

    //create J
    J = Matrix(2 * pVector.size(), cVector.size());

    //create Jt
    J = Matrix(cVector.size(), 2 * pVector.size());
	
	// You shoud replace these with a vector generalized forces and one of
	// constraints...
	//delete_this_dummy_spring = new SpringForce(pVector[0], pVector[1], dist, 1.0, 1.0);
	delete_this_dummy_rod = new RodConstraint(pVector[1], pVector[2], dist);
	delete_this_dummy_wire = new CircularWireConstraint(pVector[0], center, dist);

    cVector.push_back(delete_this_dummy_rod); //jank
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			// glRasterPos2i(0, 0);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			static char filename[80];
			sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
			
			free(buffer);
		}
	}
	frame_number++;
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int ii=0; ii< size; ii++)
	{
		pVector[ii]->draw();
	}
}

static void draw_forces ( void )
{
    for (int i = 0; i < fVector.size() ; i++)
    {
        fVector[i]->draw();
    }
	// change this to iteration over full set
	// if (delete_this_dummy_spring)
	// 	delete_this_dummy_spring->draw();
}

static void draw_constraints ( void )
{
	// change this to iteration over full set
	if (delete_this_dummy_rod)
		delete_this_dummy_rod->draw();
	if (delete_this_dummy_wire)
		delete_this_dummy_wire->draw();
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {

	}

	if ( mouse_down[2] ) {
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	if( mouse_release[0] ) {
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];

        //added to prevent particles shooting off to neptune after pressing space a couple of times
        pVector[ii]->m_Velocity[0] = 0.0;
		pVector[ii]->m_Velocity[1] = 0.0;
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;

	case ' ':
		dsim = !dsim;
		break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if (dsim)
    {
        //1. clear all force accumulators in particles
        clear_force_accumulators();
        //2. apply forces to particle accumulators
        apply_force_accumulators();

        //TODO: constraints
        // - create state vector q of size 2n (contains only the positions for each particle)
        std::vector<float> q = get_q();
        std::vector<float> q_prim = get_q_prim();

        // - get mass matrix (M), and its inverse (W)

        // for the mass matrix, populate the diagonal with the masses of the particles (2 cells per particle as we're in 2D)
        // the inverse is just 1 divided by the corresponding mass for each cell

        // -create global force vector Q of size 2n (contains only the force for each particle)
        std::vector<float> Q = get_Q();

        // C is a vector consisting of the returned values of the constraints
        // the constraints here are the implicit functions describing the constraint (like a circle equation, or a rod)
        std::vector<float> C = get_C();

        // C', for a single constraint, is the dot product of the jacobian and the velocities of the particles involved
        std::vector<float> C_prim = get_C_prim();

        // -calculate the jacobian J (i don't know how to approach this part)
        calculate_jacobians();

        // -calculate the jacobian derivative J'

        // assemble JWJ^t
        // rather then assembling it directly

        // A = JWJ^t
        // X = lambda
        // b = -J'q' - JWQ (-k_s C - k_d C')
        // solve using linear solver, this is going to populate lambda

        // Q_hat = J^t * lambda what we're trying to solve?
        // add Q_hat to affected force acumulators

        // J^t X (lambda)

        // eventually we must solve JWJ^t lambda = -J'q' - JWQ (-k_s C - k_d C') to find lambda, the lagrange multipliers

        // what are C, C', J and J' supposed to look like?

        // we can now represent q'' (aka the acceleration of each particle) as q'' = WQ

        // TODO: create a constraint function that takes in vector q (size 2n) and returns a vector of size m (# constraints)

        // our goal is to compute constraint vector Q_hat, that when added to Q guarantees that C''=0
        
        // Q_hat = J^t * lambda what we're trying to solve?

        //3. call solver 
        explicit_euler_solve(dt);
        //simulation_step( pVector, dt ); //TODO: call your solver of choice here
    }
	else
    {
        get_from_UI();
        remap_GUI();
    }

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		//dt = 0.1f;
        dt = 0.01f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;
	
	init_system();
	
	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}

