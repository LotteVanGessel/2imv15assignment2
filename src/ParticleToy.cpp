/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
   Author : Jos Stam (jstam@aw.sgi.com)
   Creation Date : Jan 9 2003

   Description:

	This code is a simple prototype that demonstrates how to use the
	code provided in my GDC2003 paper entitles "Real-Time Fluid Dynamics
	for Games". This code uses OpenGL and GLUT for graphics and interface

  =======================================================================
*/

#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <iostream>
#include <vector>
#include "Object.h"
#include "Rigidbody.h"
#include <cmath>
#include "RigidbodyCollection.h"
#include "Shape.h"
#include <cstring>
#include <set>
#include <utility>
#include "EdgeSet.h"

/* macros */

#define IX(i,j) ((i)+(N+2)*(j))
static bool DEBUG_DRAWING = false;

/* external definitions (from solver.c) */

extern void dens_step(int N, float* x, float* x0, float* u, float* v, float diff, float dt);
extern void vel_step(int N, float* u, float* v, float* u0, float* v0, float visc, float dt, float* vor, int dovorticity);

extern std::set<Edge> edges;



/* global variables */

Object* mObj = new Object(20, 20, 6);
int targetx = 20, targety = 20;
Vec2 bot_left_rectangle = Vec2(0.4, 0.4);
Vec2 top_right_rectangle = Vec2(0.6, 0.6);
Rigidbody* rb = new Rigidbody(Rect(bot_left_rectangle, top_right_rectangle));
std::vector<Rigidbody*> rbs = std::vector<Rigidbody*>();
RigidbodyCollection rbc = RigidbodyCollection();
static int N;
static float dt, diff, visc;
static float force, source;
static int dvel;
static int dgrid; // draw grid
static int dosim = !0;
static int dovorticity = !0;
static int draw_gridcells = 0;
static int override_dosim = 0;
static int draw_mode_rigidbodies = DrawModes::DrawMode::HIDE;
static int mouse_mode = 0;
static bool enable_fixed_object = false;


static float* u, * v, * u_prev, * v_prev;
static float* dens, * dens_prev;
static float* vor;


static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;

static float current_time;



/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/




static void free_data(void)
{
	if (u) free(u);
	if (v) free(v);
	if (u_prev) free(u_prev);
	if (v_prev) free(v_prev);
	if (dens) free(dens);
	if (dens_prev) free(dens_prev);
	if (vor) free(vor);
}

static void clear_data(void)
{
	int i, size = (N + 2) * (N + 2);

	for (i = 0; i < size; i++) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
	}
}

static int allocate_data(void)
{
	int size = (N + 2) * (N + 2);

	u = (float*)malloc(size * sizeof(float));
	v = (float*)malloc(size * sizeof(float));
	u_prev = (float*)malloc(size * sizeof(float));
	v_prev = (float*)malloc(size * sizeof(float));
	dens = (float*)malloc(size * sizeof(float));
	dens_prev = (float*)malloc(size * sizeof(float));
	vor = (float*)malloc(size * sizeof(float));

	if (!u || !v || !u_prev || !v_prev || !dens || !dens_prev || !vor) {
		fprintf(stderr, "cannot allocate data\n");
		return (0);
	}

	return (1);
}


/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display(void)
{
	glViewport(0, 0, win_x, win_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, 1.0, 0.0, 1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void)
{
	glutSwapBuffers();
}
static void draw_edges(void) 
{
	int i, j;
	float x, y, h;

	h = 1.0f / N;
	glColor3f(0.0f, 1.0f, 0.0f);
	glLineWidth(3.0f);

	glBegin(GL_LINES);
	for (const Edge & edge : edges)
	{
		int x2 = edge.a.x-1;
		int y2 = edge.a.y-1;
		int x1 = edge.b.x-1;
		int y1 = edge.b.y-1;
			
		if (x1 != x2){
			
			x = h * max(x1, x2);
			y = h * y1;
			glVertex2f(x, y);
			glVertex2f(x, y+h);
	
		}else{
			
			x = h * x1;
			y = h * max(y1, y2);
			glVertex2f(x, y);
			glVertex2f(x+h, y);
	
		}
	}
	glEnd();

}
static void draw_velocity(void)
{
	int i, j;
	float x, y, h;

	h = 1.0f / N;

	glColor3f(1.0f, 1.0f, 1.0f);
	glLineWidth(1.0f);

	glBegin(GL_LINES);

	for (i = 1; i <= N; i++) {
		x = (i - 0.5f) * h;
		for (j = 1; j <= N; j++) {
			y = (j - 0.5f) * h;

			glVertex2f(x, y);
			glVertex2f(x + u[IX(i, j)], y + v[IX(i, j)]);
		}
	}

	glEnd();
}

static void draw_gridlines(void){
	int i;
	float t, h;
	h = 1.0f / N;

	glColor3f(0.5f, 0.5f, 0.5f);
	glLineWidth(1.0f);

	glBegin(GL_LINES);

	for (i = 0; i <= N+1; i++){
		t = (i-1)*h;


		// horizontal lines
		glVertex2f(0.0f, t);
		glVertex2f(1.0f, t);

		// vertizontal lines
		glVertex2f(t, 0.0f);
		glVertex2f(t, 1.0f);
	}

	glEnd();
}

static void draw_density(void)
{
	int i, j;
	float x, y, h, d00, d01, d10, d11;

	h = 1.0f / N;

	glBegin(GL_QUADS);

	for (i = 0; i <= N+1; i++) {
		if (DEBUG_DRAWING)
			x = (i-1) * h;
		else
			x = (i - 0.5f) * h;
		
		for (j = 0; j <= N+1; j++) {
			if (DEBUG_DRAWING){
				y = (j-1)*h;
				d00 = dens[IX(i, j)];
				if (d00 <= 0.001) continue;
				glColor3f(d00, d00, d00); glVertex2f(x, y);
				glColor3f(d00, d00, d00); glVertex2f(x + h, y);
				glColor3f(d00, d00, d00); glVertex2f(x + h, y + h);
				glColor3f(d00, d00, d00); glVertex2f(x, y + h);
			}else{
				y = (j - 0.5f) * h;

				d00 = dens[IX(i, j)];
				d01 = dens[IX(i, j + 1)];
				d10 = dens[IX(i + 1, j)];
				d11 = dens[IX(i + 1, j + 1)];
				if (d00 + d01 + d10 + d11 <= 0.001) continue;

				glColor3f(d00, d00, d00); glVertex2f(x, y);
				glColor3f(d10, d10, d10); glVertex2f(x + h, y);
				glColor3f(d11, d11, d11); glVertex2f(x + h, y + h);
				glColor3f(d01, d01, d01); glVertex2f(x, y + h);
			}
		}
	}
	glEnd();
}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI(float* d, float* u, float* v)
{
	int i, j, size = (N + 2) * (N + 2);

	// for (i = 0; i < size; i++) {
	// 	u[i] = v[i] = d[i] = 0.0f;
	// }

	i = (int)((mx / (float)win_x) * N + 1);
	j = (int)(((win_y - my) / (float)win_y) * N + 1);

	if (i<1 || i>N || j<1 || j>N) return;

	if (mouse_down[0]) {
		// printf("Clicked LMB! mx: %i omx: %i my: %i omy: %i\n", mx, omx, my, omy);
		u[IX(i, j)] += force * (mx - omx);
		v[IX(i, j)] += force * (omy - my);
	}

	if (mouse_down[2]) {
		d[IX(i, j)] += source;
	}
	if (mouse_mode == 0)
	{
		if (mouse_down[1])
			{
				int prevx = (int)((omx / (float)win_x) * N + 1);
				int prevy = j = (int)(((win_y - omy) / (float)win_y) * N + 1);
				targetx = i;
				targety = j;
				if (targetx - mObj->size < 1) targetx = 1 + mObj->size;
				if (targety - mObj->size < 1) targety = 1 + mObj->size;
				if (targetx + mObj->size > N) targetx = N - mObj->size;
				if (targety + mObj->size > N) targety = N - mObj->size;
			}
	}
	else
	{
		if (mouse_down[1]) 
		{
			float xd, yd;
			float fractx = modf((mx / (float)win_x) * N + 1, &xd);
			int x = (int)xd;
			float fracty = modf(((win_y - my) / (float)win_y) * N + 1, &yd);
			int y = (int)yd;
			int diffx = 0;
			int diffy = 0;
			if (fabs(0.5 - fractx) > fabs(0.5 - fracty)){
				diffx = fractx < 0.5 ? -1 : 1;
			}else{
				diffy = fracty < 0.5 ? -1 : 1;
			}
 			Point prev = Point(x+diffx, y+diffy);
			Point now = Point(x, y);
			Edge edge = {prev, now};
			edges.insert(edge);
		}
	}
	

	omx = mx;
	omy = my;

	return;
}

static void clear_UI_data(float * d, float * u, float * v){
	std::memset(d, 0, (N+2)*(N+2)*sizeof(float));
	std::memset(u, 0, (N+2)*(N+2)*sizeof(float));
	std::memset(v, 0, (N+2)*(N+2)*sizeof(float));
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'c':
	case 'C':
		clear_data();
		current_time = 0;
		break;

	case 'q':
	case 'Q':
		free_data();
		exit(0);
		break;

	case 'v':
	case 'V':
		dvel = !dvel;
		break;

	case 'g':
	case 'G':
		dgrid = !dgrid;
		break;

	case 'd':
	case 'D':
		DEBUG_DRAWING = !DEBUG_DRAWING;
		break;

	case 'r':
	case 'R':
		(++draw_mode_rigidbodies) %= DrawModes::num_draw_modes;
		break;

	case 'p':
	case 'P':
		dosim = !dosim;
		break;

	case '.':
	case '>':
		override_dosim += 1;
		break;
	case 'e':
	case 'E':
		mouse_mode = mouse_mode == 0 ? 1 : 0;
		break;

	case 'o':
	case 'O':
		dovorticity = !dovorticity;
		printf(" >>> Toggled vorticity %s\n", dovorticity ? "on" : "off");
		break;

	case 'b':
	case 'B':
		draw_gridcells = !draw_gridcells;
		break;
	
	case 'f':
	case 'F':
		enable_fixed_object = !enable_fixed_object;
		mObj->enabled = enable_fixed_object;
		break;
	}



}

static void mouse_func(int button, int state, int x, int y)
{
	omx = mx = x;
	omx = my = y;

	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func(int x, int y)
{
	omx = mx;
	omy = my;
	mx = x;
	my = y;
}

static void reshape_func(int width, int height)
{
	glutSetWindow(win_id);
	glutReshapeWindow(width, height);

	win_x = width;
	win_y = height;
}

static void idle_func(void)
{
	pre_display();
	get_from_UI(dens_prev, u_prev, v_prev);
	if (override_dosim != 0 || dosim != 0){

		mObj->setCenter(targetx, targety, N);
		mObj->force(u, v, dens, N, dt);

		vel_step(N, u, v, u_prev, v_prev, visc, dt, vor, dovorticity);
		dens_step(N, dens, dens_prev, u, v, diff, dt);
		clear_UI_data(dens_prev, u_prev, v_prev);
		// update rigidbodycollection
		if (draw_mode_rigidbodies != DrawModes::DrawMode::HIDE){
			rbc.step(dt);
			rbc.apply_force_to_liquid(u, v, dt, N);
		}

		// Count time
		current_time += dt;
		override_dosim--;
		if (override_dosim < 0) override_dosim=0;
	}
	glutSetWindow(win_id);
	glutPostRedisplay();
}

static char *title_buff = (char *) malloc(sizeof(char) * 1024);
static void display_func(void)
{

	if (dvel) draw_velocity();
	else		draw_density();
	if (dgrid) draw_gridlines();

	
	mObj->draw(1.0 / N);
	draw_edges();
	rbc.draw(DrawModes::modes[draw_mode_rigidbodies], draw_gridcells != 0, N);
	
	//update title
	float dens_sum = 0;
	for(int x = 1; x <= N; x++) for (int y = 1; y <= N; y++){
		if (x >= mObj->cenX-mObj->size && x <= mObj->cenX+mObj->size && 
		    y >= mObj->cenY-mObj->size && y <= mObj->cenY+mObj->size) continue;
		dens_sum += dens[IX(x, y)];
	} 
    sprintf(title_buff, "Assignment 2 - t: %.3f - total dens: %.3f", current_time, dens_sum);
    glutSetWindowTitle(title_buff);
	


	post_display();
}


/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window(void)
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("Alias | wavefront");

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();

	pre_display();

	glutKeyboardFunc(key_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutReshapeFunc(reshape_func);
	glutIdleFunc(idle_func);
	glutDisplayFunc(display_func);
}


/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main(int argc, char** argv)
{


	glutInit(&argc, argv);

	if (argc != 1 && argc != 6) {
		fprintf(stderr, "usage : %s N dt diff visc force source\n", argv[0]);
		fprintf(stderr, "where:\n"); \
			fprintf(stderr, "\t N      : grid resolution\n");
		fprintf(stderr, "\t dt     : time step\n");
		fprintf(stderr, "\t diff   : diffusion rate of the density\n");
		fprintf(stderr, "\t visc   : viscosity of the fluid\n");
		fprintf(stderr, "\t force  : scales the mouse movement that generate a force\n");
		fprintf(stderr, "\t source : amount of density that will be deposited\n");
		exit(1);
	}

	if (argc == 1) {
		N = 16;
		dt = 0.01f;
		diff = 0.01f*dt;
		visc = 0.0f;
		force = 5.0f;
		source = 100.0f;
		fprintf(stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
			N, dt, diff, visc, force, source);
	}
	else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		diff = atof(argv[3]);
		visc = atof(argv[4]);
		force = atof(argv[5]);
		source = atof(argv[6]);
	}

	printf("\n\nHow to use this demo:\n\n");
	printf("\t Add densities with the right mouse button\n");
	printf("\t Add velocities with the left mouse button and dragging the mouse\n");
	printf("\t Place edges using middle mouse button after going in placement mode\n");
	printf("\t Set target for fixed object with middle mouse (after enabling)\n");
	printf("\t*-------------------------------------------------------*\n");
	printf("\t|   'v': Toggle density/(v)elocity display              |\n");
	printf("\t|   'o': V(o)rticity toggle                             |\n");
	printf("\t|   'c': (C)lear the simulation                         |\n");
	printf("\t|   'g': Draw (g)ridlines                               |\n");
	printf("\t|   'd': Toggle smooth (d)rawing                        |\n");
	printf("\t|   'e': Toggle (e)dge placement mode                   |\n");
	printf("\t|   'b': Toggle rigidbody cell drawing -> (B)resenham   |\n");
	printf("\t|   'r': Cycle (r)igidbody mode                         |\n");
	printf("\t|   'f': Toggle (f)ixed object                          |\n");
	printf("\t|   'p': (P)ause                                        |\n");
	printf("\t|   '>': Step forward once (can be done while paused)   |\n");
	printf("\t|   'q': (Q)uit                                         |\n");
	printf("\t*-------------------------------------------------------*\n");

	dvel = 0;
	dgrid = 0;

	if (!allocate_data()) exit(1);
	clear_data();
	omx = 0;
	omy = 0;
	win_x = 512;
	win_y = 512;
	open_glut_window();


	Shape shape2 = Shape();
	shape2.points.push_back(Vec2(0.05, 0.05) + Vec2(0.25, 0.35));
	shape2.points.push_back(Vec2(0.05, 0.05) + Vec2(0.35, 0.35));
	shape2.points.push_back(Vec2(0.05, 0.05) + Vec2(0.35,  0.3));
	shape2.points.push_back(Vec2(0.05, 0.05) + Vec2( 0.3, 0.25));
	shape2.points.push_back(Vec2(0.05, 0.05) + Vec2(0.25, 0.25));
	shape2.centroid = Vec2(0.02, 0.02) + Vec2(0.275, 0.325);
	//TRIANGULATE 
	shape2.triangulation.push_back(0);
	shape2.triangulation.push_back(1);
	shape2.triangulation.push_back(2);
	shape2.triangulation.push_back(2);
	shape2.triangulation.push_back(3);
	shape2.triangulation.push_back(4);
	shape2.triangulation.push_back(0);
	shape2.triangulation.push_back(2);
	shape2.triangulation.push_back(4);
	shape2.post_ctor();
	rbs.push_back(new Rigidbody(shape2));
	

	Shape shape3 = Shape();
	Vec2 offset = Vec2(0.6, 0.6);
	shape3.points.push_back(offset+Vec2(5*0.045, 5*0.045));
	shape3.points.push_back(offset+Vec2(5*0.045, 5*0.075));
	shape3.points.push_back(offset+Vec2(5*0.055, 5*0.075));
	shape3.points.push_back(offset+Vec2(5*0.055, 5*0.045));
	shape3.centroid = offset+Vec2(5*0.05, 5*0.05);
	shape3.triangulation = {0, 1, 2,  0, 2, 3};
	shape3.post_ctor();
	rbs.push_back(new Rigidbody(shape3));


	Shape shape4 = Shape();
	offset = Vec2(0.2, 0.7);
	shape4.points.push_back(offset + Vec2(4* 0.01, 4* 0.01));
	shape4.points.push_back(offset + Vec2(4* 0.03, 4*    0));
	shape4.points.push_back(offset + Vec2(4* 0.01, 4*-0.01));
	shape4.points.push_back(offset + Vec2(4*    0, 4*-0.03));
	shape4.points.push_back(offset + Vec2(4*-0.01, 4*-0.01));
	shape4.points.push_back(offset + Vec2(4*-0.03, 4*    0));
	shape4.points.push_back(offset + Vec2(4*-0.01, 4* 0.01));
	shape4.points.push_back(offset + Vec2(4*0,     4* 0.03));
	shape4.centroid = Vec2(0.05, 0.7);
	shape4.post_ctor();
	rbs.push_back(new Rigidbody(shape4));

	rbc.addRB(rb);
	for (Rigidbody* rb : rbs) rbc.addRB(rb);
	// rbc.addRB(rbs[1]);
	rb->omega = 0.2*3.141592*0.2144451*dt;
	rbs[0]->omega = 0.2*-3.141592*dt;
	rbs[1]->omega = 1*dt;
	rbs[2]->omega = 1.57152*dt;
	//VERY IMPORTANT
	rbc.init();

	glutMainLoop();

	exit(0);
}