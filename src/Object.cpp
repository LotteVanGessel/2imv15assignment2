#include "Object.h"
#include <GL/glut.h>
#include <iostream>
#include <cmath>

#define IX(i,j) ((i)+(N+2)*(j))

Object::Object(int cX, int cY, int s) {
	this->cenX = cX;
	this->cenY = cY;
	this->size = s;
}

void Object::reset() {
	cenX = 20;
	cenY = 20;
	size = 10;
}

void Object::draw(float h) {

	float x0 = (cenX - size - 0.5f) * h;
	float x1 = (cenX + size - 0.5f) * h;
	float y0 = (cenY - size - 0.5f) * h;
	float y1 = (cenY + size - 0.5f) * h;

	glBegin(GL_LINES);
	glLineWidth(1.0f);
	//Pink?
	glColor3f(0.7f, 0.5f, 0.5f);

	glVertex2f(x0, y0);
	glVertex2f(x1, y0);
	glVertex2f(x1, y1);
	glVertex2f(x0, y1);
	
	glEnd();
}

void Object::setCenter(int cX, int cY)
{
	if (std::abs(cX - cenX) > 1)
	{
		int increment = (cX - cenX) / std::abs(cX - cenX);
		velx = increment;
		cenX += increment; 
	}
	else {
		velx = cenX - cX;
		cenX = cX;
	}
	if (std::abs(cY - cenY) > 1)
	{
		int increment = (cY - cenY) / std::abs(cY - cenY);
		vely = increment;
		cenY += increment;
	}
	else {
		vely = cenY - cY;
		cenY = cY;
	}
}

void Object::setBound(int N, int b, float* mat)
{
	int i;
	int x = cenX;
	int y = cenY;
	int a = size;
	for (i = x - a; i <= x + a; i++) {
		mat[IX(i, y - a)] = b == 2 ? -mat[IX(i, y - 1 - a)] : mat[IX(i, y - 1 - a)];
		mat[IX(i, y + a)] = b == 2 ? -mat[IX(i, y + a + 1)] : mat[IX(i, y + a + 1)];
	}
	for (i = y - a; i <= y + a; i++) {
		mat[IX(x - a, i)] = b == 1 ? -mat[IX(x - 1 - a, i)] : mat[IX(x - 1 - a, i)];
		mat[IX(x + a, i)] = b == 1 ? -mat[IX(x + a + 1, i)] : mat[IX(x + a + 1, i)];
	}
	int j, k;
	for (j = x + 1 - a; j <= x + a - 1; j++)
	{
		for (k = y + 1 - a; k <= y + a - 1; k++)
		{
			mat[IX(j, k)] = 0;
		}
	}
	mat[IX(x - a, y - a)] = 0.5f * (mat[IX(x - 1 - a , y - a)] + mat[IX(x - a , y - 1 - a)]);
	mat[IX(x - a, y + a)] = 0.5f * (mat[IX(x - 1 - a, y + a)] + mat[IX(x - a, y + a + 1)]);
	mat[IX(x + a, y - a)] = 0.5f * (mat[IX(x + a + 1, y - a)] + mat[IX(x + a, y - 1  - a)]);
	mat[IX(x + a, y + a)] = 0.5f * (mat[IX(x + a + 1, y + a)] + mat[IX(x + a, y + a + 1)]);
}

/* Does not yet work completely. Problems: 
	the density at the corners of the square is not yet handled
	the square does not yet apply force on the fluid while it moves. 
	It generally does not seem to work
*/
void Object::force(float* u, float* v, float* dens, int N)
{
	
	if (velx < 0) 
	{
		int i = cenX - size;
		for (int j = cenY - size + 1; j < cenY + size; j++)
		{
			dens[IX(i - 1, j)] += dens[IX(i, j)];
			u[IX(i - 1, j)] -=  dens[IX(i, j)];
			dens[IX(i, j)] = 0;
		}
		if (vely > 0)
		{
			dens[IX(cenX - size, cenY + size + 1)] += 0.5f * dens[IX(cenX - size, cenY + size)];
			dens[IX(cenX - size - 1, cenY + size)] += 0.5f * dens[IX(cenX - size, cenY + size)];
			v[IX(cenX - size, cenY + size + 1)] += 0.5f * dens[IX(cenX - size, cenY + size)];
			u[IX(cenX - size - 1, cenY + size)] -= 0.5f * dens[IX(cenX - size, cenY + size)];
			dens[IX(cenX - size - 1, cenY - size)] += dens[IX(cenX - size, cenY - size)];
		}
		if (vely < 0)
		{
			dens[IX(cenX - size - 1, cenY - size)] += 0.5f * dens[IX(cenX - size, cenY - size)];
			dens[IX(cenX - size, cenY - size - 1)] += 0.5f * dens[IX(cenX - size, cenY - size)];
			dens[IX(cenX - size - 1, cenY + size)] +=  dens[IX(cenX - size, cenY + size)];
			v[IX(cenX - size, cenY + size - 1)] -= 0.5f * dens[IX(cenX - size, cenY - size)];
			u[IX(cenX - size - 1, cenY + size)] -= 0.5f * dens[IX(cenX - size, cenY - size)];
		}
	}

	if (velx > 0)
	{
		int i = cenX + size;
		for (int j = cenY - size + 1; j < cenY + size; j++)
		{
			dens[IX(i + 1, j)] += dens[IX(i, j)];
			u[IX(i + 1, j)] += 2 * dens[IX(i, j)];
			dens[IX(i, j)] = 0;
		}
		if (vely > 0)
		{
			dens[IX(cenX + size + 1, cenY + size)] += 0.5f * dens[IX(cenX + size, cenY + size)];
			dens[IX(cenX + size, cenY + size + 1 )] += 0.5f * dens[IX(cenX + size, cenY + size)];
			dens[IX(cenX + size + 1 , cenY - size)] += dens[IX(cenX + size, cenY - size)];


		}
		if (vely < 0)
		{
			dens[IX(cenX + size + 1, cenY + size)] += 0.5f * dens[IX(cenX + size, cenY + size)];
			dens[IX(cenX + size, cenY - size - 1)] += 0.5f * dens[IX(cenX + size, cenY - size)];
			dens[IX(cenX + size + 1, cenY - size)] += dens[IX(cenX + size, cenY - size)];
		}
	}

	if (vely < 0)
	{
		
	}

	if (vely > 0)
	{
		int i = cenY + size;
		for (int j = cenX - size + 1; j < cenX + size; j++)
		{
			dens[IX(j + 1, i)] += dens[IX(j, i)];
			dens[IX(j, i)] = 0;
		}
	}
	
}

void Object::velocity(int vx, int vy)
{
	velx = vx; 
	vely = vy;
}
