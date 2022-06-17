#include "Object.h"
#include <GL/glut.h>
#include <iostream>

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
	glBegin(GL_LINES);
	glColor3f(0.0f, 1.0f, 0.0f);

	float x0 = (cenX - size - 0.5f) * h;
	float x1 = (cenX + size - 0.5f) * h;
	float y0 = (cenY - size - 0.5f) * h;
	float y1 = (cenY + size - 0.5f) * h;

	glVertex2f(x0, y0);
	glVertex2f(x1, y0);

	glVertex2f(x0, y0);
	glVertex2f(x0, y1);
	
	glVertex2f(x1, y0);
	glVertex2f(x1, y1);
	
	glVertex2f(x0, y1);
	glVertex2f(x1, y1);
	
	glVertex2f(0, 0);
	glVertex2f(10, 10);
	glEnd();
}

void Object::setCenter(int cX, int cY)
{
		cenX = cX;
		cenY = cY;
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

void Object::force(int diffx, int diffy) 
{
	
}

void Object::velocity(int vx, int vy)
{
	velx = vx; 
	vely = vy;
}
