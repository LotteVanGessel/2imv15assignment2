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
	size = 5;
}

void Object::draw(float h) {

	float x0 = (cenX - size - 1) * h;
	float x1 = (cenX + size) * h;
	float y0 = (cenY - size - 1) * h;
	float y1 = (cenY + size) * h;

	int mode = 0;
	glBegin(mode == 0 ? GL_LINE_LOOP : GL_QUADS);
	glLineWidth(4.0f);
	//Pink?
	glColor3f(0.7f, 0.5f, 0.5f);

	glVertex2f(x0, y0);
	glVertex2f(x1, y0);

	glVertex2f(x1, y1);
	glVertex2f(x0, y1);
	
	glEnd();
}

void Object::setCenter(int cX, int cY, int N)
{
	if (std::abs(cX - cenX) > 1)
	{
		int increment = (cX - cenX) / std::abs(cX - cenX);
		velx = increment;
		cenX += increment; 
	}
	else {
		velx = cX-cenX;
		cenX = cX;
	}
	if (std::abs(cY - cenY) > 1)
	{
		int increment = (cY - cenY) / std::abs(cY - cenY);
		vely = increment;
		cenY += increment;
	}
	else {
		vely = cY-cenY;
		cenY = cY;
	}
	if (cenX - size < 0)
	{
		cenX = size;
	}
	if (cenY - size < 0)
	{
		cenY = size;
	}
	if (cenX + size > N)
	{
		cenX = N - size;
	}
	if (cenY + size > N)
	{
		cenY = N - size;
	}
}


//works for object
void Object::setBound(int N, int b, float* mat)
{
	int i;
	int x = cenX;
	int y = cenY;
	int a = size;
	for (i = x - a + 1; i <= x + a-1; i++) {
		if (y-a > 1) mat[IX(i, y - a)] = b == 2 ? -mat[IX(i, y - a - 1)] : mat[IX(i, y - a - 1)];
		if (y+a < N) mat[IX(i, y + a)] = b == 2 ? -mat[IX(i, y + a + 1)] : mat[IX(i, y + a + 1)];
		// mat[IX(i, y - a + 1)] = 1;
		// mat[IX(i, y + a - 1)] = 1;
		
	}
	for (i = y - a+1; i <= y + a-1; i++) {
		if (x-a > 1) mat[IX(x - a, i)] = b == 1 ? -mat[IX(x - a - 1, i)] : mat[IX(x - a - 1, i)];
		if (y+a < N) mat[IX(x + a, i)] = b == 1 ? -mat[IX(x + a + 1, i)] : mat[IX(x + a + 1, i)];
		// mat[IX(x - a+1, i)] = 1;
		// mat[IX(x + a-1, i)] = 1;
	}
	int j, k;
	
	// mat[IX(x - a - 1, y - a - 1)] = 0.5f * (mat[IX(x - a, y - a)] + mat[IX(x - a, y - a)]);
	// mat[IX(x - a - 1, y + a + 1)] = 0.5f * (mat[IX(x - a, y + a)] + mat[IX(x - a, y + a)]);
	// mat[IX(x + a + 1, y - a - 1)] = 0.5f * (mat[IX(x + a, y - a)] + mat[IX(x + a, y - a)]);
	// mat[IX(x + a + 1, y + a + 1)] = 0.5f * (mat[IX(x + a, y + a)] + mat[IX(x + a, y + a)]);
	
	if (x-a > 1){
		if(y-a > 1) mat[IX(x-a, y-a)] = 0.5f * (mat[IX(x - a - 1, y - a)] + mat[IX(x-a, y-a-1)]);
		if(y+a < N) mat[IX(x-a, y+a)] = 0.5f * (mat[IX(x - a - 1, y + a)] + mat[IX(x-a, y+a+1)]);
	}

	if (x+a < N){
		if(y-a > 1) mat[IX(x+a, y-a)] = 0.5f * (mat[IX(x + a + 1, y - a)] + mat[IX(x+a, y-a-1)]);
		if(y+a < N) mat[IX(x+a, y+a)] = 0.5f * (mat[IX(x + a + 1, y + a)] + mat[IX(x+a, y+a+1)]);
	}

	for (j = x - a + (x-a == 1 ? 0 : 1); j <= x + a - (x+a == N ? 0 : 1); j++)
	{
		for (k = y - a + (y-a == 1 ? 0 : 1); k <= y + a - (y+a == N ? 0 : 1); k++)
		{
			mat[IX(j, k)] = 0;
		}
	}
}


void Object::force(float* u, float* v, float* dens, int N)
{
	int sign_x = velx > 0 ? 1 : -1;
	sign_x = velx == 0 ? 0 : sign_x;
	int sign_y = vely > 0 ? 1 : -1;
	sign_y = vely == 0 ? 0 : sign_y;

	if (sign_x == 0 && sign_y == 0) return;
	
	if (sign_x != 0){
		int ahead = cenX + (size+1)*sign_x;
		int behind = cenX - (size+1)*sign_x; 
		for (int j = cenY - size+1; j < cenY + size; j++) {
			dens[IX(ahead, j)] += dens[IX(ahead - sign_x, j)];
			dens[IX(behind, j)] = 0;
			u[IX(ahead, j)] += sign_x * dens[IX(ahead, j)] * 10; // TODO: arbitrary constant
		}

		//hoekjes
		dens[IX(ahead, cenY-size)] += 0.5f*dens[IX(ahead-sign_x, cenY-size)];
		dens[IX(ahead, cenY+size)] += 0.5f*dens[IX(ahead-sign_x, cenY+size)];

		dens[IX(ahead-sign_x, cenY-size-1)] += 0.5f*dens[IX(ahead-sign_x, cenY-size)];
		dens[IX(ahead-sign_x, cenY+size+1)] += 0.5f*dens[IX(ahead-sign_x, cenY+size)];

	}

	if (sign_y != 0){
		int ahead = cenY + (size+1)*sign_y;
		int behind = cenY - (size+1)*sign_y; 
		for (int i = cenX - size + 1; i < cenX + size; i++) {
			dens[IX(i, ahead)] += dens[IX(i, ahead-sign_y)];
			dens[IX(i, behind)] = 0;
			v[IX(i, ahead)] += sign_y * dens[IX(i, ahead)] * 10; // TODO: arbitrary constant
		}

		//hoekjes
		dens[IX(cenX-size, ahead)] += 0.5f*dens[IX(cenX-size, ahead-sign_y)];
		dens[IX(cenX+size, ahead)] += 0.5f*dens[IX(cenX+size, ahead-sign_y)];

		dens[IX(cenX-size-1, ahead-sign_y)] += 0.5f*dens[IX(cenX-size, ahead-sign_y)];
		dens[IX(cenX+size+1, ahead-sign_y)] += 0.5f*dens[IX(cenX+size, ahead-sign_y)];

	}
}


void Object::velocity(int vx, int vy)
{
	velx = vx; 
	vely = vy;
}
