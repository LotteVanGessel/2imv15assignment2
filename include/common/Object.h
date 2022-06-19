#pragma once
using namespace std;

class Object {
public:
	int cenX;
	int cenY;
	int size;
	int velx;
	int vely;
	
	Object(int cX, int cY, int s);
	void reset();
	void draw(float h);

	void setCenter(int cX, int cY, int N);
	void setBound(int N, int b, float* mat);
	void velocity(int vx, int vy);
	void force(float* u, float* v, float* dens, int N);

};