#include "Rigidbody.h"
#include <GL/glut.h>

void Shape::calc_rel_points(){
    rel_points = std::vector<Vec2>(points.size());
    int i;
    for(i = 0; i < points.size(); i++){
        rel_points[i] = points[i] - centroid;
    } 
}

void Shape::post_ctor(){
    triangulate();

    calculate_centroid();

    calc_rel_points();
}


void Shape::draw(Vec2 &offset, float r = 1.0f, float g = 1.0f, float b = 1.0f){
    glBegin(GL_LINE_LOOP);
	glLineWidth(1.0f);
	glColor3f(r, g, b);

    for (Vec2 & point : points){
        glVertex2f(point[0], point[1]);
    }

	glEnd();
}

void Rect::calculate_centroid(){
    centroid = (points[0]+points[1]+points[2]+points[3]);
    centroid *= 0.25f;
}

void Rect::triangulate(){
    triangulation = {
        0, 1, 2,
        0, 2, 3
    };
}

void RigidbodyCollection::step(float dt){
    temp = x0; x0 = x1; x1 = temp; // swap x0 and x1, avoid malloc'ing again

    computeforceandtorque();
    //Accumulate dxdt's in y
    float* y = Dxdt;
    for (Rigidbody &rb : rbs) rb.dxdt(y);

    scalarmult(dt, Dxdt, n); // Dxdt = dt * xdot_k
    vecadd(x0, Dxdt, x1, n); // x1 = x_k + dt * xdot_k, gewoon euler forward


}

// TODO: Daadwerkelijk forces en torques berekenen
void RigidbodyCollection::computeforceandtorque(){};

Rigidbody::Rigidbody(Shape shape) : shape(shape){
    state = (float*) malloc(STATE_SIZE*sizeof(float));
    // variables are pointing to parts of state vector
    x = Vec2(state);
    R = Mat2(state + x.n);
    P = Vec2(state + x.n + R.n);
    L = Vec2(state + x.n + R.n + P.n);

    Vec2 v = Vec2();
    Vec2 F = Vec2();
    Vec2 tau = Vec2();
}

void Rigidbody::dxdt(float* y){
    *y++ = v[0];
    *y++ = v[1];

    *y++ = R[0]*omega;
    *y++ = R[1]*omega;
    *y++ = R[2]*omega;
    *y++ = R[3]*omega;

    
    *y++ = F[0];
    *y++ = F[1];

    *y++ = tau[0];
    *y++ = tau[1];
}