#include "Rigidbody.h"
#include <GL/glut.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>


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


void Shape::draw(Mat2 &rotation, Vec2 &offset, float r = 1.0f, float g = 1.0f, float b = 1.0f){
	glLineWidth(1.0f);
	glColor3f(r, g, b);
    glBegin(GL_LINE_LOOP);
    // printf("Matrix R:\n\t[%5.3f\t%5.3f]\n\t[%5.3f\t%5.3f]\nVec offset:\n\t[%5.3f\t%5.3f]\n", rotation[0], rotation[1], rotation[2], rotation[3], offset[0], offset[1]);
    for (Vec2 & v : rel_points){
        matmult(rotation, v, temp);
        vecadd(offset, temp, temp);
        glVertex2f(temp[0], temp[1]);
    }
	glEnd();
}

Rect::Rect(Vec2 &botleft, Vec2 &topright){
    points = std::vector<Vec2>{
                                Vec2( botleft[0],  botleft[1]),
                                Vec2(topright[0],  botleft[1]),
                                Vec2(topright[0], topright[1]),
                                Vec2( botleft[0], topright[1])
                                };
    rel_points = std::vector<Vec2>();
    triangulation = std::vector<int>();
    post_ctor(); 
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
    for (Rigidbody *rb : rbs) rb->dxdt(y);

    scalarmult(dt, Dxdt, n); // Dxdt = dt * xdot_k
    vecadd(x0, Dxdt, x1, n); // x1 = x_k + dt * xdot_k, gewoon euler forward

    y = x1;
    for (Rigidbody *rb : rbs){
        rb->update_state(y);
        y += Rigidbody::STATE_SIZE;
    }
}

RigidbodyCollection::RigidbodyCollection(){
    rbs = std::vector<Rigidbody*>();
}

// RigidbodyCollection::~RigidbodyCollection(){
//     free(x0);
//     free(x1);
//     free(temp);
//     free(Dxdt);
// }
void RigidbodyCollection::init(){
    is_active = true;
    n = Rigidbody::STATE_SIZE * rbs.size();
    x0   = (float*) malloc(n* sizeof(float));
    x1   = (float*) malloc(n* sizeof(float));
    temp = (float*) malloc(n* sizeof(float));
    Dxdt = (float*) malloc(n* sizeof(float));
    copy_states(x1);
}

void RigidbodyCollection::copy_states(float* dst, int l, int r){
    for (int i = l; i < ((r < 0) ? rbs.size() : r); ++i) 
        std::memcpy(dst+(i*Rigidbody::STATE_SIZE), rbs[i]->state, Rigidbody::STATE_SIZE*sizeof(float));
}

inline void resize(float* x, float* temp, int n, int m){
    std::memcpy(temp, x, Rigidbody::STATE_SIZE*n*sizeof(float));
    free(x); // give memory back
    x = (float*) malloc(Rigidbody::STATE_SIZE*m*sizeof(float));
    std::memcpy(x, temp, Rigidbody::STATE_SIZE*n*sizeof(float));
}

inline void RigidbodyCollection::resize_all(int old_size, int new_size){
    resize(x0, temp, old_size, new_size);
    resize(x1, temp, old_size, new_size);
    resize(Dxdt, temp, old_size, new_size);
    free(temp); // give memory back
    temp = (float*) malloc(Rigidbody::STATE_SIZE*new_size*sizeof(float));

    copy_states(x1, old_size, new_size);

    //Warning messsage printing
    if (resize_all_calls++ >= resize_all_limit){
        printf("Warning: RigidbodyCollection::resize_all is inefficient and repeated calls should be avoided...\n");
        resize_all_calls = 0;
    }
}

void RigidbodyCollection::addRB(Rigidbody* rb){
    rbs.push_back(rb);
    if (is_active){
        // we need to resize everything
        resize_all(n, n+1);
        n = n+1;
    }
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
    std::memset(state, 0, STATE_SIZE*sizeof(float));


    // Init position at centre of shape
    x[0] = shape.centroid[0];
    x[1] = shape.centroid[1];

    //Init R matrix to unit matrix
    R[0] = 1;
    R[3] = 1;
    // printf("x: %i : expected %i\n", x.data, state);
    // printf("R: %i : expected %i\n", R.data, state + x.n);
    // printf("P: %i : expected %i\n", P.data, state + x.n + R.n);
    // printf("L: %i : expected %i\n", L.data, state + x.n + R.n + P.n);
    v = Vec2();
    F = Vec2();
    tau = Vec2();
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

void Rigidbody::update_state(float* new_state){ std::memcpy(state, new_state, STATE_SIZE*sizeof(float)); }
void Rigidbody::draw(){ 
    shape.draw(R, x, 0.0f, 1.0f, 1.0f); 
}