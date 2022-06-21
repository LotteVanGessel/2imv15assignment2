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

inline void calc_point(Mat2 &R, Vec2 &off, Vec2 &v, Vec2 &res){
    matmult(R, v, res);
    vecadd(off, res, res);
}

void Shape::draw(Mat2 &rotation, Vec2 &offset, DrawModes::DrawMode mode, float r = 1.0f, float g = 1.0f, float b = 1.0f){
	glLineWidth(1.0f);
	glColor3f(r, g, b);
    // printf("Matrix R:\n\t[%5.3f\t%5.3f]\n\t[%5.3f\t%5.3f]\nVec offset:\n\t[%5.3f\t%5.3f]\n", rotation[0], rotation[1], rotation[2], rotation[3], offset[0], offset[1]);
    
    switch(mode){
        case DrawModes::LINES:
            glBegin(GL_LINE_LOOP);
            for (Vec2 & v : rel_points){
                calc_point(rotation, offset, v, temp);
                glVertex2f(temp[0], temp[1]);
            }
            break;
        
        case DrawModes::TRIS:
            glBegin(GL_LINES);
            for (int i = 0; i < triangulation.size(); i+=3){
                Vec2 & a = rel_points[triangulation[i]];
                Vec2 & b = rel_points[triangulation[i+1]];
                Vec2 & c = rel_points[triangulation[i+2]];
                
                calc_point(rotation, offset, a, temp);
                glVertex2f(temp[0], temp[1]);
                calc_point(rotation, offset, b, temp);
                glVertex2f(temp[0], temp[1]);

                glVertex2f(temp[0], temp[1]);
                calc_point(rotation, offset, c, temp);
                glVertex2f(temp[0], temp[1]);

                glVertex2f(temp[0], temp[1]);
                calc_point(rotation, offset, a, temp);
                glVertex2f(temp[0], temp[1]);
            }
            break;
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
    int i = 0;
    for (Rigidbody *rb : rbs) rb->dxdt(Dxdt + (i++)*Rigidbody::STATE_SIZE, dt);
    scalarmult(dt, Dxdt, n); // Dxdt = dt * xdot_k
    vecadd(x0, Dxdt, x1, n); // x1 = x_k + dt * xdot_k, gewoon euler forward

    float* y = x1;
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

void RigidbodyCollection::draw(DrawModes::DrawMode mode){
    for (Rigidbody* rb : rbs) rb->draw(mode);
}

void RigidbodyCollection::print(){
    printf("Rigidbodycollection has size %i\n", rbs.size());
    for (int i = 0; i < rbs.size(); i++) { printf("  RB - %i:\n", i); rbs[i]->print(); }
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
    omega_mat = RotMat2();
    Rdot = Mat2();
}

void Rigidbody::dxdt(float* y, float dt){
    *y++ = v[0];
    *y++ = v[1];

    omega_mat.update(omega);
    // (w - I)
    omega_mat[0] -= 1;
    omega_mat[3] -= 1;
    // Rdot = (w - I)R
    matmult(omega_mat, R, Rdot);
    // Rdot = (1/dt) * (w - I)R
    scalarmult(1/dt, Rdot, Rdot);
    *y++ = Rdot[0];
    *y++ = Rdot[1];
    *y++ = Rdot[2];
    *y++ = Rdot[3];

    
    *y++ = F[0];
    *y++ = F[1];

    *y++ = tau[0];
    *y++ = tau[1];
}

void Rigidbody::update_state(float* new_state){ 
    std::memcpy(state, new_state, STATE_SIZE*sizeof(float)); 
}
void Rigidbody::draw(DrawModes::DrawMode mode){ 
    shape.draw(R, x, mode, 0.0f, 1.0f, 1.0f); 
}

void Rigidbody::print(){
    printf("    x:\n"); x.print();
    printf("    R:\n"); R.print();
    printf("    P:\n"); P.print();
    printf("    L:\n"); L.print();
}