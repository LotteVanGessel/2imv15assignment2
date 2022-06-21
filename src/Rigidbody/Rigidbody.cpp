#include "Rigidbody.h"
#include "Shape.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>


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