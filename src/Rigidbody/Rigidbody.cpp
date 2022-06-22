#include "Rigidbody.h"
#include "Shape.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <GL/glut.h>

static inline int IX(int i, int j, int N){ return i+(N+2)*j; };


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

void Rigidbody::apply_force_to_liquid(float* u, float * v, float dt, int N){
    std::set<std::pair<Point, Vec2>> grid_cells = shape.get_grid_cells(N);
    float constant = 5e-1f * mass / (dt * grid_cells.size());

    // printf("================================\n");
    for (const std::pair<Point, Vec2> & pair : grid_cells){
        // printf("p: %i %i\n", pair.first.x, pair.first.y);
        // printf("v: %.8f %.8f\n", pair.second[0], pair.second[1]);
        // printf("c: %.3f %.3f\n", constant * mass * pair.second[0] / dt, constant * mass * pair.second[1] / dt);
        if (pair.first.x < 0 || pair.first.y < 0 || pair.first.x > N+1 || pair.first.y > N+1) continue;
        u[IX(pair.first.x, pair.first.y, N)] += constant * pair.second[0]; 
        v[IX(pair.first.x, pair.first.y, N)] += constant * pair.second[1];
        free(pair.second.data);
    }
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
    shape.update_world_space(R, x);
}

void Rigidbody::draw(DrawModes::DrawMode mode, bool draw_grid_cells, int N){ 
    shape.draw(R, x, mode, 0.0f, 1.0f, 1.0f); 
    if (draw_grid_cells){
        float h = 1.0/N;
        float hh = 0.5f*hh;
        auto grid_cells = shape.get_grid_cells(N);
        glBegin(GL_QUADS);
        glColor3f(0.5, 0.0, 1.0);
        for (auto p : grid_cells){
            glVertex2f(p.first.x*h-h, p.first.y*h-h);
            glVertex2f(p.first.x*h-h, p.first.y*h);
            glVertex2f(p.first.x*h, p.first.y*h);
            glVertex2f(p.first.x*h, p.first.y*h-h);
        }
        glEnd();

        glLineWidth(0.1f);
        glBegin(GL_LINES);
        glColor3f(1.0, 0, 0.5);
        for (auto p : grid_cells){
            glVertex2f(p.first.x*h-hh, p.first.y*h-hh);
            glVertex2f(p.first.x*h-hh+ (5e-1f * 1 / 0.01)*p.second[0], p.first.y*h-hh+(5e-1f * 1 / 0.01)*p.second[1]);
            
        }
        glEnd();
    }
}

void Rigidbody::print(){
    printf("    x:\n"); x.print();
    printf("    R:\n"); R.print();
    printf("    P:\n"); P.print();
    printf("    L:\n"); L.print();
}