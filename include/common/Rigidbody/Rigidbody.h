#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "MatrixMath.h"
#include "Shape.h"
#include <vector>



// Based on lecture notes
class Rigidbody{

    public:
    Shape shape;

    
    // Constants
    // TODO: Deze dingen doen
    float mass = 1;
    float Ibody;
    float Ibodyinv;    

    float* state; // Points to x(t) R(t) P(t) and L(t)
    // State variables
    static const int STATE_SIZE = 10;
    Vec2 x; // Position (t)
    Mat2 R; // Rotation (t)
    Vec2 P; // LMomentum(t) aka Mv(t)
    Vec2 L; // AMomentum(t) aka I(t)omega(t)

    // Derived quantities (auxiliary variables)
    float Iinv;
    Vec2 v;
    float omega;
    RotMat2 omega_mat;
    Mat2 Rdot; // used to store omega_mat * R

    // Computed quantities
    Vec2 F;   // Force(t)
    Vec2 tau; // Tau(t) -> torque
    Rigidbody(Shape shape);
    void dxdt(float* y, float dt);
    void update_state(float* new_state);
    void draw(DrawModes::DrawMode mode, bool draw_grid_cells, int N);
    void apply_force_to_liquid(float* u, float * v, float dt, int N);
    void print();
};
#endif