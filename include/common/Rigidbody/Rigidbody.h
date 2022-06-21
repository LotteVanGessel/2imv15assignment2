#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "MatrixMath.h"
#include <vector>

namespace DrawModes{
    enum DrawMode{LINES, TRIS};
    static const int num_draw_modes = 2;
    static const DrawMode modes[num_draw_modes] = {LINES, TRIS};
}

class Shape{
    public:

        std::vector<Vec2> points;
        Vec2 centroid;
        Vec2 temp;
        
        std::vector<Vec2> rel_points;

        std::vector<int> triangulation; // 3*n vector of triangle corners. Used for inertia calculation

        void calc_rel_points();
        virtual void triangulate(){};
        virtual void calculate_centroid(){};
        void draw(Mat2 &rotation, Vec2 &offset, DrawModes::DrawMode mode, float r, float g, float b);

        void post_ctor();

        Shape(){}
};

class Rect : public Shape{
    public:
        Rect(Vec2 &botleft, Vec2 &topright);

        void calculate_centroid();
        void triangulate();
};



// Based on lecture notes
class Rigidbody{

    public:
    Shape shape;

    
    // Constants
    // TODO: Deze dingen doen
    float mass;
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
    void draw(DrawModes::DrawMode mode);

    void print();
};

class RigidbodyCollection{
    std::vector<Rigidbody*> rbs;
    float* x0;
    float* x1;
    float* Dxdt;
    float* temp;

    int n;

    bool is_active = false;
    int resize_all_calls = 0;
    int resize_all_limit = 10;
    public:
        RigidbodyCollection();

        void init();

        void copy_states(float* dst, int l = 0, int r = -1);

        void step(float dt);

        void computeforceandtorque();

        void addRB(Rigidbody* rb);

        inline void resize_all(int old_size, int new_size);

        void draw(DrawModes::DrawMode mode);

        void print();
};
#endif