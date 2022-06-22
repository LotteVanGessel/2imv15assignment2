#ifndef RIGIDBODYCOLLECTION_H
#define RIGIDBODYCOLLECTION_H
#include <vector>
#include "Rigidbody.h"
#include "Collisiondetection.h"
class RigidbodyCollection{
    CollisionHelper ch;
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