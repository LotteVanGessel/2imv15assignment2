#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "MatrixMath.h"
#include "Shape.h"
#include "Rigidbody.h"

struct AABB{
    int left, bottom, right, top;
};


class SplitLine{
    public:
        Vec2*  p;
        Vec2*  q;
        Vec2 normal;
        Vec2 dir;
        void update();
};

class Contact{
    Vec2 temp;
    Rigidbody* swapper;
    public:
        bool is_contact;
        Rigidbody* rb1;
        Rigidbody* rb2;
        SplitLine l0;
        SplitLine l1;
        Contact(Rigidbody* rb1, Rigidbody* rb2);
        bool FindSplitLine(Rigidbody* rb1, Rigidbody* rb2);
        bool FindSplitLine();
        bool step();
        void cycle();
};


bool is_intersect(AABB &X, AABB &Y);

class CollisionHelper{
    std::vector<Rigidbody*> rbs;
    std::vector<Contact> contacts;
    bool last_result_is_legal_position;
    public:
        CollisionHelper();
        CollisionHelper(std::vector<Rigidbody*> rbs);
        bool is_legal_position();
        void draw();

};

#endif