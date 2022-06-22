#include "Collisiondetection.h"
#include <GL/glut.h>
#include <cstdio>

CollisionHelper::CollisionHelper(){}

// assumption legal position initially
CollisionHelper::CollisionHelper(std::vector<Rigidbody*> rbs){
    contacts = std::vector<Contact>();
    for (int i = 0; i < rbs.size(); i++){
        for (int j = i + 1; j < rbs.size(); j++){
            contacts.emplace_back(Contact(rbs[i], rbs[j]));
        }
    }
}

Contact::Contact(Rigidbody* rb1, Rigidbody* rb2) : rb1(rb1), rb2(rb2) {
    l0 = SplitLine();
    l1 = SplitLine();
    l0.dir = Vec2(0,0);
    l1.dir = Vec2(0,0);
    l0.normal = Vec2(0,0);
    l1.normal = Vec2(0,0);
    temp = Vec2(0, 0);

    if (!FindSplitLine()){
        printf("ERROR: Rigidbody starting position not legal...\n Exiting...\n");
        exit(-1);
    }
    cycle();
}

bool Contact::step(){
    //if a split line was found, it uses edge of rb1
    // we check if any point of rb2 has crossed
    // printf("===================================\n");
    l0.update();
    for (Vec2 & p : rb2->shape.world_space_points){
        vecsub(p, *l0.p, temp);
        // printf("p: %.3f %.3f l0.p: %.3f %.3f\n", p[0], p[1], (*l0.p)[0], (*l0.p)[1]);
        // printf("dot(p-l.p, l.n): %.3f\n", dot(temp, l0.normal));
        if (dot(temp, l0.normal) < 0){
            bool res = FindSplitLine();
            if (res) cycle();
            return res;
        }
    }

    return true;
}

void Contact::cycle(){
    l0.p = l1.p;
    l0.q = l1.q;
    vecassign(l0.normal, l1.normal);
    vecassign(l0.dir, l1.dir);
}

void SplitLine::update(){
    dir[0] = (*q)[0] - (*p)[0];
    dir[1] = (*q)[1] - (*p)[1];

    normal[0] = -dir[1];
    normal[1] =  dir[0];
    normal.normalise();
}

bool Contact::FindSplitLine(Rigidbody* A, Rigidbody* B){
    int n = A->shape.world_space_points.size();
    int m = B->shape.world_space_points.size();
    for (int i = 0; i < n; i++){
        // printf("Beggin loop \n");
        Vec2* a = &A->shape.world_space_points[i];
        Vec2* b = &A->shape.world_space_points[(i+1)%n];
        l1.p = a;
        l1.q = b;
        //vecsub(*b, *a, temp);
        // printf("i: %i a: %.3f %.3f b: %.3f %.3f b-a %.3f %.3f\n", i, (*a)[0], (*a)[1], (*b)[0], (*b)[1], temp[0], temp[1]);
        l1.update();
        // printf("na update\n");

        int j = 0;
        for(Vec2 & p : rb2->shape.world_space_points){
            vecsub(p, *l1.p, temp);
            // printf("vecsub in loop\n");
            if (dot(temp, l1.normal) < 0){
                // can't be splitline
                // printf("Kannie deze zijn\n");
                break;
            }
            j++;
        }
        if (j == m){
            // split line found
            // printf("Lijn gevonden\n");
            return true;
        } 
    }
    return false;
}

// assumption, i -> i+1 always turns right (cw)
bool Contact::FindSplitLine(){
    if (FindSplitLine(rb1, rb2)){
        return true;
    }else{
        // printf("Tweede\n");
        swapper = rb1;
        rb1 = rb2;
        rb2 = swapper;
        return FindSplitLine(rb1, rb2);
    }
}

bool is_intersect(AABB &X, AABB &Y){
    return !(X.left > Y.right || Y.left > X.right || X.top > Y.bottom || Y.top > X.bottom);
}

bool CollisionHelper::is_legal_position(){
    for (Contact &c : contacts){
        if (!c.step()){
            last_result_is_legal_position = false;
            // Dit moet uiteindelijk wel gebeuren, nu nog niet want testen
            // return false;
        } 
    }
    last_result_is_legal_position = true;
    for (Contact &c : contacts){
        c.cycle();
        c.l0.update();
    }
    return true;
}

void CollisionHelper::draw(){
    if (!last_result_is_legal_position) return;
    glBegin(GL_LINES);
    glColor3f(1.0, 0, 0);
    for (Contact &c : contacts){
        glColor3f(1.0, 0, 0);
        //printf("%.3f, %.3f\n", (*c.l1.p)[0] + 1*c.l1.dir[0], (*c.l1.p)[1] + 1*c.l1.dir[1]);
        glVertex2f((*c.l0.p)[0] + 30*c.l0.dir[0], (*c.l0.p)[1] + 30*c.l0.dir[1]);
        glVertex2f((*c.l0.p)[0] - 20*c.l0.dir[0], (*c.l0.p)[1] - 20*c.l0.dir[1]);

        glColor3f(0, 0, 1.0);
        glVertex2f((*c.l0.p)[0], (*c.l0.p)[1]);
        glVertex2f((*c.l0.p)[0] + 0.1*c.l0.normal[0], (*c.l0.p)[1] + 0.1*c.l0.normal[1]);
        
    }
    glColor3f(0, 1.0, 0);
    glLineWidth(0.5f);
    for (Contact &c : contacts){
        glVertex2f((*c.l0.p)[0] + 0.5*c.l0.dir[0], (*c.l0.p)[1] + 0.5*c.l0.dir[1]);
        glVertex2f(c.rb2->shape.centroid[0], c.rb2->shape.centroid[1]);
    }
    glEnd();
}