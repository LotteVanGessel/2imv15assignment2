#include "RigidbodyCollection.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>

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
