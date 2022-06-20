#include "MatrixMath.h"
#include <vector>
#include <cstring>

class Shape{
    public:
        std::vector<Vec2> points;
        Vec2 centroid;
        
        std::vector<Vec2> rel_points;

        std::vector<int> triangulation; // 3*n vector of triangle corners. Used for inertia calculation

        void calc_rel_points();
        virtual void triangulate(){};
        virtual void calculate_centroid(){};
        void draw(Vec2 &offset, float r, float g, float b);

        void post_ctor();

        Shape();
};

class Rect : Shape{
    public:
        Rect(Vec2 botleft, Vec2 topright){
            points = std::vector<Vec2>{
                                       Vec2( botleft[0],  botleft[1]),
                                       Vec2(topright[0],  botleft[1]),
                                       Vec2(topright[0], topright[1]),
                                       Vec2( botleft[0], topright[1])
                                      };
            post_ctor();
        }

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

    // Computed quantities
    Vec2 F;   // Force(t)
    Vec2 tau; // Tau(t) -> torque

    Rigidbody(Shape shape);
    ~Rigidbody(){
        free(state);
    }
    void dxdt(float* y);
    void update_state(float* new_state){ std::memcpy(state, new_state, (x.n + R.n + P.n + L.n)*sizeof(float));}
};

class RigidbodyCollection{
    std::vector<Rigidbody> rbs;
    float* x0;
    float* x1;
    float* temp;
    float* Dxdt;

    int n;

    public:
        RigidbodyCollection(){
            rbs = std::vector<Rigidbody>();
        }

        ~RigidbodyCollection(){
            free(x0);
            free(x1);
            free(temp);
            free(Dxdt);
        }
        void init(){
            n = Rigidbody::STATE_SIZE * rbs.size();
            x0   = (float*) malloc(n* sizeof(float));
            x1   = (float*) malloc(n* sizeof(float));
            temp = (float*) malloc(n* sizeof(float));
            Dxdt = (float*) malloc(n* sizeof(float));
            copy_states(x1);
        }

        void copy_states(float* dst){ for (int i = 0; i < rbs.size(); ++i) std::memcpy(dst+(i*Rigidbody::STATE_SIZE), rbs[i].state, Rigidbody::STATE_SIZE*sizeof(float));}

        void step(float dt);

        void computeforceandtorque();
};
