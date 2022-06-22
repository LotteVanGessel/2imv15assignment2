#ifndef SHAPE_H
#define SHAPE_H

#include <vector>
#include <MatrixMath.h>

namespace DrawModes{
    enum DrawMode{LINES, TRIS, HIDE};
    static const int num_draw_modes = 3;
    static const DrawMode modes[num_draw_modes] = {LINES, TRIS, HIDE};
}

class Shape{
    public:

        std::vector<Vec2> points;
        Vec2 centroid;
        Vec2 temp;
        
        std::vector<Vec2> rel_points;
        std::vector<Vec2> world_space_points;

        std::vector<int> triangulation; // 3*n vector of triangle corners. Used for inertia calculation

        void calc_rel_points();
        virtual void triangulate(){};
        virtual void calculate_centroid(){};
        void update_world_space(Mat2 &rotation, Vec2 &offset);
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
#endif