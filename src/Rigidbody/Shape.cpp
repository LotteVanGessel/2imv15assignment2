#include "Shape.h"
#include <GL/glut.h>
void Shape::calc_rel_points(){
    rel_points = std::vector<Vec2>(points.size());
    int i;
    for(i = 0; i < points.size(); i++){
        rel_points[i] = points[i] - centroid;
    } 
}

void Shape::post_ctor(){
    triangulate();

    calculate_centroid();

    calc_rel_points();
}

inline void calc_point(Mat2 &R, Vec2 &off, Vec2 &v, Vec2 &res){
    matmult(R, v, res);
    vecadd(off, res, res);
}

void Shape::update_world_space(Mat2 &rotation, Vec2 &offset){
    if (world_space_points.size() == 0){
        for (int i = 0; i < points.size(); i++) world_space_points.push_back(Vec2());
    }
    for (int i = 0; i < world_space_points.size(); i++){
        calc_point(rotation, offset, rel_points[i], world_space_points[i]);
    }
}

void Shape::draw(Mat2 &rotation, Vec2 &offset, DrawModes::DrawMode mode, float r = 1.0f, float g = 1.0f, float b = 1.0f){
	glLineWidth(1.0f);
	glColor3f(r, g, b);
    // printf("Matrix R:\n\t[%5.3f\t%5.3f]\n\t[%5.3f\t%5.3f]\nVec offset:\n\t[%5.3f\t%5.3f]\n", rotation[0], rotation[1], rotation[2], rotation[3], offset[0], offset[1]);
    
    switch(mode){
        case DrawModes::LINES:
            glBegin(GL_LINE_LOOP);
            for (Vec2 & v : world_space_points){
                glVertex2f(v[0], v[1]);
            }
            glEnd();
            break;
        
        case DrawModes::TRIS:
            glBegin(GL_LINES);
            for (int i = 0; i < triangulation.size(); i+=3){
                Vec2 & a = world_space_points[triangulation[i]];
                Vec2 & b = world_space_points[triangulation[i+1]];
                Vec2 & c = world_space_points[triangulation[i+2]];
                
                glVertex2f(a[0], a[1]);
                glVertex2f(b[0], b[1]);

                glVertex2f(b[0], b[1]);
                glVertex2f(c[0], c[1]);

                glVertex2f(c[0], c[1]);
                glVertex2f(a[0], a[1]);
            }
            glEnd();
            break;
    }
	
}

Rect::Rect(Vec2 &botleft, Vec2 &topright){
    points = std::vector<Vec2>{
                                Vec2( botleft[0],  botleft[1]),
                                Vec2( botleft[0], topright[1]),
                                Vec2(topright[0], topright[1]),
                                Vec2(topright[0],  botleft[1])
                                };
    rel_points = std::vector<Vec2>();
    triangulation = std::vector<int>();
    post_ctor(); 
}

void Rect::calculate_centroid(){
    centroid = (points[0]+points[1]+points[2]+points[3]);
    centroid *= 0.25f;
}

void Rect::triangulate(){
    triangulation = {
        0, 1, 2,
        0, 2, 3
    };
}