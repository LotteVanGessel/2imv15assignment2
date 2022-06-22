#include "Shape.h"
#include <GL/glut.h>
#include <cstdio>
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
        for (int i = 0; i < points.size(); i++) world_space_velocity.push_back(Vec2());
    }
    for (int i = 0; i < world_space_points.size(); i++){
        vecassign(temp, world_space_points[i]);
        calc_point(rotation, offset, rel_points[i], world_space_points[i]);
        vecsub(world_space_points[i], temp, temp);
        vecassign(world_space_velocity[i], temp);
        // printf("wsv: %.5f %.5f\n", world_space_velocity[i][0], world_space_velocity[i][1]);
    }
}

void Shape::draw(Mat2 &rotation, Vec2 &offset, DrawModes::DrawMode mode, float r = 1.0f, float g = 1.0f, float b = 1.0f){
	glLineWidth(5.0f);
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


std::set<std::pair<Point, Vec2>> Shape::get_grid_cells(){
    std::set<std::pair<Point, Vec2>> grid_cells;
    int N = 64;
    float h = 1.0/N;
    float hh = 0.5*h;
    int n = world_space_points.size();
    for (int i = 0; i < n; i++){
        Vec2& a = world_space_points[i];
        Vec2& b = world_space_points[(i+1)%n];
        vecsub(b, a, temp);
        
        Point start((int)(a[0]*N)+1, (int)(a[1]*N)+1);
        Point end((int)(b[0]*N)+1, (int)(b[1]*N)+1);

        int dirx = end.x-start.x > 0 ? 1 : -1;
        int diry = end.y-start.y > 0 ? 1 : -1;
        if (end.x-start.x == 0 || end.y-start.y == 0) return grid_cells;
        float dydx = temp[1]/temp[0];
        float y = a[1];
        float dx = h;
        float div = 1.0/(end.x-start.x);
        Vec2& va = world_space_velocity[i];
        Vec2& vb = world_space_velocity[i];
        Vec2 interpa = Vec2(0, 0);
        Vec2 interpb = Vec2(0, 0);
        for (int x = start.x; x != end.x && dirx != 0 && std::abs(div) > 0.00001; x += dirx){
            scalarmult(div*(x-start.x)*dirx, va, interpa);
            scalarmult(1-div*(x-start.x)*dirx, vb, interpb);
            // calculate y
            grid_cells.insert(std::make_pair(Point(x, (int)(y*N)), 
                                             Vec2(interpa[0] + interpb[0], interpa[1] + interpb[1])));
            y += dydx * dx;
        }

        vecsub(b, a, temp);
        float dxdy = temp[0]/temp[1];
        float x = a[0];
        float dy = h;
        div = 1.0/(end.y-start.y);
        vecsub(vb, va, temp);
        for (int y = start.y; y != end.y && diry != 0 && std::abs(div) > 0.00001; y += diry){
            scalarmult(div*(y-start.y)*diry, va, interpa);
            scalarmult(1-div*(y-start.y)*diry, vb, interpb);
            grid_cells.insert(std::make_pair(Point((int)(x*N),y),
                                             Vec2(interpa[0] + interpb[0], interpa[1] + interpb[1])));
            x += dxdy * dy;
        }
    }
    return grid_cells;
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