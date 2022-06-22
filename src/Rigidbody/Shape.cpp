#include "Shape.h"
#include <GL/glut.h>
#include <cstdio>
#include <cmath>
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

void lowline(std::set<std::pair<Point, Vec2>> &grid_cells, Point &start, Point &end, Vec2& va, Vec2& interpv, Vec2& temp){
    int x = start.x;
    int y = start.y;

    int dx = end.x - start.x;
    int dy = end.y - start.y;
    

    int yinc = dy < 0 ? -1 : 1;
    dy *= yinc;
    int D = 2*dy-dx;

    float invlength = 1/sqrt(dx*dx + dy*dy);
    for (; x != end.x; x += 1){
        // interpolate between va and vb
        float t = sqrt((x-start.x)*(x-start.x)+(y-start.y)*(y-start.y))*invlength;
        scalarmult(t, interpv, temp);
        grid_cells.insert(std::make_pair(Point(x, y), Vec2(va[0]+temp[0], va[1]+temp[1])));
        if (D > 0){
            y += yinc;
            D += 2 * (dy-dx);
        }else{
            D += 2*dy;
        }
    }
}

void highline(std::set<std::pair<Point, Vec2>> &grid_cells, Point &start, Point &end, Vec2& va, Vec2& interpv, Vec2& temp){
    int x = start.x;
    int y = start.y;

    int dx = end.x - start.x;
    int dy = end.y - start.y;
    

    int xinc = dx < 0 ? -1 : 1;
    dx *= xinc;
    int D = 2*dx-dy;

    float invlength = 1/sqrt(dx*dx + dy*dy);
    for (; y != end.y; y += 1){
        // interpolate between va and vb
        float t = sqrt((x-start.x)*(x-start.x)+(y-start.y)*(y-start.y))*invlength;
        scalarmult(t, interpv, temp);
        grid_cells.insert(std::make_pair(Point(x, y), Vec2(va[0]+temp[0], va[1]+temp[1])));
        if (D > 0){
            x += xinc;
            D += 2 * (dx-dy);
        }else{
            D += 2*dx;
        }
    }
}



std::set<std::pair<Point, Vec2>> Shape::get_grid_cells(int N){
    std::set<std::pair<Point, Vec2>> grid_cells;
    float h = 1.0/N;
    float hh = 0.5*h;
    int n = world_space_points.size();
    Vec2 interpv = Vec2(0, 0);
    for (int i = 0; i < n; i++){
        Vec2& a = world_space_points[i];
        Vec2& b = world_space_points[(i+1)%n];
        Vec2& va = world_space_velocity[i];
        Vec2& vb = world_space_velocity[(i+1)%n];
        vecsub(b, a, temp);
        vecsub(vb, va, interpv);
        Point start((int)(a[0]*N)+1, (int)(a[1]*N)+1);
        Point end((int)(b[0]*N)+1, (int)(b[1]*N)+1);

        int dx = end.x - start.x;
        int dy = end.y - start.y;
        int xinc = dx < 0 ? -1 : 1;
        int yinc = dy < 0 ? -1 : 1;

        if (dx == 0){
            float invlength = 1/sqrt(dx*dx + dy*dy);
            int x = start.x;
            for (int y = start.y; y != end.y; y+=yinc){
                float t = sqrt((x-start.x)*(x-start.x)+(y-start.y)*(y-start.y))*invlength;
                scalarmult(t, interpv, temp);
                grid_cells.insert(std::make_pair(Point(x, y), Vec2(va[0]+temp[0], va[1]+temp[1])));
            }
        }else if (dy == 0){
            float invlength = 1/sqrt(dx*dx + dy*dy);
            int y = start.y;
            for (int x = start.x; x != end.x; x+=xinc){
                float t = sqrt((x-start.x)*(x-start.x)+(y-start.y)*(y-start.y))*invlength;
                scalarmult(t, interpv, temp);
                grid_cells.insert(std::make_pair(Point(x, y), Vec2(va[0]+temp[0], va[1]+temp[1])));
            }
        }else if (dx * xinc > dy*yinc){
            if (start.x > end.x){
                scalarmult(-1, interpv, interpv);
                lowline(grid_cells, end, start, vb, interpv, temp);
            }else{
                lowline(grid_cells, start, end, va, interpv, temp);
            }
        }else{
            if (start.y > end.y){
                scalarmult(-1, interpv, interpv);
                highline(grid_cells, end, start, vb, interpv, temp);
            }else{
                highline(grid_cells, start, end, va, interpv, temp);
            }
        }
    }
    
    free(interpv.data);
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