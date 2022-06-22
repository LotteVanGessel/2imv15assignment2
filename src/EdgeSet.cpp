#include "EdgeSet.h"
#include <utility>
#include "MatrixMath.h"
Point::Point(int x, int y) : x(x), y(y){};

Edge::Edge(Point a, Point b) : a(a), b(b){};


bool operator==(const Point & p1, const Point & p2){
    return p1.x == p2.x && p1.y == p2.y;
}

bool operator<(const Point & p1, const Point & p2){
    return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
}

bool operator<(const Edge & e1, const Edge & e2){
    return e1.a < e2.a || (e1.a == e2.a && e1.b < e2.b);
}

bool operator<(const std::pair<Point, Vec2> & p1, const std::pair<Point, Vec2> & p2){
    return p1.first < p2.first || (p1.first == p2.first && (p1.second[0] < p2.second[0] || (p1.second[0] == p2.second[0] && p1.second[1] < p2.second[1])));
}