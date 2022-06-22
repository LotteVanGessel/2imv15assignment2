#include "EdgeSet.h"

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