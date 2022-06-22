#ifndef EDGESET_H
#define EDGESET_H
#include <utility>
#include "MatrixMath.h"

struct Point{
	int x, y;
    Point(int x, int y);
};

struct Edge {
	Point a, b;
    Edge(Point a, Point b);
};

bool operator==(const Point & p1, const Point & p2);

bool operator<(const Point & p1, const Point & p2);

bool operator<(const Edge & e1, const Edge & e2);

bool operator<(const std::pair<Point, Vec2> & p1, const std::pair<Point, Vec2> & p2);

#endif