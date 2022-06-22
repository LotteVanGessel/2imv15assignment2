#ifndef EDGESET_H
#define EDGESET_H

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

#endif