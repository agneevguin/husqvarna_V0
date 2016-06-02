#ifndef _POLYGON_H
#define	_POLYGON_H

#include <vector>
#include "config.h"
#include "point.h"
#include "line.h"
#include "edge.h"

class Polygon {
public:
    Point & operator[](const int &i);
    Point& at(const int &i);
    Point& first();
    Point& last();
    int size() const;
    EdgeList decomp();
    bool isReflex(const int &i);
    bool canSee(const int &a, const int &b);
    void push(const Point &p);
    void clear();
    void makeCCW();
    void reverse();
    void render() const;
    Polygon copy(const int &i, const int &j);

    vector<Point> tp;
    vector<Line> tl;
private:
    vector<Point> v;
};

#endif	/* _POLYGON_H */

