#ifndef _POINT_H
#define	_POINT_H

#include "config.h"

class Point {
public:
    union {
        struct {
            Scalar x, y;
        };
        Scalar v[2];
    };

    Point();
    Point(Scalar x, Scalar y);
    Scalar& operator[](int i);

    friend Scalar area(const Point &a, const Point &b, const Point &c);
    friend bool left(const Point &a, const Point &b, const Point &c);
    friend bool leftOn(const Point &a, const Point &b, const Point &c);
    friend bool right(const Point &a, const Point &b, const Point &c);
    friend bool rightOn(const Point &a, const Point &b, const Point &c);
    friend bool collinear(const Point &a, const Point &b, const Point &c);
    friend Scalar sqdist(const Point &a, const Point &b);
};

#endif	/* _POLYGON_H */

