#include "line.h"

Point lineInt(const Line &l1, const Line &l2) {
    Point i;
    Scalar a1, b1, c1, a2, b2, c2, det;
    a1 = l1.second.y - l1.first.y;
    b1 = l1.first.x - l1.second.x;
    c1 = a1 * l1.first.x + b1 * l1.first.y;
    a2 = l2.second.y - l2.first.y;
    b2 = l2.first.x - l2.second.x;
    c2 = a2 * l2.first.x + b2 * l2.first.y;
    det = a1 * b2 - a2*b1;
    if (!eq(det, 0)) { // lines are not parallel
        i.x = (b2 * c1 - b1 * c2) / det;
        i.y = (a1 * c2 - a2 * c1) / det;
    }
    return i;
}