#include <algorithm>
#include <limits>
#include "polygon.h"
#include "line.h"

Point& Polygon::operator[](const int &i) {
    return v[i];
}

Point& Polygon::at(const int &i) {
    const int s = v.size();
    return v[i < 0 ? i % s + s : i % s];
}

Point& Polygon::first() {
    return v.front();
}

Point& Polygon::last() {
    return v.back();
}

int Polygon::size() const {
    return v.size();
}

void Polygon::push(const Point &p) {
    v.push_back(p);
}

void Polygon::clear() {
    v.clear();
    tp.clear();
    tl.clear();
}

void Polygon::makeCCW() {
    int br = 0;

    // find bottom right point
    for (int i = 1; i < v.size(); ++i) {
        if (v[i].y < v[br].y || (v[i].y == v[br].y && v[i].x > v[br].x)) {
            br = i;
        }
    }

    // reverse poly if clockwise
    if (!left(at(br - 1), at(br), at(br + 1))) {
        reverse();
    }
}

void Polygon::reverse() {
    std::reverse(v.begin(), v.end());
}

bool Polygon::isReflex(const int &i) {
    return right(at(i - 1), at(i), at(i + 1));
}

bool Polygon::canSee(const int &a, const int &b) {
    Point p;
    Scalar dist;

    if (leftOn(at(a + 1), at(a), at(b)) && rightOn(at(a - 1), at(a), at(b))) {
        return false;
    }
    dist = sqdist(at(a), at(b));
    for (int i = 0; i < v.size(); ++i) { // for each edge
        if ((i + 1) % v.size() == a || i == a) // ignore incident edges
            continue;
        if (leftOn(at(a), at(b), at(i + 1)) && rightOn(at(a), at(b), at(i))) { // if diag intersects an edge
            p = lineInt(Line(at(a), at(b)), Line(at(i), at(i + 1)));
            if (sqdist(at(a), p) < dist) { // if edge is blocking visibility to b
                return false;
            }
        }
    }

    return true;
}

Polygon Polygon::copy(const int &i, const int &j) {
    Polygon p;
    if (i < j) {
        p.v.insert(p.v.begin(), v.begin() + i, v.begin() + j + 1);
    } else {
        p.v.insert(p.v.begin(), v.begin() + i, v.end());
        p.v.insert(p.v.end(), v.begin(), v.begin() + j + 1);
    }
    return p;
}

EdgeList Polygon::decomp() {
    EdgeList min, tmp1, tmp2;
    int nDiags = std::numeric_limits<int>::max();

    for (int i = 0; i < v.size(); ++i) {
        if (isReflex(i)) {
            for (int j = 0; j < v.size(); ++j) {
                if (canSee(i, j)) {
                    tmp1 = copy(i, j).decomp();
                    tmp2 = copy(j, i).decomp();
                    tmp1.insert(tmp1.end(), tmp2.begin(), tmp2.end());
                    if (tmp1.size() < nDiags) {
                        min = tmp1;
                        nDiags = tmp1.size();
                        min.push_back(Edge(at(i), at(j)));
                    }
                }
            }
        }
    }
    
    return min;
}
