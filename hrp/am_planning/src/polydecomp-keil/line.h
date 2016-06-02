#ifndef _LINE_H
#define	_LINE_H

#include <utility>
#include <vector>
#include "point.h"

typedef std::pair<Point, Point> Line;
Point lineInt(const Line &l1, const Line &l2);

#endif	/* _LINE_H */

