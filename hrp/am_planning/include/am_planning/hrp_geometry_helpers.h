#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <vector>

#ifndef HRP_GEOMETRY_HELPERS_H
#define HRP_GEOMETRY_HELPERS_H

namespace Husqvarna
{
    
typedef struct
{
    double minY;
    double maxY;
    double minX;
    double maxX;
} MinMaxPoints;

typedef struct
{
    geometry_msgs::Point32 p1;
    geometry_msgs::Point32 p2;
} Line;

typedef std::vector<Line> LineList;

geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt);
geometry_msgs::Point toPoint(geometry_msgs::Point32 pt);
double sqr(double x);
double dist2(geometry_msgs::Point32 v, geometry_msgs::Point32 w);
double distToSegmentSquared(geometry_msgs::Point32 p, geometry_msgs::Point32 v, geometry_msgs::Point32 w);
double distToSegment(geometry_msgs::Point32 p, geometry_msgs::Point32 v, geometry_msgs::Point32 w);
void findMinMaxPoly(MinMaxPoints& res, geometry_msgs::Polygon& poly);
bool lineIntersection(Line& l1, Line& l2, geometry_msgs::Point32& res);
bool getLineFromPoly(Line& scanLine, geometry_msgs::Polygon& poly, Line& line);
bool pointEqual(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2);
int findLeftMostPointIndex(geometry_msgs::Polygon& poly);
int getOrientation(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p);
void getConvexHull(geometry_msgs::Polygon& input, geometry_msgs::Polygon& output);
void shrinkPolygon(double shrinkFactor, geometry_msgs::Polygon& input, geometry_msgs::Polygon& output);
bool isSimple(geometry_msgs::Polygon& poly);
double findPolySquaredEdgeLength(geometry_msgs::Polygon& poly);
void reverseSegment(geometry_msgs::Polygon& input, geometry_msgs::Polygon& output, int i, int j);
void decomposePolygonCgal(geometry_msgs::Polygon& input, std::vector<geometry_msgs::Polygon>& output);
void decomposePolygonKeil(geometry_msgs::Polygon& input, std::vector<geometry_msgs::Polygon>& output);
void shrinkPolygonCgal(double shrinkFactor, geometry_msgs::Polygon& input, geometry_msgs::Polygon& output);
}
#endif