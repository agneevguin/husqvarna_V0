
#include <am_planning/hrp_geometry_helpers.h>
#include <boost/foreach.hpp>
#include "polydecomp-keil/polygon.h"

namespace Husqvarna
{
////////////////////////////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////////////////////////////
geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt)
{
    geometry_msgs::Point32 point32;
    point32.x = pt.x;
    point32.y = pt.y;
    point32.z = pt.z;
    return point32;
}

geometry_msgs::Point toPoint(geometry_msgs::Point32 pt)
{
    geometry_msgs::Point point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    return point;
}

typedef struct 
{
    Edge e;
    bool usedLeft;
    bool usedRight;
    bool isShared;
    bool isDeleted;
    
}MultiPolyEdge;

typedef std::vector<MultiPolyEdge> MultiPolyEdgeList;
////////////////////////////////////////////////////////////////////////
// Calculate length to a LINE (from a point) helpers...
// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
////////////////////////////////////////////////////////////////////////
double sqr(double x)
{
    return x * x;
}

double dist2(geometry_msgs::Point32 v, geometry_msgs::Point32 w)
{
    return sqr(v.x - w.x) + sqr(v.y - w.y);
}

// Return minimum distance (squared) between line segment vw and point p
double distToSegmentSquared(geometry_msgs::Point32 p, geometry_msgs::Point32 v, geometry_msgs::Point32 w)
{
    double l2 = dist2(v, w);

    if (l2 == 0)
    {
        return dist2(p, v);
    }

    double t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

    if (t < 0)
    {
        return dist2(p, v);
    }

    if (t > 1)
    {
        return dist2(p, w);
    }

    geometry_msgs::Point32 vproj;

    vproj.x = v.x + t * (w.x - v.x);
    vproj.y = v.y + t * (w.y - v.y);

    return dist2(p, vproj);
}

double distToSegment(geometry_msgs::Point32 p, geometry_msgs::Point32 v, geometry_msgs::Point32 w)
{
    return sqrt(distToSegmentSquared(p, v, w));
}

////////////////////////////////////////////////////////////////////////
// Fill helpers
////////////////////////////////////////////////////////////////////////



void findMinMaxPoly(MinMaxPoints& res, geometry_msgs::Polygon& poly)
{
    res.minY = 1000.0;
    res.minX = 1000.0;
    res.maxY = -1000.0;
    res.maxX = -1000.0;

    BOOST_FOREACH(geometry_msgs::Point32 point, poly.points)
    {
        if (point.x < res.minX)
        {
            res.minX = point.x;
        }
        if (point.x > res.maxX)
        {
            res.maxX = point.x;
        }

        if (point.y < res.minY)
        {
            res.minY = point.y;
        }
        if (point.y > res.maxY)
        {
            res.maxY = point.y;
        }
    }
}

// ROUNDING ERROR THAT CAUSES LINES TO NOT INTERSECT
#define ERR (0.001)

bool lineIntersection(Line& l1, Line& l2, geometry_msgs::Point32& res)
{
    // Store the values for fast access and easy
    // equations-to-code conversion
    double x1 = l1.p1.x;
    double x2 = l1.p2.x;
    double x3 = l2.p1.x;
    double x4 = l2.p2.x;

    double y1 = l1.p1.y;
    double y2 = l1.p2.y;
    double y3 = l2.p1.y;
    double y4 = l2.p2.y;

    double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // If d is zero, there is no intersection
    if (d == 0)
    {
        return false;
    }

    // Get the x and y
    double pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
    double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
    double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

    // Check if the x and y coordinates are within both lines
    if (x < (fmin(x1, x2) - ERR) || x > (fmax(x1, x2) + ERR) || x < (fmin(x3, x4) - ERR) || x > (fmax(x3, x4) + ERR))
    {
        return false;
    }
    if (y < (fmin(y1, y2) - ERR) || y > (fmax(y1, y2) + ERR) || y < (fmin(y3, y4) - ERR) || y > (fmax(y3, y4) + ERR))
    {
        return false;
    }

    // Return the point of intersection
    res.x = x;
    res.y = y;
    return true;
}

bool getLineFromPoly(Line& scanLine, geometry_msgs::Polygon& poly, Line& line)
{
    geometry_msgs::Point32 pint;

    std::vector<geometry_msgs::Point32> foundPts;

    // std::cout << "Scan-line: (" << scanLine.p1.x << "," << scanLine.p1.y << ") <-> ("  << scanLine.p2.x << "," <<
    // scanLine.p2.y << ")" << std::endl;

    // We need to test with all lines in poly...
    for (int i = 0; i < poly.points.size() - 1; i++)
    {
        Line tstLine;
        tstLine.p1.x = poly.points[i].x;
        tstLine.p1.y = poly.points[i].y;

        tstLine.p2.x = poly.points[i + 1].x;
        tstLine.p2.y = poly.points[i + 1].y;

        // std::cout << "Line: (" << tstLine.p1.x << "," << tstLine.p1.y << ") <-> ("  << tstLine.p2.x << "," <<
        // tstLine.p2.y << ")" << std::endl;

        if (lineIntersection(scanLine, tstLine, pint))
        {
            // std::cout << "--> Intersection: (" << pint.x << "," << pint.y << ")" << std::endl;
            foundPts.push_back(pint);
        }
    }

    // Also test with last line
    Line tstLine;
    int i = poly.points.size() - 1;

    tstLine.p1.x = poly.points[i].x;
    tstLine.p1.y = poly.points[i].y;

    tstLine.p2.x = poly.points[0].x;
    tstLine.p2.y = poly.points[0].y;

    // std::cout << "Line: (" << tstLine.p1.x << "," << tstLine.p1.y << ") <-> ("  << tstLine.p2.x << "," <<
    // tstLine.p2.y << ")" << std::endl;

    if (lineIntersection(scanLine, tstLine, pint))
    {
        // std::cout << "--> Intersection: (" << pint.x << "," << pint.y << ")" << std::endl;
        foundPts.push_back(pint);
    }

    double maxX = -1000.0;
    double minX = 1000.0;
    BOOST_FOREACH(geometry_msgs::Point32 point, foundPts)
    {
        // std::cout << "--> Intersection: (" << point.x << "," << point.y << ")" << std::endl;
        if (point.x > maxX)
        {
            maxX = point.x;
        }
        if (point.x < minX)
        {
            minX = point.x;
        }
    }

    if (foundPts.size() <= 1)
    {
        return false;
    }

    if (maxX != minX)
    {
        line.p1.x = minX;
        line.p1.y = scanLine.p1.y;

        line.p2.x = maxX;
        line.p2.y = scanLine.p1.y;
        return true;
    }
    else
    {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////
// ConvexHull calculation
////////////////////////////////////////////////////////////////////////

bool pointEqual(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
    return ((p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z));
}

int findLeftMostPointIndex(geometry_msgs::Polygon& poly)
{
    double minX = 1000.0;

    int index = 0;
    int foundIndex = -1;
    BOOST_FOREACH(geometry_msgs::Point32 point, poly.points)
    {
        if (point.x < minX)
        {
            minX = point.x;
            foundIndex = index;
        }
        index++;
    }

    return foundIndex;
}

int getOrientation(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p)
{
    // Determinant
    int Orin = (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);

    if (Orin > 0)
    {
        // (* Orientaion is to the left-hand side  *)
        return -1;
    }

    if (Orin < 0)
    {
        // (* Orientaion is to the right-hand side *)
        return 1;
    }

    //  (* Orientaion is neutral aka collinear  *)
    return 0;
}

void getConvexHull(geometry_msgs::Polygon& input, geometry_msgs::Polygon& output)
{
    // get leftmost point
    int leftMostIndex = findLeftMostPointIndex(input);
    geometry_msgs::Point32 vPointOnHull = input.points[leftMostIndex];

    geometry_msgs::Point32 vEndpoint;
    do
    {
        output.points.push_back(vPointOnHull);

        vEndpoint = input.points[0];

        for (int i = 1; i < input.points.size(); i++)
        {
            if (pointEqual(vPointOnHull, vEndpoint) ||
                (getOrientation(vPointOnHull, vEndpoint, input.points[i]) == -1))
            {
                vEndpoint = input.points[i];
            }
        }

        vPointOnHull = vEndpoint;

    } while (!pointEqual(vEndpoint, output.points[0]));
}

void shrinkPolygon(double shrinkFactor, geometry_msgs::Polygon& input, geometry_msgs::Polygon& output)
{
    // Calculate average point of polygon
    double avgX = 0;
    double avgY = 0;

    BOOST_FOREACH(geometry_msgs::Point32 point, input.points)
    {
        avgX += point.x;
        avgY += point.y;
    }

    avgX /= input.points.size();
    avgY /= input.points.size();

    std::cout << "AVG point in polygon: (" << avgX << ", " << avgY << ") " << std::endl;

    // Copy & shrink
    BOOST_FOREACH(geometry_msgs::Point32 point, input.points)
    {
        // Find vector towards mid-point
        geometry_msgs::Point32 towardsMid;
        towardsMid.x = avgX - point.x;
        towardsMid.y = avgY - point.y;
        towardsMid.z = point.z;

        // Scale this one...
        towardsMid.x *= (1.0 - shrinkFactor);
        towardsMid.y *= (1.0 - shrinkFactor);
        towardsMid.z *= (1.0 - shrinkFactor);

        // move point a little
        point.x += towardsMid.x;
        point.y += towardsMid.y;
        point.z += towardsMid.z;

        // Add to list
        output.points.push_back(point);
    }
}

bool isSimple(geometry_msgs::Polygon& poly)
{
    // Loop thorough all edges and check if any intersects
    Line l1,l2;
    geometry_msgs::Point32 isectP;
    int len = poly.points.size();
    std::cout << "isSimple?: len:" <<  len << std::endl;
    for (int i = 0; i < len-2; i++)
    {
        
        l1.p1 = poly.points[i];
        l1.p2 = poly.points[i+1];
        for (int j = i+2; j < len-1; j++)
        {
            l2.p1 = poly.points[j];
            l2.p2 = poly.points[j+1];
            if (lineIntersection(l1,l2, isectP) == true)
            {
                std::cout << "isSimple?: No, intersect between: "<<  i << " and " <<  j << std::endl;
                return false;
            }
        }
        
        if (i > 0)
        {
            // The wrap line:
            l2.p1 = poly.points[len-1];
            l2.p2 = poly.points[0];
            if (lineIntersection(l1,l2, isectP) == true)
            {
                std::cout << "isSimple?: No, intersect (wrap)" << std::endl;
                return false;
            }
        }
        
    }
    std::cout << "isSimple?: Yes" << std::endl;
    return true;
}

// For converting complex to simple
double findPolySquaredEdgeLength(geometry_msgs::Polygon& poly)
{
    double dist2, dx, dy;
    int len;
    dist2 = 0;
    
    len = poly.points.size();
    
    for (int i = 0; i < len-1; i++)
    {
        dx = poly.points[i].x - poly.points[i+1].x;
        dy = poly.points[i].y - poly.points[i+1].y;
        dist2 += dx*dx + dy*dy;
    }
   
    dx = poly.points[len-1].x - poly.points[0].x;
    dy = poly.points[len-1].y - poly.points[0].y;
    dist2 += dx*dx + dy*dy;
   
    return dist2;
}

// Swap two random edges!
void reverseSegment(geometry_msgs::Polygon& input, geometry_msgs::Polygon& output, int i, int j)
{
    //int i,j;
    int len;
    len = input.points.size();
    
    // Select two random points in the polygon

    // Assign a copy first
    output = input;
    
    //Input check!
    if (i >= len)
    {
        i = len-1;
    }
    if (j >= len)
    {
        j = len-1;
    }
    
    // Let i be the smaller one...
    if (j == i)
    {
        return;
    }
    else if (j < i)
    {
        int tmp = i;
        i = j;
        j = tmp;
    }
    


 
    // take the points from the input in reversed order between i and j
    for (int k = 0; k < (j - i); k++)
    {
        output.points[k+i] = input.points[j-k];
    }
    
    // Final outside of loop... 
    output.points[j] = input.points[i];
}

bool isEqual(geometry_msgs::Point32& p1, Point p2)
{
    double dx = fabs(p1.x-p2.x);
    double dy = fabs(p1.y-p2.y);
    if (dx < 0.001 && dy < 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

Point toKielPoint(geometry_msgs::Point32 pIn)
{
    Point p;
    p.x = pIn.x;
    p.y = pIn.y;
    return p;
}
// take a Polygon and create a list of edges, including new ones for decomposition
void makeKielEdgeList(MultiPolyEdgeList &edgeList, geometry_msgs::Polygon& input)
{
    // Format the polygon for Kiel processing
    Polygon keilPoly;
    int len = input.points.size();
    for (int i = 0; i < len; i++)
    {
        keilPoly.push(Point(input.points[i].x, input.points[i].y));
    }
    // Find the edges to be added!
    EdgeList eListKiel = keilPoly.decomp();
    
    MultiPolyEdgeList eList;
    MultiPolyEdge mpe;

    
    //Add all edges from the polygon, and the new ones to the list of edges
    //std::vector<geometry_msgs::Point32>::iterator pIter = input.points.begin();
    for (int i = 0; i < input.points.size(); i++)
    {
        mpe.e.first = toKielPoint(input.points[i]);
        mpe.e.second = toKielPoint(input.points[(i+1) % input.points.size()]);
        mpe.usedLeft = false;
        mpe.usedRight = false;
        mpe.isShared = false;
        mpe.isDeleted = false;
        eList.push_back(mpe);
    }
}

void decomposePolygonKeil(geometry_msgs::Polygon& input, std::vector<geometry_msgs::Polygon>& output)
{
    MultiPolyEdgeList eList;
    output.push_back(input);
}

}