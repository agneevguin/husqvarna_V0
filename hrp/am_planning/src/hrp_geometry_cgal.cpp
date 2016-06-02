#include <am_planning/hrp_geometry_helpers.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <cassert>
#include <list>

#include <vector>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <string>

#include <boost/shared_ptr.hpp>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>

namespace Husqvarna
{

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K> Traits;
typedef CGAL::Is_convex_2<Traits> Is_convex_2;
typedef Traits::Polygon_2 Polygon_2;
typedef Traits::Point_2 Point_2;
typedef Polygon_2::Vertex_const_iterator Vertex_iterator;
typedef std::list<Polygon_2> Polygon_list;
typedef CGAL::Partition_is_valid_traits_2<Traits, Is_convex_2> Validity_traits;
typedef CGAL::Creator_uniform_2<int, Point_2> Creator;
typedef CGAL::Random_points_in_square_2<Point_2, Creator> Point_generator;
typedef std::list<Polygon_2>::const_iterator Polygon_iterator;

// Offset polygon and skeleton stuff----------

typedef K::FT FT;
typedef CGAL::Straight_skeleton_2<K> Ss;

typedef boost::shared_ptr<Polygon_2> PolygonPtr;
typedef boost::shared_ptr<Ss> SsPtr;

typedef std::vector<PolygonPtr> PolygonPtrVector;

//---------------
void cgalPolyFromRos(geometry_msgs::Polygon& input, Polygon_2& output)
{
    for (int i = 0; i < input.points.size(); i++)
    {
        output.push_back(Point_2(input.points[i].x, input.points[i].y));
    }

    // Make sure we have a COUNTERCLOCKWISE polygon
    if (output.orientation() == CGAL::CLOCKWISE)
    {
        output.clear();
        for (int i = input.points.size() - 1; i >= 0; i--)
        {
            output.push_back(Point_2(input.points[i].x, input.points[i].y));
        }
    }
}

void rosPolyFromCgal(Polygon_2 input, geometry_msgs::Polygon& output)
{
    // Iterate through the points for a polygon
    for (Vertex_iterator vi = input.vertices_begin(); vi != input.vertices_end(); ++vi)
    {
        Point_2 v = *vi;
        geometry_msgs::Point32 pt;
        pt.x = v.x();
        pt.y = v.y();

        output.points.push_back(pt);
    }
}

void shrinkPolygonCgal(double shrinkFactor, geometry_msgs::Polygon& input, geometry_msgs::Polygon& output)
{
    Polygon_list cgalOutputPolys;
    Traits partitionTraits;
    Validity_traits validityTraits;

    // Convert from ROS-polygon to CGAL polygon
    Polygon_2 cgalInput;
    cgalPolyFromRos(input, cgalInput);

    FT lOffset = 1;

    PolygonPtrVector inner_offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset, cgalInput);
    PolygonPtrVector outer_offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(lOffset, cgalInput);

    rosPolyFromCgal(*inner_offset_polygons[0],output);
}

void decomposePolygonCgal(geometry_msgs::Polygon& input, std::vector<geometry_msgs::Polygon>& output)
{
    Polygon_list cgalOutputPolys;
    Traits partitionTraits;
    Validity_traits validityTraits;

    // Convert from ROS-polygon to CGAL polygon
    Polygon_2 cgalInput;

    for (int i = 0; i < input.points.size(); i++)
    {
        cgalInput.push_back(Point_2(input.points[i].x, input.points[i].y));
    }

    // Make sure we have a COUNTERCLOCKWISE polygon
    if (cgalInput.orientation() == CGAL::CLOCKWISE)
    {
        cgalInput.clear();
        for (int i = input.points.size() - 1; i >= 0; i--)
        {
            cgalInput.push_back(Point_2(input.points[i].x, input.points[i].y));
        }
    }

    // Do the decomposition using CGAL
    CGAL::optimal_convex_partition_2(
        cgalInput.vertices_begin(), cgalInput.vertices_end(), std::back_inserter(cgalOutputPolys), partitionTraits);

    std::cout << "CHECK output data!" << std::endl;
    assert(CGAL::partition_is_valid_2(cgalInput.vertices_begin(),
                                      cgalInput.vertices_end(),
                                      cgalOutputPolys.begin(),
                                      cgalOutputPolys.end(),
                                      validityTraits));

    // Now walk through the result and convert to ROS
    for (Polygon_iterator poly_it = cgalOutputPolys.begin(); poly_it != cgalOutputPolys.end(); poly_it++)
    {
        Polygon_2 p = *poly_it;

        geometry_msgs::Polygon outputPoly;

        // Iterate through the points for a polygon
        for (Vertex_iterator vi = p.vertices_begin(); vi != p.vertices_end(); ++vi)
        {
            Point_2 v = *vi;
            geometry_msgs::Point32 pt;
            pt.x = v.x();
            pt.y = v.y();

            outputPoly.points.push_back(pt);
        }

        output.push_back(outputPoly);
    }
}
}