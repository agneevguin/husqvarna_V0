/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include <am_planning/basic_coverage.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <am_planning/geometry_tools.h>
#include <am_planning/hrp_geometry_helpers.h>
#include <am_planning/PathToPose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt16.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <boost/foreach.hpp>

namespace Husqvarna
{

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// BasicCoverage class
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

bool BasicCoverage::getPathFromPoly(am_planning::PathFromPolygon::Request& req,
                                    am_planning::PathFromPolygon::Response& res)
{

    border = req.border;
    calculateFill(req.start.pose, req.border.polygon);
    res.path = coveragePath;
    ROS_INFO("Sending Path back...");
    return true;
}

bool BasicCoverage::getPolygonsFromPoly(am_planning::PolygonsFromPolygon::Request& req,
                                        am_planning::PolygonsFromPolygon::Response& res)
{
    std::vector<geometry_msgs::Polygon> outputPolygons;
    // decomposePolygonKeil(req.inputPolygon.polygon, outputPolygons);
    decomposePolygonCgal(req.inputPolygon.polygon, outputPolygons);

    for (int i = 0; i < outputPolygons.size(); i++)
    {
        geometry_msgs::PolygonStamped tmpPoly;
        tmpPoly.polygon = outputPolygons[i];
        tmpPoly.header.stamp = ros::Time::now();
        tmpPoly.header.frame_id = "odom_combined";
        res.outputPolygons.push_back(tmpPoly);
    }
    ROS_INFO("BasicCoverage::getPolygonsFromPoly... Done");
    return true;
}

bool BasicCoverage::getSimpleFromComplex(am_planning::PolygonsFromPolygon::Request& req,
                                         am_planning::PolygonsFromPolygon::Response& res)
{
    geometry_msgs::PolygonStamped swappedPoly;
    geometry_msgs::PolygonStamped simplePoly = req.inputPolygon;
    int len = simplePoly.polygon.points.size();
    double oldLen, newLen;

    ROS_INFO("BasicCoverage::getSimpleFromComplex Start... polygon length: %d", len);

    if (isSimple(simplePoly.polygon))
    {
        res.outputPolygons.push_back(simplePoly);
        ROS_INFO("BasicCoverage::getSimpleFromComplex Already Simple, return");
        return true;
    }
    else if (len > 0)
    {
        // Basic algorithm: Compute the shortest path joining all point, thus creating a simple polygon!
        // Approach: Simulated annealing, swap the path at two random spots and keep the new with a
        // probability depending on the length of the new path and the cooling factor! (hardcoded to 1 if shorter... )

        for (int i = 0; i < 10000; i++)
        {
            int first = rand() % len;
            int second = rand() % len;
            reverseSegment(simplePoly.polygon, swappedPoly.polygon, first, second);

            // Now swap if better, TODO use probability and cooling scheme...
            oldLen = findPolySquaredEdgeLength(simplePoly.polygon);
            newLen = findPolySquaredEdgeLength(swappedPoly.polygon);

            if (newLen < oldLen)
            {
                //    ROS_INFO("------------------old: %f, new: %f, i: %d, j: %d", oldLen, newLen, first, second);
                simplePoly = swappedPoly;
                oldLen = newLen;
            }
        }

        res.outputPolygons.push_back(simplePoly);
        ROS_INFO("BasicCoverage::getSimpleFromComplex ...Done");
        border_pub.publish(simplePoly);
        return true;
    }
    else
    {
        ROS_ERROR("BasicCoverage::getSimpleFromComplex: Length of poly Zero, not permitted!");
        return false;
    }
}

void BasicCoverage::unitTest(void)
{
    static ros::Publisher border_pub = nh.advertise<geometry_msgs::PolygonStamped>("border_debug", 100);
    static int cnt = 0;
    static bool initRandom = true;
    static int len;
    static bool doneSwap = false;

    double oldLen;
    double newLen;
    geometry_msgs::PolygonStamped startPoly;
    geometry_msgs::PolygonStamped swappedPoly;
    static geometry_msgs::PolygonStamped fixedPoly;

    geometry_msgs::Point32 pTmp;

    if (initRandom)
    {
        // Randomize a polygon as input
        for (int i = 0; i < 100; i++)
        {
            pTmp.x = -10 + rand() % 20;
            pTmp.y = -10 + rand() % 20;
            startPoly.polygon.points.push_back(pTmp);
        }
        fixedPoly = startPoly;
        len = fixedPoly.polygon.points.size();
        initRandom = false;
    }

    if (doneSwap == false)
    {
        for (int i = 0; i < 100000; i++)
        {
            int first = rand() % len;
            int second = rand() % len;
            // first = 1;
            // second = 4;
            reverseSegment(fixedPoly.polygon, swappedPoly.polygon, first, second);

            // Now swap if better, TODO use probability and cooling scheme...
            oldLen = findPolySquaredEdgeLength(fixedPoly.polygon);
            newLen = findPolySquaredEdgeLength(swappedPoly.polygon);
            if (newLen < oldLen)
            // if (doneSwap == false)
            {
                ROS_INFO("------------------old: %f, new: %f, i: %d, j: %d", oldLen, newLen, first, second);
                fixedPoly = swappedPoly;

                oldLen = newLen;
            }
        }
    }
    doneSwap = true;

    // ROS_INFO("Length of polygon: %f", oldLen);
    fixedPoly.header.stamp = ros::Time::now();
    fixedPoly.header.frame_id = "odom_combined";

    border_pub.publish(fixedPoly);
    // ROS_INFO("testing... %d", cnt++);
}

bool BasicCoverage::getPathToGoal(am_planning::PathToPose::Request& req, am_planning::PathToPose::Response& res)
{
    border = req.border;
    goal = req.goal;
    // Current pose...
    calculatePathTo(req.start.pose);
    res.path = coveragePath;
    ROS_INFO("Sending Path back to Goal...");
    //
    //for (int i = 0; i < coveragePath.poses.size(); i++)
    //{
    //   ROS_INFO("(%f, %f)", coveragePath.poses[i].pose.position.x, coveragePath.poses[i].pose.position.y);
    //}
    return true;
}

bool BasicCoverage::getBorderPathFromPoly(am_planning::PathFromPolygon::Request& req,
                                          am_planning::PathFromPolygon::Response& res)
{
    ROS_INFO("BasicCoverage::getBorderPathFromPoly...");
    border = req.border;
    calculateAround();
    res.path = coveragePath;
    ROS_INFO("Sending Border Path back...");
    return true;
}

BasicCoverage::BasicCoverage(const ros::NodeHandle& nodeh)
{
    // Init attributes
    this->nh = nodeh;

    srand(5);
    // Setup some ROS stuff
    joy_sub = nh.subscribe("nano2", 1, &BasicCoverage::joyCallback, this);
    border_pub = nh.advertise<geometry_msgs::PolygonStamped>("border_debug", 100);

    covService = nh.advertiseService("get_basic_coverage", &BasicCoverage::getPathFromPoly, this);
    ROS_INFO("Service /get_basic_coverage registered.");

    borderPathService = nh.advertiseService("get_border_path", &BasicCoverage::getBorderPathFromPoly, this);
    ROS_INFO("Service /get_border_path registered.");

    decomposePolygonService = nh.advertiseService("convex_decompose", &BasicCoverage::getPolygonsFromPoly, this);
    ROS_INFO("Service /convex_decompose registered.");

    complexToSimpleService = nh.advertiseService("complex_to_simple", &BasicCoverage::getSimpleFromComplex, this);
    ROS_INFO("Service /convex_decompose registered.");

    pathToGoalService = nh.advertiseService("get_path_to", &BasicCoverage::getPathToGoal, this);
    ROS_INFO("Service /get_border_path registered.");

    fromToClient = nh.serviceClient<am_planning::PathToPose>("get_astar_path_to");

    // Parameters
    ros::NodeHandle n_private("~");

    waitingForRelease = false;
    joyDisabled = true;

    newPath = false;

    shrinkFactor = 0.50;

    n_private.param<double>("corridorWidth", corridorWidth, 0.20);
    ROS_INFO("Param: corridorWidth: [%f]m", corridorWidth);
}

BasicCoverage::~BasicCoverage()
{
}

void BasicCoverage::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
{
    // Special case to handle key release event...
    if (waitingForRelease)
    {
        waitingForRelease = false;
        return;
    }

    // Need to "enable" this before use of params...
    if (joyDisabled)
    {
        // Pressing R[4]
        if (j->buttons[25 + 16 + 3] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("BasicCoverage::NANO KONTROL enabled!");
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing R[4]
        if (j->buttons[25 + 16 + 3] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("BasicCoverage::NANO KONTROL disabled!");
            return;
        }
    }

    double shrink = j->axes[6] * 1.0;
    if (shrink != 0)
    {
        shrinkFactor = shrink;
        ROS_INFO("shrinkFactor = %f", shrinkFactor);
    }
    else
    {
        shrinkFactor = 0.80;
        ROS_INFO("shrinkFactor = %f", shrinkFactor);
    }

    double cw = j->axes[8 + 6] * 2.0;
    if (cw != 0)
    {
        corridorWidth = cw;
        ROS_INFO("corridorWidth = %f", corridorWidth);
    }
    else
    {
        corridorWidth = 0.20;
        ROS_INFO("corridorWidth = %f", corridorWidth);
    }
}

double BasicCoverage::getCollisionFactor(geometry_msgs::Point32 pos)
{
    double sign = -1.0;
    if (frontier_exploration::pointInPolygon(pos, border.polygon))
    {
        sign = 1.0;
    }

    // Calculate how far we are from the border...
    double minDistance = 9999;

    int l = border.polygon.points.size();

    // All lines but the last one
    for (int i = 0; i < l - 1; i++)
    {
        double d = distToSegment(pos, border.polygon.points[i], border.polygon.points[i + 1]);
        if (d < minDistance)
        {
            minDistance = d;
        }
    }

    // Last one...
    double d = distToSegment(pos, border.polygon.points[0], border.polygon.points[l - 1]);
    if (d < minDistance)
    {
        minDistance = d;
    }

    // Inside/outside?
    minDistance *= sign;

    return minDistance;
}

void BasicCoverage::calculateAround(void)
{
    // Clear path
    coveragePath.poses.clear();

    BOOST_FOREACH(geometry_msgs::Point32 point, border.polygon.points)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = point.z;

        coveragePath.poses.push_back(pose);
    }

    // Add the first point again.
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Point32 point;

    point = border.polygon.points[0];

    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;

    coveragePath.poses.push_back(pose);
}

void BasicCoverage::calculatePathTo(geometry_msgs::Pose& startPose)
{
    // Clear path
    coveragePath.poses.clear();

    //
    // Calculate a point before the goal to get heading right
    //
    geometry_msgs::PoseStamped preDockPose;
    geometry_msgs::PoseStamped dockPose;
    
    // Extract ROLL, PITCH and YAW from pose
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(goal.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double lDist = 0.50;
    double cuttingDiscOffset = 0.25;
    //Account for the center of the knife vs the baselink, se URDF file...
    //TODO: do this using TF!
    lDist += cuttingDiscOffset;
    
    preDockPose.pose.position.x = goal.pose.position.x - cos(M_PI * 2.0 - yaw) * lDist;
    preDockPose.pose.position.y = goal.pose.position.y + sin(M_PI * 2.0 - yaw) * lDist;
    preDockPose.pose.position.z = goal.pose.position.z;
    
    //ROS_INFO("BasicCoverage::PreDockPose: (%f,%f)", preDockPose.pose.position.x, preDockPose.pose.position.y);
    // Get a path to just outside the charging station
    am_planning::PathToPose srv;
    srv.request.border.polygon = border.polygon;
    srv.request.start.pose = startPose;
    srv.request.goal.pose = preDockPose.pose;

    if (fromToClient.call(srv))
    {
        if (srv.response.path.poses.size() >= 2)
        {
            for (int i = 0; i < srv.response.path.poses.size(); i++)
            {
                geometry_msgs::PoseStamped pose = srv.response.path.poses[i];
                coveragePath.poses.push_back(pose); 
            }
        }
        else
        {
            ROS_INFO("Returned path too short to be usable...");
        }
    }

    // Now add the goal
    dockPose.pose.position.x = goal.pose.position.x + cos(M_PI * 2.0 - yaw) * cuttingDiscOffset;
    dockPose.pose.position.y = goal.pose.position.y - sin(M_PI * 2.0 - yaw) * cuttingDiscOffset;
    dockPose.pose.position.z = goal.pose.position.z;
    coveragePath.poses.push_back(dockPose);
    
    
}

void BasicCoverage::calculateFill(geometry_msgs::Pose& startPose, geometry_msgs::Polygon& area)
{
    fillPoints.points.clear();

    geometry_msgs::Polygon oneAreaPoints;

    geometry_msgs::Pose coverageStart;
    geometry_msgs::Pose start;
    geometry_msgs::Pose goal;
    bool allButFirstOne = false;

    // Decompose the border...
    std::vector<geometry_msgs::Polygon> convexPolygons;

    decomposePolygonCgal(area, convexPolygons);

    BOOST_FOREACH(geometry_msgs::Polygon poly, convexPolygons)
    {
        calculateFillOnConvexPolygon(poly, oneAreaPoints);

        // Get the start (i.e. the first point in the path)
        goal.position.x = oneAreaPoints.points[0].x;
        goal.position.y = oneAreaPoints.points[0].y;
        goal.position.z = oneAreaPoints.points[0].z;

        if (allButFirstOne)
        {
            // Now, try a service call to get a path
            am_planning::PathToPose srv;
            srv.request.border.polygon = area;

            srv.request.start.pose = start;
            srv.request.goal.pose = goal;

            if (fromToClient.call(srv))
            {
                if (srv.response.path.poses.size() >= 2)
                {
                    // Skip the first and last point from the path-planner as these are
                    // already in the fillPoints (from the coverage).
                    for (int i = 1; i < srv.response.path.poses.size() - 2; i++)
                    {
                        geometry_msgs::PoseStamped pose = srv.response.path.poses[i];
                        geometry_msgs::Point32 p;
                        p.x = pose.pose.position.x;
                        p.y = pose.pose.position.y;
                        p.z = pose.pose.position.z;
                        fillPoints.points.push_back(p);
                    }
                }
                else
                {
                    ROS_INFO("Returned path too short to be usable...");
                }
            }
        }
        else
        {
            // First time, save the origin point for later use
            coverageStart.position.x = oneAreaPoints.points[0].x;
            coverageStart.position.y = oneAreaPoints.points[0].y;
            coverageStart.position.z = oneAreaPoints.points[0].z;
        }

        // Copy the resulting path into the total path
        BOOST_FOREACH(geometry_msgs::Point32 point, oneAreaPoints.points)
        {
            fillPoints.points.push_back(point);
        }

        // Save this as the next start position
        start.position.x = oneAreaPoints.points[oneAreaPoints.points.size() - 1].x;
        start.position.y = oneAreaPoints.points[oneAreaPoints.points.size() - 1].y;
        start.position.z = oneAreaPoints.points[oneAreaPoints.points.size() - 1].z;

        allButFirstOne = true;
    }

    // Now convert/export this as a Path as well
    coveragePath.poses.clear();

    // Finally, add a path from startPose to firstposition in path.
    am_planning::PathToPose srv;
    srv.request.border.polygon = area;
    srv.request.start.pose = startPose;
    srv.request.goal.pose = coverageStart;

    if (fromToClient.call(srv))
    {
        if (srv.response.path.poses.size() >= 2)
        {
            // Skip last point from the path-planner as these are
            // already in the fillPoints (from the coverage).
            for (int i = 0; i < srv.response.path.poses.size() - 2; i++)
            {
                geometry_msgs::PoseStamped pose = srv.response.path.poses[i];
                pose.pose.position.x = pose.pose.position.x;
                pose.pose.position.y = pose.pose.position.y;
                pose.pose.position.z = pose.pose.position.z;
                coveragePath.poses.push_back(pose);
            }
        }
        else
        {
            ROS_INFO("Returned path too short to be usable...");
        }
    }

    BOOST_FOREACH(geometry_msgs::Point32 point, fillPoints.points)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = point.z;

        coveragePath.poses.push_back(pose);
    }

    patternOk = true;
    newPath = true;
}

void BasicCoverage::calculateFillOnConvexPolygon(geometry_msgs::Polygon& area, geometry_msgs::Polygon& oneAreaPoints)
{

    geometry_msgs::Polygon tempPoints;
    oneAreaPoints.points.clear();
    tempPoints.points.clear();

    // Idea:
    // Assume a convex polygon, and try to find a number of
    // scan-lines that will form a connected path

    MinMaxPoints mmP;
    findMinMaxPoly(mmP, area);

    // Calculate average point of polygon
    double avgX = 0;
    double avgY = 0;

    BOOST_FOREACH(geometry_msgs::Point32 point, area.points)
    {
        avgX += point.x;
        avgY += point.y;
    }

    avgX /= area.points.size();
    avgY /= area.points.size();

    // std::cout << "minX: " << mmP.minX << " maxX: " << mmP.maxX << std::endl;
    // std::cout << "minY: " << mmP.minY << " maxY: " << mmP.maxY << std::endl;

    // Move along X-axis

    // Step lines every delta meter
    double delta = corridorWidth;

    int lineCounter = 0;

    for (double y = mmP.minY; y < mmP.maxY; y += delta)
    {
        lineCounter++;

        // std::cout << "SCAN-LINE: " << y << std::endl;

        Line line;
        Line scanLine;

        scanLine.p1.x = mmP.minX;
        scanLine.p1.y = y;

        scanLine.p2.x = mmP.maxX;
        scanLine.p2.y = y;

        if (getLineFromPoly(scanLine, area, line))
        {
            // Add the scan line...
            if (lineCounter % 2)
            {
                tempPoints.points.push_back(line.p1);
                tempPoints.points.push_back(line.p2);
            }
            else
            {
                tempPoints.points.push_back(line.p2);
                tempPoints.points.push_back(line.p1);
            }
        }
    }

    // Add a "start" close to the start of the polygon
    double shrinkFactor = 1.0;
    geometry_msgs::Point32 towardsMid;
    double l, dx, dy;
    geometry_msgs::Point32 startPoint;
    geometry_msgs::Point32 endPoint;

    addShrink = false;
    fullFill = true;

    if (addShrink)
    {
        // Find vector towards mid-point
        towardsMid.x = avgX - tempPoints.points[0].x;
        towardsMid.y = avgY - tempPoints.points[0].y;
        towardsMid.z = tempPoints.points[0].z;

        l = sqrtf(towardsMid.x * towardsMid.x + towardsMid.y * towardsMid.y);
        dx = towardsMid.x * shrinkFactor / l;
        dy = towardsMid.y * shrinkFactor / l;

        startPoint.x = tempPoints.points[0].x + dx;
        startPoint.y = tempPoints.points[0].y + dy;
        startPoint.z = 0;
        oneAreaPoints.points.push_back(startPoint);
    }

    // Copy the rest
    if (fullFill)
    {

        BOOST_FOREACH(geometry_msgs::Point32 point, tempPoints.points)
        {
            oneAreaPoints.points.push_back(point);
        }
    }
    else
    {
        // This is a lazy mans fill...i.e. a temp diagonal... :)
        oneAreaPoints.points.push_back(tempPoints.points[0]);
        oneAreaPoints.points.push_back(tempPoints.points[tempPoints.points.size() - 1]);
    }

    if (addShrink)
    {
        // Add end inside, towards mid...
        towardsMid.x = avgX - tempPoints.points[tempPoints.points.size() - 1].x;
        towardsMid.y = avgY - tempPoints.points[tempPoints.points.size() - 1].y;
        towardsMid.z = tempPoints.points[tempPoints.points.size() - 1].z;

        l = sqrtf(towardsMid.x * towardsMid.x + towardsMid.y * towardsMid.y);
        dx = towardsMid.x * shrinkFactor / l;
        dy = towardsMid.y * shrinkFactor / l;

        endPoint.x = tempPoints.points[tempPoints.points.size() - 1].x + dx;
        endPoint.y = tempPoints.points[tempPoints.points.size() - 1].y + dy;
        endPoint.z = 0;

        oneAreaPoints.points.push_back(endPoint);
    }
}

bool BasicCoverage::setup()
{
    ROS_INFO("BasicCoverage::setup()");

    return true;
}

bool BasicCoverage::update(ros::Duration dt)
{
    static int cnt = 0;
    // unitTest();
    return true;
}

void BasicCoverage::publishFillPattern(ros::Time current_time)
{

    if (patternOk == false)
    {
        return;
    }

    visualization_msgs::Marker points;
    visualization_msgs::Marker line_strip;

    points.header.stamp = current_time;
    points.header.frame_id = "odom_combined";

    line_strip.header.stamp = current_time;
    line_strip.header.frame_id = "odom_combined";

    points.ns = "coverage_path";
    line_strip.ns = "coverage_path";

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    if (!fillPoints.points.empty())
    {
        points.action = visualization_msgs::Marker::ADD;
        line_strip.action = visualization_msgs::Marker::ADD;

        points.pose.orientation.w = 1.0;
        line_strip.pose.orientation.w = 1.0;

        points.scale.x = 0.1;
        points.scale.y = 0.1;

        line_strip.scale.x = 0.05;
        line_strip.scale.y = 0.05;
        line_strip.scale.z = 0.05;

        BOOST_FOREACH(geometry_msgs::Point32 point, fillPoints.points)
        {
            line_strip.points.push_back(toPoint(point));
            points.points.push_back(toPoint(point));
        }

        points.color.a = 1.0;
        points.color.b = 1.0;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
    }
    else
    {
        points.action = visualization_msgs::Marker::DELETE;
        line_strip.action = visualization_msgs::Marker::DELETE;
    }
}
}
