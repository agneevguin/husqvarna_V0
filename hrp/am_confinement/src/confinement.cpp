/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_confinement/confinement.h"

#include <tf/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <am_confinement/geometry_tools.h>
#include <am_driver/ConfinementStatus.h>
#include <am_planning/PolygonsFromPolygon.h>
#include <pwd.h>

#include <std_msgs/UInt16.h>

#include <fstream>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <boost/foreach.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

namespace Husqvarna
{

    
void shrinkPolygonCgal(double shrinkFactor, geometry_msgs::Polygon& input, geometry_msgs::Polygon& output);
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
// ConvexHull calculation
////////////////////////////////////////////////////////////////////////

bool pointEqual(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
    return ((p1.x == p2.x) && (p1.y == p2.y) && (p1.z == p2.z));
}

int findLeftMostPointIndex(geometry_msgs::PolygonStamped& poly)
{
    double minX = 1000.0;

    int index = 0;
    int foundIndex = -1;
    BOOST_FOREACH(geometry_msgs::Point32 point, poly.polygon.points)
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

void getConvexHull(geometry_msgs::PolygonStamped& input, geometry_msgs::PolygonStamped& output)
{
    // get leftmost point
    int leftMostIndex = findLeftMostPointIndex(input);
    geometry_msgs::Point32 vPointOnHull = input.polygon.points[leftMostIndex];

    geometry_msgs::Point32 vEndpoint;
    do
    {
        output.polygon.points.push_back(vPointOnHull);

        vEndpoint = input.polygon.points[0];

        for (int i = 1; i < input.polygon.points.size(); i++)
        {
            if (pointEqual(vPointOnHull, vEndpoint) ||
                (getOrientation(vPointOnHull, vEndpoint, input.polygon.points[i]) == -1))
            {
                vEndpoint = input.polygon.points[i];
            }
        }

        vPointOnHull = vEndpoint;

    } while (!pointEqual(vEndpoint, output.polygon.points[0]));
}

////////////////////////////////////////////////////////////////////////
// Shrink polygon
////////////////////////////////////////////////////////////////////////
void shrinkPolygon(double shrinkFactor, geometry_msgs::PolygonStamped& input, geometry_msgs::PolygonStamped& output)
{
    // Calculate average point of polygon
    double avgX = 0;
    double avgY = 0;

    BOOST_FOREACH(geometry_msgs::Point32 point, input.polygon.points)
    {
        avgX += point.x;
        avgY += point.y;
    }

    avgX /= input.polygon.points.size();
    avgY /= input.polygon.points.size();

    std::cout << "AVG point in polygon: (" << avgX << ", " << avgY << ") " << std::endl;

    // Copy & shrink
    BOOST_FOREACH(geometry_msgs::Point32 point, input.polygon.points)
    {
        // Find vector towards mid-point
        geometry_msgs::Point32 towardsMid;
        towardsMid.x = avgX - point.x;
        towardsMid.y = avgY - point.y;
        towardsMid.z = point.z;

        // Scale this one...
        // towardsMid.x *= (1.0 - shrinkFactor);
        // towardsMid.y *= (1.0 - shrinkFactor);
        // towardsMid.z *= (1.0 - shrinkFactor);
        // move point a little
        // point.x += towardsMid.x;
        // point.y += towardsMid.y;
        // point.z += towardsMid.z;

        // Metric dimnishing:
        // shrinkFactor defines how many meters the confinement shall be shrinked (in direction towards center point)
        // let dx and dy be the movement of point and s the lenght of that vector (shrinkFactor)
        // let l be the lenght of the vector from point p to the center point, let x,y be the length of x, y direction
        // of this vector.
        // by proportionality then s/l = dx/x = dy/y => dx=x*s/l and dy=y*s/l. This is verified by inserting this into
        // s=sqrt(dx² + dy²) = sqrt(s²/l²(dx² + dy²)) = s(sqrt(dx² + dy²)/l) = s by the definition of l.

        double l = sqrtf(towardsMid.x * towardsMid.x + towardsMid.y * towardsMid.y);
        double dx = towardsMid.x * shrinkFactor / l;
        double dy = towardsMid.y * shrinkFactor / l;

        // move point a little
        point.x += dx;
        point.y += dy;
        point.z += 0;

        // Add to list
        output.polygon.points.push_back(point);
    }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// Confinement class
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

Confinement::Confinement(const ros::NodeHandle& nodeh)
{
    // Init attributes
    this->nh = nodeh;

    // Setup some ROS stuff
    joy_sub = nh.subscribe("nano2", 1, &Confinement::joyCallback, this);
    status_sub = nh.subscribe("sensor_status", 1, &Confinement::statusCallback, this);
    odom_sub = nh.subscribe("odom_combined", 1, &Confinement::odomCallback, this);
    rviz_click_sub = nh.subscribe("clicked_point", 1, &Confinement::pointCallback, this);
    beacon_sub = nh.subscribe("beacon_pos", 1, &Confinement::beaconPosCallback, this);
    mode_sub = nh.subscribe("confinement_cmd", 1, &Confinement::modeCallback, this);

    border_pub = nh.advertise<geometry_msgs::PolygonStamped>("border", 1);
    stay_out_pub = nh.advertise<geometry_msgs::PolygonStamped>("stay_out", 10);
    rviz_click_pub = nh.advertise<visualization_msgs::Marker>("confinement_helpers", 100);
    col_pub = nh.advertise<am_driver::ConfinementStatus>("confinement", 1);

    simpleFromComplexClient = nh.serviceClient<am_planning::PolygonsFromPolygon>("complex_to_simple");
    decomposeClient = nh.serviceClient<am_planning::PolygonsFromPolygon>("convex_decompose");

    // Parameters
    ros::NodeHandle n_private("~");

    waitingForRelease = false;
    joyDisabled = true;

    newOdomData = false;

    hasClickedOnce = false;
    patternOk = false;

    newPath = false;

    n_private.param<double>("shrinkFactor", shrinkFactor, 0.30);
    n_private.param<double>("beaconRadius", beaconCollisionRadius, 0.75);

    struct passwd* pw = getpwuid(getuid());
    char* homedir = pw->pw_dir;
    config_file_name = homedir;
    
    std::string confinementFile;
    std::string defConfinementFile = (std::string)homedir + (std::string)"/.ros/confinement_config.txt";
    this->nh.param("confinementFile", confinementFile, defConfinementFile);
    config_file_name = confinementFile;
    
    borderCalculated = false;

    fixedConfWidth = 3.0;
    fixedConfHeight = 3.0;

    n_private.param<double>("fixedConfWidth", fixedConfWidth, 3.0);
    n_private.param<double>("fixedConfHeight", fixedConfHeight, 3.0);
}

Confinement::~Confinement()
{
}

void Confinement::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
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
        // Pressing S[5]
        if (j->buttons[25 + 6] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("Confinement::NANO KONTROL enabled!");
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing S[5]
        if (j->buttons[25 + 6] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("Confinement::NANO KONTROL disabled!");
            return;
        }
    }

    // STOP
    if (j->buttons[6] == 1)
    {
        ROS_INFO("CONFINEMENT - Clear Path & Borders");

        border.polygon.points.clear();
        return;
    }
    // RECORD
    if (j->buttons[8] == 1)
    {
        // This will take the current position,
        // create a "click" and inser this as it came
        // from rviz... ;)
        geometry_msgs::Point32 point;
        point.x = odom.pose.pose.position.x;
        point.y = odom.pose.pose.position.y;
        point.z = odom.pose.pose.position.z;

        createPolyFromClick(point);
    }

    // FWD
    if (j->buttons[5] == 1)
    {
    }

    double shrink = j->axes[6] * 2.0;
    if (shrink != 0)
    {
        shrinkFactor = shrink;
        ROS_INFO("shrinkFactor = %f", shrinkFactor);
    }
    else
    {
        shrinkFactor = 1.0;
        ROS_INFO("shrinkFactor = %f", shrinkFactor);
    }
    
    double width = j->axes[0] * 10.0;
    if (width != 0)
    {
        fixedConfWidth = width;
        ROS_INFO("fixedConfWidth = %f", fixedConfWidth);
    }
    else
    {
        fixedConfWidth = 3.0;
        ROS_INFO("fixedConfWidth = %f", fixedConfWidth);
    }

    double heigth = j->axes[1] * 10.0;
    if (heigth != 0)
    {
        fixedConfHeight = heigth;
        ROS_INFO("fixedConfHeight = %f", fixedConfHeight);
    }
    else
    {
        fixedConfHeight = 3.0;
        ROS_INFO("fixedConfHeight = %f", fixedConfHeight);
    }    
}

void Confinement::modeCallback(const std_msgs::UInt16::ConstPtr& msg)
{

    // BUILD COVERAGE FROM BEACONS
    if (msg->data == 0x01)
    {
        ROS_INFO("CONFINEMENT - Generate Beacon Polygon");

        // Generate a convex polygon
        generateBeaconPoly();

        return;
    }
    // CLEAR COVERAGE
    if (msg->data == 0x02)
    {
        ROS_INFO("CONFINEMENT - Clear Points, Path & Borders");

        border.polygon.points.clear();
        input.polygon.points.clear();

        // Remove all stay-outs
        stayOutPolygons.clear();
        stayInPolygons.clear();

        borderCalculated = false;

        for (int i = 1; i < 20; i++)
        {
            visualization_msgs::Marker cleanMe;
            cleanMe.header.frame_id = "odom_combined";
            cleanMe.type = visualization_msgs::Marker::LINE_STRIP;
            cleanMe.action = visualization_msgs::Marker::DELETE;
            cleanMe.id = i;
            cleanMe.ns = "stayin";
            rviz_click_pub.publish(cleanMe);
        }

        for (int i = 1; i < 20; i++)
        {
            visualization_msgs::Marker cleanMe;
            cleanMe.header.frame_id = "odom_combined";
            cleanMe.type = visualization_msgs::Marker::LINE_STRIP;
            cleanMe.action = visualization_msgs::Marker::DELETE;
            cleanMe.id = i+500;
            cleanMe.ns = "stayout";
            rviz_click_pub.publish(cleanMe);
        }

        return;
    }
    // RECORD POINT
    if (msg->data == 0x03)
    {
        ROS_INFO("CONFINEMENT - POINT STORED!");

        // This will take the current position,
        // create a "click" and inser this as it came
        // from rviz... ;)
        geometry_msgs::Point32 point;
        point.x = odom.pose.pose.position.x;
        point.y = odom.pose.pose.position.y;
        point.z = odom.pose.pose.position.z;

        createPolyFromClick(point);
    }

    // CLOSE Polygon
    if (msg->data == 0x07)
    {
        createPolyFromClick(startPoint);
        
    }
    if (msg->data == 0x09)
    {
        saveConfinementToFile(config_file_name);
    }
    
    if (msg->data == 0x0A)
    {
        loadConfinementFromFile(config_file_name);
    }
    // Generate CONFINEMENT around robot
    if ((msg->data >= 0x20) && (msg->data < 0x2f))
    {
        ROS_INFO("CONFINEMENT - Generate SQUARED confinement");
        generateConfinementAroundRobot(msg->data);
        return;
    }
}

void Confinement::statusCallback(const am_driver::SensorStatus::ConstPtr& msg)
{
    // Sensor status
    // std::cout << "Sensor Status:" << msg->sensorStatus << std::endl;

    // 0x04 - Collision
    // 0x02 - Out of area

    statusCollision = (msg->sensorStatus & 0x04);
    statusOutside = (msg->sensorStatus & 0x02);

    // std::cout << "statusCollision:" << statusCollision << std::endl;
    // std::cout << "statusOutside:" << statusOutside << std::endl;
}

void Confinement::loadConfinementFromFile(std::string fileName)
{
   
    std::string line;
    std::ifstream cfg_file(fileName.c_str());
    std::cout << "Loading from:" << fileName << std::endl;
    
    // Parsing: Look for the lines with border, stayIns and stayOuts. Posted in python (numpy) format
    if (cfg_file.is_open())
    {
        while (getline(cfg_file, line))
        {
            //~ cout << line << '\n';
            // Format <id x y z> example: DECA0100-101 1.75 4.54 1.05
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(" =([,])"),boost::token_compress_on);

            if (strs[0] == "border")
            {
                border.polygon.points.clear();
                int numPoints = (strs.size() - 1) / 2;
                //std::cout << "Import border polygon with " << (strs.size()-1)/2 << " points." << std::endl;
                for (int i = 2; i < strs.size()-1; i+=2)
                {
                    geometry_msgs::Point32 p;
                    p.x = boost::lexical_cast<float>(strs[i]);
                    p.y = boost::lexical_cast<float>(strs[i+1]);
                   // std::cout << "(" << p.x << "," << p.y << ")" << std::endl;
                    border.polygon.points.push_back(p);
                }
                borderCalculated = true;
            }
            else if (strs[0] == "stayInPolygons")
            {
                stayInPolygons.clear();
                
                // Split for each polygon
                std::vector<std::string> polyStrs;
                boost::split(polyStrs, line, boost::is_any_of("x"),boost::token_compress_on);
               // std::cout << "Import " << polyStrs.size() - 1 << " stay in regions."<< std::endl;
                for (int i = 1; i < polyStrs.size(); i++)
                {
                    
                    geometry_msgs::Polygon poly;
                    std::vector<std::string> newStrs;
                    boost::split(newStrs, polyStrs[i], boost::is_any_of(" =([,])"),boost::token_compress_on);

                    //border.polygon.points.clear();
                    int numPoints = (newStrs.size() - 1) / 2;
                  //  std::cout << i << ": Import stayInPolygon with " << (newStrs.size()-1)/2 << " points."<<  std::endl;
                    for (int i = 1; i < newStrs.size()-1; i+=2)
                    {
                        geometry_msgs::Point32 p;
                        p.x = boost::lexical_cast<float>(newStrs[i]);
                        p.y = boost::lexical_cast<float>(newStrs[i+1]);
                   //     std::cout << "(" << p.x << "," << p.y << ")" << std::endl;
                        poly.points.push_back(p);
                    }
                   // std::cout << "...done" << std::endl;
                    stayInPolygons.push_back(poly);
                    
                }
            }
            else if (strs[0] == "stayOutPolygons")
            {
                stayOutPolygons.clear();
                
                // Split for each polygon
                std::vector<std::string> polyStrs;
                boost::split(polyStrs, line, boost::is_any_of("x"),boost::token_compress_on);
              //  std::cout << "Import " << polyStrs.size() - 1 << " stay out regions."<< std::endl;
                for (int i = 1; i < polyStrs.size(); i++)
                {
                    
                    geometry_msgs::Polygon poly;
                    std::vector<std::string> newStrs;
                    boost::split(newStrs, polyStrs[i], boost::is_any_of(" =([,])"),boost::token_compress_on);

                    //border.polygon.points.clear();
                    int numPoints = (newStrs.size() - 1) / 2;
               //     std::cout << i << ": Import stayOutPolygon with " << (newStrs.size()-1)/2 << " points." <<  std::endl;
                    for (int i = 1; i < newStrs.size()-1; i+=2)
                    {
                        geometry_msgs::Point32 p;
                        p.x = boost::lexical_cast<float>(newStrs[i]);
                        p.y = boost::lexical_cast<float>(newStrs[i+1]);
                   //     std::cout << "(" << p.x << "," << p.y << ")" << std::endl;
                        poly.points.push_back(p);
                    }
                 //   std::cout << "...done" << std::endl;
                    stayOutPolygons.push_back(poly);
                    
                }
            }
        }
        cfg_file.close();
    }
    else
    {
         std::cout << "Unable to open file:" << fileName << std::endl;
    }
}

void Confinement::saveConfinementToFile(std::string fileName)
{
    std::ofstream cfg_file;
    cfg_file.open(fileName.c_str());

    std::cout << "Saving to:" << fileName << std::endl;
   
    if (!cfg_file.is_open())
    {
        ROS_ERROR("Failed to create file!");
        return;
    }

    cfg_file << "import numpy as np" << std::endl;
    
    cfg_file << "border = np.matrix([";
    for (int i = 0; i < border.polygon.points.size(); i++)
    {
        cfg_file << "[" << border.polygon.points[i].x << "," << border.polygon.points[i].y << "]";
        if (i < border.polygon.points.size() - 1)
        {
            cfg_file << ",";
        }
    }
    cfg_file << "])\n";
    
    cfg_file << "stayInPolygons = [";
    for (int i = 0; i < stayInPolygons.size(); i++)
    {
        cfg_file << "np.matrix([";
        for (int j = 0; j < stayInPolygons[i].points.size(); j++)
        {
            cfg_file << "[" << stayInPolygons[i].points[j].x << "," << stayInPolygons[i].points[j].y << "]";
            if (j < stayInPolygons[i].points.size() - 1)
            {
                cfg_file << ",";
            }
        }
        cfg_file << "])";
        if (i < stayInPolygons.size() - 1)
        {
            cfg_file << ",";
        }
    }
    cfg_file << "]\n";
    
    cfg_file << "stayOutPolygons = [";
    for (int i = 0; i < stayOutPolygons.size(); i++)
    {
        cfg_file << "np.matrix([";
        for (int j = 0; j < stayOutPolygons[i].points.size(); j++)
        {
            cfg_file << "[" << stayOutPolygons[i].points[j].x << "," << stayOutPolygons[i].points[j].y << "]";
            if (j < stayOutPolygons[i].points.size() - 1)
            {
                cfg_file << ",";
            }
        }
        cfg_file << "])";
        if (i < stayOutPolygons.size() - 1)
        {
            cfg_file << ",";
        }
    }
    cfg_file << "]\n";
}

double Confinement::getCollisionFactorPoly(geometry_msgs::Polygon poly, geometry_msgs::Point32 pos)
{
    double minDistance = 9999.0;

    double sign = -1.0;
    if (frontier_exploration::pointInPolygon(pos, poly))
    {
        sign = 1.0;
    }

    // Calculate how far we are from the border...

    int l = poly.points.size();

    // All lines but the last one
    for (int i = 0; i < l - 1; i++)
    {
        double d = distToSegment(pos, poly.points[i], poly.points[i + 1]);
        if (d < minDistance)
        {
            minDistance = d;
        }
    }

    // Last one...
    double d = distToSegment(pos, poly.points[0], poly.points[l - 1]);
    if (d < minDistance)
    {
        minDistance = d;
    }

    // Inside/outside?
    minDistance *= sign;

    return minDistance;
}

double Confinement::getCollisionFactor(geometry_msgs::Point32 pos)
{
    double minDistance = 9999.0;
    double maxDistance = -9999.0;

    //
    // Check the border (we should be inside this one)
    //
    /*    if (!border.polygon.points.empty())
        {
            double borderDistance = getCollisionFactorPoly(border.polygon, pos);
            if (borderDistance < minDistance)
            {
                minDistance = borderDistance;
            }
        }
    */
    //
    // Check all stay-in areas
    //
    std::vector<geometry_msgs::Polygon>::iterator it;
    it = stayInPolygons.begin();
    while (it != stayInPolygons.end())
    {
        // Test all stay-in polys...
        double stayInDistance = getCollisionFactorPoly(*it, pos);

        if (stayInDistance > maxDistance)
        {
            maxDistance = stayInDistance;
        }

        it++;
    }

    // We have now saved the "maximum" of all stay-ins, this is the
    // minDistance used for further checks.
    if (stayInPolygons.size() > 0)
    {
        minDistance = maxDistance;
    }

    //
    // Check all stay-out areas
    //
    it = stayOutPolygons.begin();
    while (it != stayOutPolygons.end())
    {
        // Test all stay-out polys...
        double stayOutDistance = getCollisionFactorPoly(*it, pos);

        // Invert this one (stay out compared to border)
        stayOutDistance = -stayOutDistance;

        if (stayOutDistance < minDistance)
        {
            minDistance = stayOutDistance;
        }

        it++;
    }

    //
    // Check all beacons, collision with a radius from these
    //
    if (beaconPositions.numBeacons > 0)
    {
        double beaconDist = getBeaconCollision(pos);
        // std::cout << "BeaconDist: " << beaconDist << std::endl;
        if (beaconDist < minDistance)
        {
            minDistance = beaconDist;
        }
    }
    
    return minDistance;
}

double Confinement::getBeaconCollision(geometry_msgs::Point32 pos)
{
    double dx;
    double dy;
    double dist;

    double minDist = 9999.0;

    // Check collision with Beacon
    // std::cout << "Check beacon collisions for " << beaconPositions.numBeacons << " beacons.";

    for (int i = 0; i < beaconPositions.numBeacons; i++)
    {
        dx = pos.x - beaconPositions.beaconPosX[i];
        dy = pos.y - beaconPositions.beaconPosY[i];
        dist = sqrt(dx * dx + dy * dy);
        // Calculate a distance to the radius
        // Inside: return a negative distance
        // Outside: return a positive distance
        dist = dist - beaconCollisionRadius;
        if (dist < minDist)
        {
            minDist = dist;
        }
    }
    return minDist;
}

void Confinement::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Copy & save for use in update
    odom = *msg;

    double avgCol = 0.0;

    tf::StampedTransform transform;
    try
    {
        geometry_msgs::Point32 p;

        // REAR RIGHT
        listener.lookupTransform("/odom_combined", "/loop_rear_right", ros::Time(0), transform);
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = transform.getOrigin().z();

        conStatus.rearRightCollision = getCollisionFactor(p);
        // avgCol += conStatus.rearRightCollision;
        // std::cout << "POS: " << p.x << " " << p.y << " " << p.z << " = " << conStatus.rearRightCollision <<
        // std::endl;

        // REAR LEFT
        listener.lookupTransform("/odom_combined", "/loop_rear_left", ros::Time(0), transform);
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = transform.getOrigin().z();
        conStatus.rearLeftCollision = getCollisionFactor(p);
        // avgCol += conStatus.rearLeftCollision;

        // FRONT RIGHT
        listener.lookupTransform("/odom_combined", "/loop_front_right", ros::Time(0), transform);
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = transform.getOrigin().z();
        conStatus.frontRightCollision = getCollisionFactor(p);
        // avgCol += conStatus.frontRightCollision;

        // FRONT LEFT
        listener.lookupTransform("/odom_combined", "/loop_front_center", ros::Time(0), transform);
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = transform.getOrigin().z();
        conStatus.frontLeftCollision = getCollisionFactor(p);
        // avgCol += conStatus.frontLeftCollision;
        // std::cout << "POS: " << p.x << " " << p.y << " " << p.z << " = " << conStatus.frontLeftCollision <<
        // std::endl;

        // avgCol /= 4.0;
        // std::cout << "AvgCol: " << avgCol << std::endl;
    }
    catch (tf::TransformException ex)
    {
        // ROS_ERROR("%s",ex.what());
    }

    newOdomData = true;
}

void Confinement::pointCallback(const geometry_msgs::PointStamped::ConstPtr& point)
{
    geometry_msgs::Point32 p;

    p.x = point->point.x;
    p.y = point->point.y;
    p.z = point->point.z;

    createPolyFromClick(p);
}

void Confinement::setBorderFromComplex(bool shrink, geometry_msgs::PolygonStamped complexPoly)
{
    geometry_msgs::PolygonStamped simplePoly;

    // Remove old one...
    border.polygon.points.clear();
    stayInPolygons.clear();

    // SRV: complexToSimple
    am_planning::PolygonsFromPolygon srv;
    srv.request.inputPolygon = complexPoly;
    if (simpleFromComplexClient.call(srv))
    {
        simplePoly = srv.response.outputPolygons[0];
        ROS_INFO("Confinement::GotSimplePoly");
    }
    else
    {
        ROS_ERROR("simpleFromComplexClient.call(srv) failed, using convex hull instead...");
        getConvexHull(complexPoly, simplePoly);
    }
    // Shrink the simplePoly to become the border
    if (shrink)
    {
        //shrinkPolygon(shrinkFactor, simplePoly, border);
        ROS_INFO("Shrink...!");
        shrinkPolygonCgal(shrinkFactor, simplePoly.polygon, border.polygon);
        ROS_INFO("... Done Shrink...!");
    }
    else
    {
        border = simplePoly;
    }

    // SRV: convexDecomposition
    srv.request.inputPolygon = border;
    if (decomposeClient.call(srv))
    {
        // Loop through the polygons and add to stay-in
        for (int i = 0; i < srv.response.outputPolygons.size(); i++)
        {
            stayInPolygons.push_back(srv.response.outputPolygons[i].polygon);
        }
        ROS_INFO("Confinement::GotDecomposition");
    }
    else
    {
        ROS_ERROR("decomposeClient.call(srv) failed, using the border as is...a warning!!!");
        stayInPolygons.push_back(border.polygon);
    }

    borderCalculated = true;
}

void Confinement::createPolyFromClick(geometry_msgs::Point32& point)
{
    hasClickedOnce = true;

    if (input.polygon.points.empty())
    {
        // first control point, so initialize
        input.header.frame_id = "odom_combined";
        input.polygon.points.push_back(point);
        startPoint = point;
        ROS_INFO("Confinement::Clicked FIRST point!");
    }
    else if (input.polygon.points.size() > 1 &&
             frontier_exploration::pointsNearby(input.polygon.points.front(), point, 0.3))
    {
        // check if last boundary point, i.e. nearby to first point
        if (input.polygon.points.size() < 3)
        {
            ROS_ERROR("Confinement::Not a valid polygon, restarting");
            input.polygon.points.clear();
        }
        else
        {
            ROS_INFO("Confinement::Done!");
            if (borderCalculated == false)
            {
                // border.polygon.points = input.polygon.points;
                setBorderFromComplex(false, input);
            }
            else
            {
                // New polygon, though border exists. This means stay away area!
                ROS_INFO("Confinement::Added new stay out area!");
                stayOutPolygons.push_back(input.polygon);
            }
            input.polygon.points.clear();
        }
    }
    else
    {
        // otherwise, must be a regular point inside boundary polygon
        input.polygon.points.push_back(point);
        input.header.stamp = ros::Time::now();
        ROS_INFO("Confinement::Clicked point!");
    }
}

void Confinement::beaconPosCallback(const am_driver::BeaconPositions::ConstPtr& msg)
{
    beaconMutex.lock();
    // Copy and store these...
    beaconPositions = *msg;
    beaconMutex.unlock();
}

void Confinement::generateBeaconPoly(void)
{
    geometry_msgs::PolygonStamped complexPoly;

    //
    // Add the beacons in ID-order!
    // Used to be able to define a specific confinement
    // that does not/should not be optimized.
    //

    // Create a set (sorted) of id's, and a lookup of their position in the
    // original list
    std::set<std::pair<std::string, int> > idLookUp;

    for (int i = 0; i < beaconPositions.numBeacons; i++)
    {
        std::pair<std::string, int> item;
        item = std::make_pair(beaconPositions.beaconId[i], i);
        idLookUp.insert(item);
    }

    std::set<std::pair<std::string, int> >::iterator it;
    for (it = idLookUp.begin(); it != idLookUp.end(); ++it)
    {
        std::pair<std::string, int> item = *it;

        int index = item.second;

        // std::cout << "ID: " << item.first << " = " << item.second << std::endl;

        geometry_msgs::Point32 p;

        p.x = beaconPositions.beaconPosX[index];
        p.y = beaconPositions.beaconPosY[index];
        p.z = beaconPositions.beaconPosZ[index];

        complexPoly.polygon.points.push_back(p);
    }

    setBorderFromComplex(true, complexPoly);
}

void Confinement::generateConfinementAroundRobot(int shapeNum)
{
    geometry_msgs::PolygonStamped complexPoly;
    geometry_msgs::Point32 p;
    complexPoly.polygon.points.clear();
    
    for (int i = 1; i < 20; i++)
    {
        visualization_msgs::Marker cleanMe;
        cleanMe.header.frame_id = "odom_combined";
        cleanMe.type = visualization_msgs::Marker::LINE_STRIP;
        cleanMe.action = visualization_msgs::Marker::DELETE;
        cleanMe.id = i;
        cleanMe.ns = "stayin";
        rviz_click_pub.publish(cleanMe);
    }

    // Get the robot position
    geometry_msgs::Point32 bot;
    bot.x = odom.pose.pose.position.x;
    bot.y = odom.pose.pose.position.y;
    bot.z = odom.pose.pose.position.z;

    switch (shapeNum)
    {
    case 0x21:

        // 1
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 2
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 3
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y + 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 4
        p.x = bot.x + 1.5 * fixedConfWidth;
        p.y = bot.y + 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 5
        p.x = bot.x + 1.5 * fixedConfWidth;
        p.y = bot.y - 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 6
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y - 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        break;

    case 0x22:

        // 1
        p.x = bot.x - 1.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 2
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 3
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y + 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 4
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y + 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 5
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 6
        p.x = bot.x + 1.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 7
        p.x = bot.x + 1.5 * fixedConfWidth;
        p.y = bot.y - 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 8
        p.x = bot.x - 1.5 * fixedConfWidth;
        p.y = bot.y - 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);
        
        break;
        
    case 0x23:

        // 1
        p.x = bot.x - 1.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 2
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 3
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y + 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 4
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y + 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 5
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 6
        p.x = bot.x + 1.5 * fixedConfWidth;
        p.y = bot.y + 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);
        
        // 7
        p.x = bot.x + 1.5 * fixedConfWidth;
        p.y = bot.y - 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 8
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y - 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 9
        p.x = bot.x + 0.5 * fixedConfWidth;
        p.y = bot.y - 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);
        
        // 10
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y - 0.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);
        
        // 11
        p.x = bot.x - 0.5 * fixedConfWidth;
        p.y = bot.y - 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // 12
        p.x = bot.x - 1.5 * fixedConfWidth;
        p.y = bot.y - 1.5 * fixedConfHeight;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);
        
        break;
        
    case 0x20:
    default:

        // LL
        p.x = bot.x - fixedConfWidth / 2.0;
        p.y = bot.y - fixedConfHeight / 2.0;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // LR
        p.x = bot.x + fixedConfWidth / 2.0;
        p.y = bot.y - fixedConfHeight / 2.0;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // TR
        p.x = bot.x + fixedConfWidth / 2.0;
        p.y = bot.y + fixedConfHeight / 2.0;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        // TL
        p.x = bot.x - fixedConfWidth / 2.0;
        p.y = bot.y + fixedConfHeight / 2.0;
        p.z = bot.z;
        complexPoly.polygon.points.push_back(p);

        break;
    }

    setBorderFromComplex(false, complexPoly);
}

bool Confinement::setup()
{
    ROS_INFO("Confinement::setup()");

    return true;
}

bool Confinement::update(ros::Duration dt)
{
    ros::Time current_time = ros::Time::now();

    // PUBLISHING
    if (borderCalculated)
    {
        border.header.stamp = current_time;
        border.header.frame_id = "odom_combined";
        border_pub.publish(border);
    }

    std::vector<geometry_msgs::Polygon>::iterator it;
    it = stayOutPolygons.begin();
    while (it != stayOutPolygons.end())
    {
        geometry_msgs::PolygonStamped stayOutPoly;
        stayOutPoly.header.stamp = current_time;
        stayOutPoly.header.frame_id = "odom_combined";
        stayOutPoly.polygon = *it;
        stay_out_pub.publish(stayOutPoly);
        it++;
    }

    conStatus.header.stamp = current_time;
    conStatus.header.frame_id = "odom_combined";
    col_pub.publish(conStatus);

    publishBorderClicks();
    publishConfinement();
    return true;
}

void Confinement::publishBorderClicks(void)
{

    if (hasClickedOnce == false)
    {
        return;
    }

    visualization_msgs::Marker points;
    visualization_msgs::Marker line_strip;

    points.header = input.header;
    line_strip.header = input.header;

    points.ns = "explore_points";
    line_strip.ns = "explore_points";

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    if (!input.polygon.points.empty())
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

        BOOST_FOREACH(geometry_msgs::Point32 point, input.polygon.points)
        {
            line_strip.points.push_back(toPoint(point));
            points.points.push_back(toPoint(point));
        }

        if (borderCalculated)
        {
            points.color.a = 1.0;
            points.color.r = 1.0;
            points.color.g = 0.0;
            points.color.b = 0.0;

            line_strip.color.a = 1.0;
            line_strip.color.r = 1.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 0.0;
        }
        else
        {
            points.color.a = 1.0;
            points.color.r = 0.0;
            points.color.g = 1.0;
            points.color.b = 0.0;

            line_strip.color.a = 1.0;
            line_strip.color.r = 0.0;
            line_strip.color.g = 1.0;
            line_strip.color.b = 0.0;
        }
    }
    else
    {
        points.action = visualization_msgs::Marker::DELETE;
        line_strip.action = visualization_msgs::Marker::DELETE;
    }

    rviz_click_pub.publish(points);
    rviz_click_pub.publish(line_strip);
}

void Confinement::publishConfinement(void)
{
    //
    // Add the BORDER
    //
    visualization_msgs::Marker borderStrip;

    borderStrip.header.frame_id = "odom_combined";
    borderStrip.ns = "border";
    borderStrip.id = 0;
    borderStrip.type = visualization_msgs::Marker::LINE_STRIP;

    if (!border.polygon.points.empty())
    {
        borderStrip.action = visualization_msgs::Marker::ADD;

        borderStrip.pose.orientation.w = 1.0;

        borderStrip.scale.x = 0.06;
        borderStrip.scale.y = 0.06;
        borderStrip.scale.z = 0.06;

        BOOST_FOREACH(geometry_msgs::Point32 point, border.polygon.points)
        {
            borderStrip.points.push_back(toPoint(point));
        }

        // Add last line...
        borderStrip.points.push_back(toPoint(border.polygon.points[0]));

        borderStrip.color.r = 0.0;
        borderStrip.color.g = 1.0;
        borderStrip.color.b = 0.0;
        borderStrip.color.a = 0.5;
    }
    else
    {
        borderStrip.action = visualization_msgs::Marker::DELETE;
    }

    rviz_click_pub.publish(borderStrip);

    //
    // Add the STAYIN AREAS
    //
    int id = 1;
    std::vector<geometry_msgs::Polygon>::iterator it;
    it = stayInPolygons.begin();
    while (it != stayInPolygons.end())
    {
        geometry_msgs::Polygon poly = *it;

        if (!poly.points.empty())
        {
            visualization_msgs::Marker stayInStrip;
            stayInStrip.header.frame_id = "odom_combined";
            stayInStrip.ns = "stayin";
            stayInStrip.id = id++;
            stayInStrip.type = visualization_msgs::Marker::LINE_STRIP;

            stayInStrip.action = visualization_msgs::Marker::ADD;

            stayInStrip.pose.orientation.w = 1.0;

            stayInStrip.scale.x = 0.06;
            stayInStrip.scale.y = 0.06;
            stayInStrip.scale.z = 0.06;

            BOOST_FOREACH(geometry_msgs::Point32 point, poly.points)
            {
                stayInStrip.points.push_back(toPoint(point));
            }

            // Add last line...
            stayInStrip.points.push_back(toPoint(poly.points[0]));

            stayInStrip.color.r = 1.0;
            stayInStrip.color.g = 1.0;
            stayInStrip.color.b = 0.0;
            stayInStrip.color.a = 0.5;

            rviz_click_pub.publish(stayInStrip);
        }

        it++;
    }

    //
    // Add the STAYOUT AREAS
    //
    id = 500;
    it = stayOutPolygons.begin();
    while (it != stayOutPolygons.end())
    {
        geometry_msgs::Polygon poly = *it;

        if (!poly.points.empty())
        {
            visualization_msgs::Marker stayOutStrip;
            stayOutStrip.header.frame_id = "odom_combined";
            stayOutStrip.ns = "stayout";
            stayOutStrip.id = id++;
            stayOutStrip.type = visualization_msgs::Marker::LINE_STRIP;

            stayOutStrip.action = visualization_msgs::Marker::ADD;

            stayOutStrip.pose.orientation.w = 1.0;

            stayOutStrip.scale.x = 0.06;
            stayOutStrip.scale.y = 0.06;
            stayOutStrip.scale.z = 0.06;

            BOOST_FOREACH(geometry_msgs::Point32 point, poly.points)
            {
                stayOutStrip.points.push_back(toPoint(point));
            }

            // Add last line...
            stayOutStrip.points.push_back(toPoint(poly.points[0]));

            stayOutStrip.color.r = 1.0;
            stayOutStrip.color.g = 0.0;
            stayOutStrip.color.b = 0.0;
            stayOutStrip.color.a = 0.5;

            rviz_click_pub.publish(stayOutStrip);
        }

        it++;
    }

    //
    // Publish the Beacon radius
    //
    id = 1000;
    for (int i = 0; i < beaconPositions.numBeacons; i++)
    {
        visualization_msgs::Marker rangeCircle;

        rangeCircle.header.frame_id = "odom_combined";
        rangeCircle.ns = "beaconzones";
        rangeCircle.id = id++;

        rangeCircle.type = visualization_msgs::Marker::SPHERE;
        rangeCircle.action = visualization_msgs::Marker::ADD;

        rangeCircle.pose.position.x = beaconPositions.beaconPosX[i];
        rangeCircle.pose.position.y = beaconPositions.beaconPosY[i];

        rangeCircle.pose.orientation.x = 0.0;
        rangeCircle.pose.orientation.y = 0.0;
        rangeCircle.pose.orientation.z = 0.0;
        rangeCircle.pose.orientation.w = 1.0;

        rangeCircle.color.r = 1.0;
        rangeCircle.color.g = 0.0;
        rangeCircle.color.b = 0.0;
        rangeCircle.color.a = 0.3;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        rangeCircle.scale.x = 2.0 * beaconCollisionRadius;
        rangeCircle.scale.y = 2.0 * beaconCollisionRadius;
        rangeCircle.scale.z = 0.1;
        rangeCircle.pose.position.z = 0.0;

        rviz_click_pub.publish(rangeCircle);
    }
}
}
