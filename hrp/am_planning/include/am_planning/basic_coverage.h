/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef BASIC_COVERAGE_H
#define BASIC_COVERAGE_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/UInt16.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <am_driver/SensorStatus.h>
#include <am_driver/ConfinementStatus.h>
#include <am_driver/BeaconPositions.h>
#include <nav_msgs/Path.h>
#include <am_planning/PathFromPolygon.h>
#include <am_planning/PolygonsFromPolygon.h>
#include <am_planning/PathToPose.h>

namespace Husqvarna
{

class BasicCoverage
{
public:
    BasicCoverage(const ros::NodeHandle& nodeh);
    ~BasicCoverage();

    bool setup();
    bool update(ros::Duration dt);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& j);
    void statusCallback(const am_driver::SensorStatus::ConstPtr& msg);
    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);
    bool getPathFromPoly(am_planning::PathFromPolygon::Request& req, am_planning::PathFromPolygon::Response& res);
    bool getPolygonsFromPoly(am_planning::PolygonsFromPolygon::Request& req,
                             am_planning::PolygonsFromPolygon::Response& res);
    bool getBorderPathFromPoly(am_planning::PathFromPolygon::Request& req, am_planning::PathFromPolygon::Response& res);
    bool getSimpleFromComplex(am_planning::PolygonsFromPolygon::Request& req,
                              am_planning::PolygonsFromPolygon::Response& res);
    bool getPathToGoal(am_planning::PathToPose::Request& req, am_planning::PathToPose::Response& res);

    void publishBorderClicks(void);

    double getCollisionFactor(geometry_msgs::Point32 p);
    void publishFillPattern(ros::Time current_time);

    void calculateFillOnConvexPolygon(geometry_msgs::Polygon& area, geometry_msgs::Polygon& oneAreaPoints);
    void calculateFill(geometry_msgs::Pose& startPose, geometry_msgs::Polygon& area);

    void calculatePathTo(geometry_msgs::Pose& startPose);
    void calculateAround(void);

    void unitTest();

    // ROS data
    ros::NodeHandle nh;

    ros::Publisher coverage_pub;
    ros::ServiceServer covService;
    ros::ServiceServer borderPathService;
    ros::ServiceServer pathToGoalService;
    ros::ServiceServer decomposePolygonService;
    ros::ServiceServer complexToSimpleService;
    ros::Subscriber joy_sub;
    ros::Publisher border_pub;

    ros::ServiceClient fromToClient;

    // Joy
    bool waitingForRelease;
    bool joyDisabled;

    // Border
    geometry_msgs::PolygonStamped border;
    geometry_msgs::PolygonStamped input;
    geometry_msgs::PoseStamped goal;

    // Fill path
    geometry_msgs::Polygon fillPoints;
    bool patternOk;
    nav_msgs::Path coveragePath;
    bool newPath;

    //
    double shrinkFactor;
    double corridorWidth;

    bool addShrink;
    bool fullFill;
};

typedef boost::shared_ptr<BasicCoverage> BasicCoveragePtr;
}

#endif
