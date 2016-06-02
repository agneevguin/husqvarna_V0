/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef CONFINEMENT_H
#define CONFINEMENT_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/UInt16.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <am_driver/SensorStatus.h>
#include <am_driver/ConfinementStatus.h>
#include <am_driver/BeaconPositions.h>
#include <nav_msgs/Path.h>

namespace Husqvarna
{

class Confinement
{
public:
    Confinement(const ros::NodeHandle& nodeh);
    ~Confinement();

    bool setup();
    bool update(ros::Duration dt);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& j);
    void statusCallback(const am_driver::SensorStatus::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void beaconPosCallback(const am_driver::BeaconPositions::ConstPtr& msg);
    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);

    void publishBorderClicks(void);
    void publishConfinement(void);

    double getCollisionFactorPoly(geometry_msgs::Polygon poly, geometry_msgs::Point32 pos);
    double getCollisionFactor(geometry_msgs::Point32 p);
    double getBeaconCollision(geometry_msgs::Point32 pos);

    void generateBeaconPoly(void);
    void createPolyFromClick(geometry_msgs::Point32& point);
    void generateConfinementAroundRobot(int shapeNum);

    void saveConfinementToFile(std::string fileName);
    void loadConfinementFromFile(std::string fileName);

    void setBorderFromComplex(bool shrink, geometry_msgs::PolygonStamped complexPoly);

    // ROS data
    ros::NodeHandle nh;

    ros::Publisher border_pub;
    ros::Publisher stay_out_pub;
    ros::Publisher rviz_click_pub;
    ros::Publisher col_pub;

    ros::Subscriber joy_sub;
    ros::Subscriber status_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber rviz_click_sub;
    ros::Subscriber beacon_sub;
    ros::Subscriber mode_sub;

    ros::ServiceClient simpleFromComplexClient;
    ros::ServiceClient decomposeClient;

    tf::TransformListener listener;
    tf::TransformBroadcaster br;

    // Mode
    bool defineCuttingAreaMode;

    // Joy
    bool waitingForRelease;
    bool joyDisabled;

    // Status of MOWER
    bool statusCollision;
    bool statusOutside;
    am_driver::ConfinementStatus conStatus;

    // Border
    geometry_msgs::PolygonStamped border;
    geometry_msgs::PolygonStamped input;
    std::vector<geometry_msgs::Polygon> stayOutPolygons;
    std::vector<geometry_msgs::Polygon> stayInPolygons;

    geometry_msgs::Point32 startPoint;
        
    // Copy of last odom position
    nav_msgs::Odometry odom;
    bool newOdomData;

    bool hasClickedOnce;
    bool borderCalculated;

    // Fill path
    geometry_msgs::Polygon fillPoints;
    bool patternOk;
    bool newPath;

    // Beacon positions
    am_driver::BeaconPositions beaconPositions;
    boost::mutex beaconMutex;

    std::string config_file_name;

    double shrinkFactor;
    double corridorWidth;
    double beaconCollisionRadius;

    double fixedConfWidth;
    double fixedConfHeight;
};

typedef boost::shared_ptr<Confinement> ConfinementPtr;
}

#endif
