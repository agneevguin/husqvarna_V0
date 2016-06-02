/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef STEERING_H
#define STEERING_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <std_msgs/UInt16.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <am_driver/SensorStatus.h>

namespace Husqvarna
{

typedef struct
{
    int command;
    double speed;
    double distance;
    double angle;
    int collisionState;
} command_t;

class StraightSteering
{
public:
    StraightSteering(const ros::NodeHandle& nodeh);
    ~StraightSteering();

    bool setup();
    bool update(ros::Duration dt);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& j);
    void printConfig(void);
    void stop(void);
    void statusCallback(const am_driver::SensorStatus::ConstPtr& msg);
    void gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void nmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr& msg);

    // ROS data
    ros::NodeHandle nh;
    ros::Subscriber joySub;
    ros::Subscriber statusSub;
    ros::Subscriber gpsVelSub;
    ros::Subscriber gpsFixSub;
    ros::Subscriber nmeaSub;
    ros::Publisher cmdPub;

    // Joy
    bool waitingForRelease;
    bool joyDisabled;

    int state; 
    int fixStatus;
    bool startControl;
    bool startInit;
    bool statusOutside;
    bool statusCollision;
    double speed;
    double latestHeading;
    double referenceHeading;
    
    bool doAverageHeading;
    double averageStart;
    double controlStart;
    double headingDiffSum;
    double headingCount;
    double headingAverage;
    double headingAverageStart;
    
    
    ros::Time initStartTime;
    ros::Time averageStartTime;
    
    double wMax;
    double wMin;
    double err;
    double kp;
};

typedef boost::shared_ptr<StraightSteering> StraightSteeringPtr;
}

#endif
