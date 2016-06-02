/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef TRACKING_APP_H
#define TRACKING_APP_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <std_msgs/UInt16.h>
#include <am_driver/SensorStatus.h>
#include <am_driver/ConfinementStatus.h>

namespace Husqvarna
{

class TrackingApp
{
public:
    TrackingApp(const ros::NodeHandle& nodeh);
    ~TrackingApp();

    bool setup();
    bool update(ros::Duration dt);

private:
    void statusCallback(const am_driver::SensorStatus::ConstPtr& msg);

    // ROS data
    ros::NodeHandle nh;
    ros::Publisher cmdPub;

    ros::Subscriber statusSub;

    int state;

    ros::Time startedAt;
    ros::Time stopppedAt;
    ros::Duration wantedMowingTime;

    int logCounter;

    int sensorStatus;
    int operationalMode;
    bool userStopped;
    bool inCS;
};

typedef boost::shared_ptr<TrackingApp> TrackingAppPtr;
}

#endif
