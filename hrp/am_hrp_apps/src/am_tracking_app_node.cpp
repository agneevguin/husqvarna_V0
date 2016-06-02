/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include <iostream>
#include <ros/ros.h>
#include "am_hrp_apps/tracking_app.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "am_hrp_tracking_app_node");
    ros::NodeHandle n;

    ros::Time lastTime;

    Husqvarna::TrackingAppPtr track(new Husqvarna::TrackingApp(n));

    bool res = track->setup();
    if (!res)
    {
        return -1;
    }

    ros::Rate rate(15.0);

    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration dt = current_time - last_time;
        last_time = current_time;

        track->update(dt);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
