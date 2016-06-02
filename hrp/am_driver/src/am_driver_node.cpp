/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include "am_driver/automower.h"

int main( int argc, char** argv )
{
	ros::init(argc,argv, "am_driver_node");
	ros::NodeHandle n;

	ros::Time lastTime;

	Husqvarna::AutomowerPtr am(new Husqvarna::Automower(n));
	
	bool res = am->setup();
	if (!res)
	{
		return -1;
	}
	
	ros::Rate rate(50.0);
	
	ros::Time last_time = ros::Time::now();
	
	while( ros::ok() )
	{
		ros::Time current_time = ros::Time::now();
		ros::Duration dt = current_time - last_time;
		last_time = current_time;
		am->update(dt);

		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}
