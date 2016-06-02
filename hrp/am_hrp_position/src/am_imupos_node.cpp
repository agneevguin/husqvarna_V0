/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */
#include <iostream>
#include <ros/ros.h>

#include "am_hrp_position/imu_position.h"

int main( int argc, char** argv )
{
	ros::init(argc,argv, "am_hrp_imupos_node");
	ros::NodeHandle n;

	ros::Time lastTime;

	Husqvarna::IMUPositionPtr pos(new Husqvarna::IMUPosition(n));
	
	bool res = pos->setup();
	if (!res)
	{
		return -1;
	}
	
	ros::Rate rate(100.0);
	ros::Time last_time = ros::Time::now();
	
	int updateCounter = 5;
	
	while( ros::ok() )
	{
		updateCounter--;
		if (updateCounter == 0)
		{
			updateCounter = 5;
			ros::Time current_time = ros::Time::now();
			ros::Duration dt = current_time - last_time;
			last_time = current_time;
			pos->update(dt);
		}

		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}
