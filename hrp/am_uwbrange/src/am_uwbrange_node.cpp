/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */
#include <iostream>
#include <ros/ros.h>

#include "am_uwbrange/range.h"

int main( int argc, char** argv )
{
	ros::init(argc,argv, "am_uwbrange_node");
	ros::NodeHandle n;

	ros::Time lastTime;

	Husqvarna::RangePtr range(new Husqvarna::Range(n));
	
	bool res = range->setup();
	if (!res)
	{
		return -1;
	}

	range->run();
	
	return 0;
}
