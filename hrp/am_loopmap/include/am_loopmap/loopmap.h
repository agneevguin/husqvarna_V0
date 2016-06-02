/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#ifndef LOOPMAP_H
#define LOOPMAP_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <am_driver/Loop.h>

namespace Husqvarna
{

class LoopMap
{
public:
	LoopMap(const ros::NodeHandle& nodeh);
	~LoopMap();
	
	bool setup();
	bool update(ros::Duration dt);
	
private:
	void loopCallback(const am_driver::Loop::ConstPtr& msg);
	
	void updateMap(double x, double y, int loopValue);

	void updateProbaInMap(int mx, int my, double p);
	
	double mapFilter(double newValue, double oldValue);
	
	void createGaussianFilterL(double gKernel[][21], double std_deviation);
	void createGaussianFilterS(double gKernel[][5], double std_deviation);
	
	// ROS data
	ros::NodeHandle nh;
	ros::Publisher map_pub;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	ros::Subscriber loop_sub;
	
	// My Map
	std_msgs::Header theHeader;
	nav_msgs::MapMetaData theInfo;	
	nav_msgs::OccupancyGrid og;
	
	// Gaussian kernel
	double gKernelL[21][21];
	double gKernelS[5][5];
	double stdDev;
	double offset;
	double scaleFactor;
	
	int8_t* mapData;
};

typedef boost::shared_ptr<LoopMap> LoopMapPtr;

}

#endif
