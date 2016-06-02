/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#ifndef COVMAP_H
#define COVMAP_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <am_driver/Loop.h>

namespace Husqvarna
{

class CovMap
{
public:
	CovMap(const ros::NodeHandle& nodeh);
	~CovMap();
	
	bool setup();
	bool update(ros::Duration dt);
	
private:
	void loopCallback(const am_driver::Loop::ConstPtr& msg);
	
	void updateGrassCut();
	void updateMap(double x, double y, double probValue);
	void updateProbaInMap(int mx, int my, double p);
	
	double mapFilter(double newValue, double oldValue);
	
	void createGaussianFilterL(double gKernel[][41], double std_deviation);
	void createGaussianFilterS(double gKernel[][21], double std_deviation);
	void createGaussianFilterVS(double gKernel[][11], double std_deviation);
	
	// ROS data
	ros::NodeHandle nh;
	ros::Publisher map_pub;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	
	// My Map
	std_msgs::Header theHeader;
	nav_msgs::MapMetaData theInfo;	
	nav_msgs::OccupancyGrid og;
	
	// Gaussian kernel
	double gKernelVS[11][11];
	double gKernelS[21][21];
	double gKernelL[41][41];
	double stdDev;
	double offset;
	double scaleFactor;
	
	int8_t* mapData;
};

typedef boost::shared_ptr<CovMap> CovMapPtr;

}

#endif
