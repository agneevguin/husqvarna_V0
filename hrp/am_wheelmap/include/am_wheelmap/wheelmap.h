/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#ifndef WHEELMAP_H
#define WHEELMAP_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <am_driver/Loop.h>
#include <am_driver/WheelEncoder.h>

namespace Husqvarna
{

class WheelMap
{
public:
	WheelMap(const ros::NodeHandle& nodeh);
	~WheelMap();
	
	bool setup();
	bool update(ros::Duration dt);
	
private:
	void loopCallback(const am_driver::Loop::ConstPtr& msg);
	
	void updateGrassCut();
	void updateMap(double x, double y, double probValue);
	void updateProbaInMap(int mx, int my, double p);
	
	void encoderCallback(const am_driver::WheelEncoder::ConstPtr& msg);
	double mapFilter(double newValue, double oldValue);
	// ROS data
	ros::NodeHandle nh;
	ros::Publisher map_pub;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	ros::Subscriber encoderSub;
	
	double lWheelEnc;
	double rWheelEnc;
		
	// My Map
	std_msgs::Header theHeader;
	nav_msgs::MapMetaData theInfo;	
	nav_msgs::OccupancyGrid og;
	
	// Gaussian kernel
	double offset;
	double scaleFactor;
	
	int8_t* mapData;
};

typedef boost::shared_ptr<WheelMap> WheelMapPtr;

}

#endif
