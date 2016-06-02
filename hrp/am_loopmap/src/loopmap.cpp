/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */
 
#include "am_loopmap/loopmap.h"

#include <tf/transform_datatypes.h>
#include <am_driver/Loop.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

namespace Husqvarna
{


#define ROUND(num) ((int)(num < 0 ? (num - 0.5) : (num + 0.5)))

#define AM_LOOP_WIDTH	(200)
#define AM_LOOP_HEIGHT	(200)

#define AM_MAP_SCALE_X (10.0)
#define AM_MAP_SCALE_Y (10.0)

static const unsigned char AM_LM_NO_INFORMATION = 255;
static const unsigned char AM_LM_LETHAL_OBSTACLE = 254;
static const unsigned char AM_LM_INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char AM_LM_FREE_SPACE = 0;

LoopMap::LoopMap(const ros::NodeHandle& nodeh)
{
	// Init attributes
	this->nh = nodeh;

	// Setup some ROS stuff
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
	loop_sub = nh.subscribe("loop", 1, &LoopMap::loopCallback, this);

	// Allocate the map
	//theMap = new Map(100,100);
	mapData = new int8_t[AM_LOOP_WIDTH*AM_LOOP_HEIGHT];
	memset(mapData, AM_LM_FREE_SPACE, sizeof(mapData));
	
	// Setup the header & info statically...
	theHeader.frame_id = "map";
	theInfo.resolution = 0.1;
	theInfo.width = AM_LOOP_WIDTH;
	theInfo.height = AM_LOOP_HEIGHT;	
	theInfo.origin.position.x = -10.0;
	theInfo.origin.position.y = -10.0;
	theInfo.origin.position.z = 0.0;

	// Parameters
	ros::NodeHandle n_private("~");
	
	double def_stdDev = 1.0;
	n_private.param("standard_deviation", stdDev, def_stdDev);
	ROS_INFO("Standard deviation: %f", stdDev);

	double def_offset = 0.2;
	n_private.param("offset", offset, def_offset);
	ROS_INFO("Offset: %f", offset);

	double def_scaleFactor = 0.8;
	n_private.param("scale_factor", scaleFactor, def_scaleFactor);
	ROS_INFO("Scale factor: %f", scaleFactor);

	// Create kernels...
    createGaussianFilterL(gKernelL, stdDev);
    createGaussianFilterS(gKernelS, stdDev);

}

LoopMap::~LoopMap()
{
	delete[] mapData;
}

bool LoopMap::setup()
{
	ROS_INFO("LoopMap::setup()");

	return true;
}

void LoopMap::loopCallback(const am_driver::Loop::ConstPtr& msg)
{
	//ROS_INFO("Loop-frontCenter: %d", msg->frontCenter);
	
	tf::StampedTransform transform;
	try
	{
		// FRONT CENTER
		listener.lookupTransform("/map", "/loop_front_center",  msg->header.stamp, transform);
		
		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();
		
		updateMap(x,y, msg->frontCenter);

		// REAR RIGHT
		listener.lookupTransform("/map", "/loop_rear_right",  msg->header.stamp, transform);
		
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		
		updateMap(x,y, msg->rearRight);

		// REAR LEFT
		listener.lookupTransform("/map", "/loop_rear_left",  msg->header.stamp, transform);
		
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		
		updateMap(x,y, msg->rearLeft);


	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		//ros::Duration(1.0).sleep();
	}	
}

double LoopMap::mapFilter(double newValue, double oldValue)
{
	double res;
	
	if (newValue > 2.0 )
	{
		// More certain of grass...
		res = oldValue + newValue/2.5;
	}
	else
	{
		// More uncertain of grass...
		res = oldValue - 0.1;
	}
	
	//res = newValue + oldValue;

	//std::cout << "LV:" << newValue << " " << oldValue <<  " " << res << std::endl;


	// Limit to what is nice for ROS map data [0...100]
	if (res < 0.0)
	{
		res = 0;
	}
	if (res > 100)
	{
		res = 100;
	}
	return res;
}

void LoopMap::updateMap(double x, double y, int loopValue)
{
	
	// From costmap_2d
	// 0		Free space (not used/seen yet)
	// 1...127	Non-Free (but no collision)
	// 128..252 Possibly collision
	// 253..255 Definitely collision
	
	//ROS_INFO("Loop (%f,%f) = %d", x,y, loopValue);
	
	int mx = ROUND(x*AM_MAP_SCALE_X) + AM_LOOP_WIDTH/2;
	int my = ROUND(y*AM_MAP_SCALE_Y) + AM_LOOP_HEIGHT/2;
	//ROS_INFO("Loop (%d,%d) = %d", mx,my, loopValue);
	
	if ((loopValue <= 12000) && (loopValue >= 1000))
	{
		// Probably collision collision
		double p = (double)loopValue;
		
		p = p / 12000;
		if (p > 1.0)
		{
			p = 1.0;
		}
		// Invert (12000 = low posibility)
		p = 1.0 - p;
		p = 1 + p*100;
		
		updateProbaInMap(mx, my, p);
		//ROS_INFO("Probably (%d,%d) = %d (%d)", mx,my, mapData[mx + my*100], loopValue);
	}
	else if (loopValue <= 1000)
	{
		// Definitley collision...
		double p = 100;
		updateProbaInMap(mx, my, p);
		//ROS_INFO("Definitley (%d,%d) = %d (%d)", mx,my, mapData[mx + my*100], loopValue);
	}
	else 
	{
		// Free space...i.e. > 500
		double p = 0;
		updateProbaInMap(mx, my, p);
		//ROS_INFO("Free space (%d,%d) = %d (%d)", mx,my, mapData[mx + my*100], loopValue);
	}
}

void LoopMap::updateProbaInMap(int mx, int my, double p)
{
	// Apply a gaussian to this
	double pMap[5][5];
	
    for(int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            pMap[i][j] = p * scaleFactor * gKernelS[i][j] - offset;
		}
    }

	// Update in map... 
	for (int x = -2; x <= 2; x++)
	{
		for(int y = -2; y <= 2; y++)
		{
			int xx = mx + x;
			int yy = my + y;
			
			// Check bounds...
			if ((xx >= 0) && (xx<AM_LOOP_WIDTH))
			{
				if ((yy >= 0) && (yy<AM_LOOP_HEIGHT))
				{
					double pv = pMap[x+2][y+2];
					mapData[xx + yy*AM_LOOP_WIDTH] = (int)mapFilter(pv, (double)mapData[xx + yy*AM_LOOP_WIDTH]);
				}
			}
		}
	}
}


bool LoopMap::update(ros::Duration dt)
{
	ros::Time current_time = ros::Time::now();
	theHeader.stamp = current_time;
	
	// Calculate the TF from the pose...
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.125));
	tf::Quaternion qyaw = tf::createQuaternionFromYaw(0.0);
	transform.setRotation(qyaw);
	
	// Send the TF
	br.sendTransform(tf::StampedTransform(transform, current_time, "map", "odom_combined"));

	// Publish the map
	//nav_msgs::OccupancyGrid *og = theMap->NewOccupancyGrid();
	//og->header = theHeader;
	//og->info = theInfo;
	
	og.header = theHeader;
	og.info = theInfo;
	og.data = std::vector<int8_t>(mapData, mapData + AM_LOOP_WIDTH*AM_LOOP_HEIGHT);
	
	map_pub.publish(og);
	
	return true;
}

void LoopMap::createGaussianFilterL(double gKernel[][21], double std_deviation)
{
	double sigma = std_deviation;
	double r, s = 2.0 * sigma * sigma;

	int w = 21;
	int h = 21;

	int hw = ((w-1)/2);
	int hh = ((h-1)/2);

	// sum is for normalization
	double sum = 0.0;

	// generate wxh kernel
	for (int x = -hw; x <= hw; x++)
	{
		for(int y = -hh; y <= hh; y++)
		{
			r = sqrt(x*x + y*y);
			gKernel[x + hw][y + hh] = (exp(-(r*r)/s))/(M_PI * s);
			sum += gKernel[x + hw][y + hh];
		}
	}
 
    // normalize the Kernel
	for(int i = 0; i < w; ++i)
	{
		for(int j = 0; j < w; ++j)
		{
			gKernel[i][j] /= sum;
		}
	}
}

void LoopMap::createGaussianFilterS(double gKernel[][5], double std_deviation)
{
	double sigma = std_deviation;
	double r, s = 2.0 * sigma * sigma;

	int w = 5;
	int h = 5;

	int hw = ((w-1)/2);
	int hh = ((h-1)/2);

	// sum is for normalization
	double sum = 0.0;

	// generate wxh kernel
	for (int x = -hw; x <= hw; x++)
	{
		for(int y = -hh; y <= hh; y++)
		{
			r = sqrt(x*x + y*y);
			gKernel[x + hw][y + hh] = (exp(-(r*r)/s))/(M_PI * s);
			sum += gKernel[x + hw][y + hh];
		}
	}
 
    // normalize the Kernel
	for(int i = 0; i < w; ++i)
	{
		for(int j = 0; j < w; ++j)
		{
			gKernel[i][j] /= sum;
		}
	}
}

}
