/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */
 
#include "am_wheelmap/wheelmap.h"
#include <am_driver/WheelEncoder.h>

#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

namespace Husqvarna
{

#define ROUND(num) ((int)(num < 0 ? (num - 0.5) : (num + 0.5)))

#define AM_MAP_WIDTH	(2000)
#define AM_MAP_HEIGHT	(2000)

#define AM_MAP_SCALE_X (100.0)
#define AM_MAP_SCALE_Y (100.0)

static const unsigned char AM_LM_NO_INFORMATION = 255;
static const unsigned char AM_LM_LETHAL_OBSTACLE = 254;
static const unsigned char AM_LM_INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char AM_LM_FREE_SPACE = 0;


void WheelMap::encoderCallback(const am_driver::WheelEncoder::ConstPtr &msg)
{
	//~ Header header
	//~ ROS_INFO("lwheel %f", msg->lwheel);
	//~ ROS_INFO("rwheel %f", msg->rwheel);
	this->rWheelEnc = msg->rwheel;
	this->lWheelEnc = msg->lwheel;
	

}

WheelMap::WheelMap(const ros::NodeHandle& nodeh)
{
	// Init attributes
	this->nh = nodeh;

	// Setup some ROS stuff
	
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
	ROS_INFO("nh.subscribe, wheel_encoder\n");
	encoderSub = nh.subscribe("wheel_encoder", 1, &WheelMap::encoderCallback, this);

	//ros::spin();
	// Allocate the map
	mapData = new int8_t[AM_MAP_WIDTH*AM_MAP_HEIGHT];
	memset(mapData, AM_LM_FREE_SPACE, sizeof(mapData));
	
	// Setup the header & info statically...
	theHeader.frame_id = "map2";
	theInfo.resolution = 0.01;
	theInfo.width = AM_MAP_WIDTH;
	theInfo.height = AM_MAP_HEIGHT;	
	theInfo.origin.position.x = -10.0;
	theInfo.origin.position.y = -10.0;
	theInfo.origin.position.z = 0.0;

	// Parameters
	ros::NodeHandle n_private("~");
	
	double def_offset = 0.0;
	n_private.param("offset", offset, def_offset);
	ROS_INFO("Offset: %f", offset);

	double def_scaleFactor = 15.0;
	n_private.param("scale_factor", scaleFactor, def_scaleFactor);
	ROS_INFO("Scale factor: %f", scaleFactor);




}

WheelMap::~WheelMap()
{
	delete[] mapData;
}

bool WheelMap::setup()
{
	ROS_INFO("WheelMap::setup()");

	return true;
}


void WheelMap::updateGrassCut()
{
	tf::StampedTransform transform;
	try
	{

		double x;
		double y;
		
		// REAR RIGHT
		listener.lookupTransform("/map2", "/back_right_wheel",  ros::Time(0), transform);
		
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		
		if (this->rWheelEnc > 0.00001 || this->rWheelEnc < -0.00001)
			updateMap(x,y, 1.0);

		// REAR LEFT
		listener.lookupTransform("/map2", "/back_left_wheel",  ros::Time(0), transform);
		
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		
		if (this->lWheelEnc > 0.00001 || this->lWheelEnc < -0.00001)
			updateMap(x,y, 1.0);
			
		// FRONT RIGHT
		listener.lookupTransform("/map2", "/front_right_swivel_wheel",  ros::Time(0), transform);
		
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		
		/* Skipp mapping of front wheels for now! */
		//~ if (this->rWheelEnc > 0.00001 || this->rWheelEnc < -0.00001 || this->lWheelEnc > 0.00001 || this->lWheelEnc < -0.00001)
			//~ updateMap(x,y, 1.0);

		// FRONT LEFT
		listener.lookupTransform("/map2", "/front_left_swivel_wheel",  ros::Time(0), transform);
		
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		
		/* Skipp mapping of front wheels for now! */
		//~ if (this->rWheelEnc > 0.00001 || this->rWheelEnc < -0.00001 || this->lWheelEnc > 0.00001 || this->lWheelEnc < -0.00001)
			//~ updateMap(x,y, 1.0);
		//~ 
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}	
}

double WheelMap::mapFilter(double newValue, double oldValue)
{
	double res = newValue + oldValue;
	
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

void WheelMap::updateMap(double x, double y, double probValue)
{
	
	// From costmap_2d
	// 0		Free space (not used/seen yet)
	// 1...127	Non-Free (but no collision)
	// 128..252 Possibly collision
	// 253..255 Definitely collision

	// Calculate map coords
	int mx = ROUND(x*AM_MAP_SCALE_X) + AM_MAP_WIDTH/2;
	int my = ROUND(y*AM_MAP_SCALE_Y) + AM_MAP_HEIGHT/2;
	
	updateProbaInMap(mx, my, probValue*100.0);
	
}

void WheelMap::updateProbaInMap(int mx, int my, double p)
{

	// Update in map... 
	for (int x = -2; x <= 2; x++)
	{
		for(int y = -2; y <= 2; y++)
		{
			int xx = mx + x;
			int yy = my + y;
			
			// Check bounds...
			if ((xx >= 0) && (xx<AM_MAP_WIDTH))
			{
				if ((yy >= 0) && (yy<AM_MAP_HEIGHT))
				{
					/* TODO: check timing, only add if crossing again... */
					mapData[xx + yy*AM_MAP_WIDTH] = (int)mapFilter(10.0, (double)mapData[xx + yy*AM_MAP_WIDTH]);
				}
			}
		}
	}
}


bool WheelMap::update(ros::Duration dt)
{
	ros::Time current_time = ros::Time::now();
	theHeader.stamp = current_time;
	
	// Calculate the TF from the pose...
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.125));
	tf::Quaternion qyaw = tf::createQuaternionFromYaw(0.0);
	transform.setRotation(qyaw);
	
	// Send the TF
	br.sendTransform(tf::StampedTransform(transform, current_time, "map2", "odom_combined"));

	// Do the mapping
	updateGrassCut();

	// Publish the map
	//nav_msgs::OccupancyGrid *og = theMap->NewOccupancyGrid();
	//og->header = theHeader;
	//og->info = theInfo;
	
	og.header = theHeader;
	og.info = theInfo;
	og.data = std::vector<int8_t>(mapData, mapData + AM_MAP_WIDTH*AM_MAP_HEIGHT);
	
	map_pub.publish(og);
	
	return true;
}


}
