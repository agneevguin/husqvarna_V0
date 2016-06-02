/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#ifndef STEERING_H
#define STEERING_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <std_msgs/UInt16.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <am_driver/SensorStatus.h>
#include <am_driver/Loop.h>
#include <am_driver/Range.h>
#include <am_driver/ConfinementStatus.h>
#include <am_driver/BeaconPositions.h>

#include <nav_msgs/Path.h>

namespace Husqvarna
{

class Particle
{
public:
	Particle(double x, double y, double z, double h);
	~Particle();

	double xpos;
	double ypos;
	double zpos;
	double heading;
	double weight;
};

class Beacon : public Particle
{
public:
	Beacon();
	~Beacon();

	std::string id;
	double range;
	ros::Time lastSeen;
};

typedef struct 
{
	int command;
	double speed;
	double distance;
	double angle;
	int collisionState;
} command_t;

class RangeSteering
{
public:
	RangeSteering(const ros::NodeHandle& nodeh);
	~RangeSteering();
	
	bool setup();
	bool update(ros::Duration dt);
	
	
private:
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& j);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void rangeCallback(const am_driver::Range::ConstPtr& msg);
	void odomCombinedCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void cycleRangeCover(ros::Duration dt);
	void cycleDock(ros::Duration dt);
	void beaconPosCallback(const am_driver::BeaconPositions::ConstPtr& msg);
	void setDockPath();
	void statusCallback(const am_driver::SensorStatus::ConstPtr& msg);
	void printConfig(void);
	void stop(void);
	
	double calculateTargetHeading();
	
	// ROS data
	ros::NodeHandle nh;
	
	ros::Subscriber odomSub;
	ros::Subscriber odomCombinedSub;
	ros::Subscriber joySub;
	ros::Subscriber rangeSub;
	ros::Subscriber statusSub;
	ros::Subscriber beaconPosSub;

	ros::Publisher beaconPub;
	ros::Publisher cmdPub;
	ros::Publisher coveragePub;
	ros::Publisher modePub;
	
	
	nav_msgs::Path coveragePath;
	
	tf::TransformListener listener;
	tf::TransformBroadcaster br;
	
	// Copy of last command
	uint16_t nextMode;
	bool newGoal;

	// Copy of last odom position
	nav_msgs::Odometry odom;
	nav_msgs::Odometry odomCombined;

	Beacon beaconList[2];
	std::string robotId;
	// Joy 
	bool waitingForRelease;
	bool joyDisabled;
	bool addBeacons;
	// Ranging
	boost::mutex rangeMutex;
	bool newRange;
	int rangeCount;
	
	int state;
	int dockState;
	int rangeState;
	double minRange;
	int numInterBeaconSamples;
	
	bool statusCollision;
	bool statusOutside;
	
	
	double heading;
	bool doControl;
	bool doDock;
	double radius;
	double radiusInc;
	double speed;
	double speedDock;
	double turnSpeed;
	
	
	double rangeHome;
	double baselineOffset;
	double leftRange;
	double rightRange;
	double baseDist;
	double dockStart;
	bool dockStarted;
	
	double baseX;
	double baseY;	
	double x1,y1,x2,y2;
	
	/* Estimation stuff */
	double bearingEst;
	double bearingLeft;	
	double bearingRight;
	double headingEst;
	
			
	std::string beacon1Id;
	std::string beacon2Id;
	// Straight goal
	double distance;
	bool newOdomData;
	double odoHeading;
	double startHeading;
	double homeHeading;
	int followDirection;
	ros::Time tStart;
	ros::Time timeSample;
	bool passed180;
	
	bool seekDockStart;
	// PID
	double err;
	double errOld;
	double iErr;
	double dErr;
	double errDock;
	double errOldDock;
	double iErrDock;
	double dErrDock;
	
	double kp;
	double ki;
	double kd;
	
	double kpDock;
	double kiDock;
	double kdDock;
		 
	/* Local positioning */ 
	double cosAlpha;
	double xL;
	double yL;
	double dx;
	double dy;
	double distToHome;
	double angleToHome;


};

typedef boost::shared_ptr<RangeSteering> RangeSteeringPtr;

}

#endif
