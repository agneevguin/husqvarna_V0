/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef IMU_POSITION_H
#define IMU_POSITION_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <am_driver/Range.h>
#include <am_driver/WheelEncoder.h>
#include <am_driver/BeaconPositions.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Imu.h>

namespace Husqvarna
{

class IMUPosition
{
public:
    IMUPosition(const ros::NodeHandle& nodeh);
    ~IMUPosition();

    bool setup();
    bool update(ros::Duration dt);

private:
    // Methods
    void odomCallback(const nav_msgs::Odometry::ConstPtr& o);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& j);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg);

    void setDefaultValues(void);

    bool stateEstimatePosition(ros::Time currentTime);

    // Publish data
    bool publishData(ros::Duration dt, ros::Time currentTime);

    // ROS data
    ros::NodeHandle nh;

    ros::Subscriber odomSub;
    ros::Subscriber joySub;
    ros::Subscriber cmdSub;
    ros::Subscriber encoderSub;
    ros::Subscriber imuSub;
    ros::Subscriber imuResetSub;

    ros::Publisher odomPub;
    ros::Publisher posePub;
    ros::Publisher posePfPub;
    ros::Publisher poseArrayPub;
    ros::Publisher helperPub;

    tf::TransformBroadcaster br;

    // Robot position estimation
    double xpos, ypos, zpos;
    double heading;
    double lastHeading;
    bool gotFirst;
    std::string posTopicName;

    double imuOffsetYaw;
    double imuDeltaYaw;
    double lastImuYaw;
    double lastTwistZ;

    // Angular velocities
    double omegaImuYaw;
    ros::Time lastImuTime;
    double omegaOdoYaw;
    ros::Time lastOdoTime;

    tf::Quaternion qyaw;
    geometry_msgs::PoseStamped robotPose;
    geometry_msgs::PoseArray estimated_poses;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry lastOdom;

    char movingStatus;

    double dxTot;
    double dyTot;
    double dhTot;
    double lastOdoHeading;

    bool newOdo;
    bool newImu;

    // Joy
    bool waitingForRelease;
    bool joyDisabled;

    bool smoothHeadingMode;

    int onlyOdo;

};

typedef boost::shared_ptr<IMUPosition> IMUPositionPtr;
}

#endif
