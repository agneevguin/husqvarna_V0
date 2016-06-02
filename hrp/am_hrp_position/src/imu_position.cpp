/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_hrp_position/imu_position.h"
#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <std_msgs/Float32.h>
#include <am_driver/SlipDetection.h>

using namespace std;

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))
#define NOISE(n) (((rand() % 1000) / 1000.0) * n - n / 2.0)

#define FIX_ANGLES(a)       \
    if (a > M_PI * 2.0)     \
    {                       \
        a = a - M_PI * 2.0; \
    }                       \
    else if (a < 0.0)       \
    {                       \
        a = a + M_PI * 2.0; \
    }

#define IMU_DRIFT_INACTIVE (0)
#define IMU_DRIFT_WAITING (1)
#define IMU_DRIFT_SAMPLE (2)
#define IMU_DRIFT_NOTHING_NEW (100)

namespace Husqvarna
{

IMUPosition::IMUPosition(const ros::NodeHandle& nodeh)
{
    // Init attributes
    nh = nodeh;

    // Setup some ROS stuff
    odomSub = nh.subscribe("odom", 1, &IMUPosition::odomCallback, this);
    joySub = nh.subscribe("nano2", 1, &IMUPosition::joyCallback, this);
    imuSub = nh.subscribe("imu", 1, &IMUPosition::imuCallback, this);
    imuResetSub = nh.subscribe("imu_reset", 1, &IMUPosition::imuResetCallback, this);

    posePub = nh.advertise<geometry_msgs::PoseStamped>("pose_combined", 10);
    poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("estimated_poses", 1);
    helperPub = nh.advertise<visualization_msgs::Marker>("helpers", 25);

    // Initialize the intial pose
    robotPose.pose.position.x = 0.0;
    robotPose.pose.position.y = 0.0;
    robotPose.pose.position.z = 0.0;
    xpos = 0.0;
    ypos = 0.0;
    zpos = 0.0;
    heading = 0.0;
    qyaw = tf::createQuaternionFromYaw(heading);

    dxTot = 0;
    dyTot = 0;
    dhTot = 0;
    lastOdoHeading = 0;
    newOdo = false;
    lastHeading = 0;
    gotFirst = false;
    // Parameters
    ros::NodeHandle n_private("~");

    posTopicName = "odom_combined";
    n_private.param<std::string>("posTopicName", posTopicName, "odom_combined");
    ROS_INFO("Param: posTopicName: %s", posTopicName.c_str());

    odomPub = nh.advertise<nav_msgs::Odometry>(posTopicName, 25);

    // Only use odo?
    onlyOdo = 1;
    n_private.param("onlyOdo", onlyOdo, 1);
    ROS_INFO("Param: onlyOdo: [%d]", onlyOdo);

    waitingForRelease = false;
    joyDisabled = true;

    // Default
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    tf::Quaternion qyaw = tf::createQuaternionFromYaw(0.0);
    odom.pose.pose.orientation.x = qyaw.x();
    odom.pose.pose.orientation.y = qyaw.y();
    odom.pose.pose.orientation.z = qyaw.z();
    odom.pose.pose.orientation.w = qyaw.w();

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    setDefaultValues();

    smoothHeadingMode = false;

    imuOffsetYaw = 0.0;
    imuDeltaYaw = 0.0;
    lastImuYaw = 0.0;
    newImu = false;

    imuOffsetYaw = 0.0;

    lastTwistZ = 0.0;
}

IMUPosition::~IMUPosition()
{
}

void IMUPosition::setDefaultValues(void)
{
    ROS_INFO("Set default values:");

    // Reset this...
    imuOffsetYaw = 0.0;

}

bool IMUPosition::setup()
{
    return true;
}

void IMUPosition::odomCallback(const nav_msgs::Odometry::ConstPtr& o)
{
    // cout << "Odom message: " << o->pose.pose.position.x << ", " << o->pose.pose.position.y << ", " <<
    // o->pose.pose.position.z << endl;

    tf::quaternionMsgToTF(o->pose.pose.orientation, qyaw);
    odom.pose.pose.position.x = o->pose.pose.position.x;
    odom.pose.pose.position.y = o->pose.pose.position.y;
    odom.pose.pose.position.z = o->pose.pose.position.z;
    odom.pose.pose.orientation = o->pose.pose.orientation;
    odom.twist.twist.linear.x = o->twist.twist.linear.x;
    odom.twist.twist.linear.y = o->twist.twist.linear.y;
    odom.twist.twist.angular.z = o->twist.twist.angular.z;

    double dx = odom.pose.pose.position.x - lastOdom.pose.pose.position.x;
    double dy = odom.pose.pose.position.y - lastOdom.pose.pose.position.y;

    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    double odoHeading = tf::getYaw(pose.getRotation());

    double dh = odoHeading - lastOdoHeading;

    if (dh > M_PI)
    {
        dh = dh - M_PI * 2.0;
    }
    else if (dh < -M_PI)
    {
        dh = dh + M_PI * 2.0;
    }

    if (gotFirst == true)
    {
        dxTot = dxTot + dx;
        dyTot = dyTot + dy;
        dhTot = dhTot + dh;
    }
    else
    { // Avoid using junk values in lastOdom, start from zero.
        gotFirst = true;
    }
    lastOdom = odom;
    lastOdoHeading = odoHeading;

    // Save for detetction of drive straight
    lastTwistZ = odom.twist.twist.angular.z;

    // Calculate the angular velocity
    ros::Time now = ros::Time::now();
    ros::Duration dOdoTime = now - lastOdoTime;
    lastOdoTime = now;
    omegaOdoYaw = dh / dOdoTime.toSec();

    // Check moving status
    if (odom.twist.twist.linear.x > 0)
    {
        // FORWARD
        movingStatus = 1;
        // std::cout << "F!" << std::endl;
    }
    else if (odom.twist.twist.linear.x < 0)
    {
        // BACKWARD
        movingStatus = -1;
        // std::cout << "B!" << std::endl;
    }
    else if (odom.twist.twist.angular.z == 0)
    {
        // STAND STILL
        movingStatus = 0;
        // std::cout << "---" << std::endl;
    }
    else
    {
        // TURNING on SPOT (i.e. not forward)
        movingStatus = -1;
        // std::cout << "-T-" << std::endl;
    }

    newOdo = true;
}

void IMUPosition::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    /* If data is received, use it (default odo only is assumed) */
    onlyOdo = 0;
    tf::Quaternion q;
    double r, p, y;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(r, p, y);

    double imuYaw = y;

    FIX_ANGLES(imuYaw);

    // Build the deltas...
    imuDeltaYaw += lastImuYaw - imuYaw;
    FIX_ANGLES(imuDeltaYaw);

    // Calculate the angular velocity
    ros::Time now = ros::Time::now();
    ros::Duration dImuTime = now - lastImuTime;
    lastImuTime = now;
    double dImu = imuYaw - lastImuYaw;

    if (dImu > M_PI)
    {
        dImu = dImu - M_PI * 2.0;
    }
    else if (dImu < -M_PI)
    {
        dImu = dImu + M_PI * 2.0;
    }

    omegaImuYaw = dImu / dImuTime.toSec();

    lastImuYaw = imuYaw;

    newImu = true;
}

void IMUPosition::imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

    ROS_INFO("IMUPosition::imuResetCallback! (%f, %f)",msg->position.x,msg->position.y);
    // Set heading from orientation
    tf::Quaternion q;
    double r, p, y;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(r, p, y);
    heading = y;

    // Set pose to the IMU Positions
    xpos = msg->position.x;
    ypos = msg->position.y;
    zpos = msg->position.z;
}

void IMUPosition::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
{
    // Special case to handle key release event...
    if (waitingForRelease)
    {
        waitingForRelease = false;
        return;
    }

    // Need to "enable" this before use of params...
    if (joyDisabled)
    {
        // Pressing R[1]
        if (j->buttons[25 + 16 + 1] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("IMUPOS::NANO KONTROL enabled!");
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing R[1]
        if (j->buttons[25 + 16 + 1] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("IMUPOS::NANO KONTROL disabled!");
            return;
        }
    }

    // Pressing M[1]
    if (j->buttons[25 + 8 + 1] == 1)
    {
        if (onlyOdo == true)
        {
            onlyOdo = false;
        }
        else
        {
            onlyOdo = true;
        }
        ROS_INFO("IMUPOS::NANO KONTROL onlyOdo = %d", onlyOdo);
        waitingForRelease = true;
    }
}

bool IMUPosition::update(ros::Duration dt)
{
    ros::Time currentTime = ros::Time::now();

    stateEstimatePosition(currentTime);
    publishData(dt, currentTime);

    return true;
}

bool IMUPosition::stateEstimatePosition(ros::Time currentTime)
{

    ////////////////////////////////////////////////////////////
    // ODOMETRY/GYRO ESTIMATION
    ////////////////////////////////////////////////////////////

    double distance = 0;
    double deltaHeading = 0;
    double xdist;
    double ydist;

    // Use the heading from the gyro
    if (newImu)
    {
        if (movingStatus != 0)
        {
            // std::cout << "New IMU!" << std::endl;
            heading = lastImuYaw + imuOffsetYaw;
            imuDeltaYaw = 0.0;
            FIX_ANGLES(heading);
        }
        else
        {
            // STANDSTILL.

            // Calculate the new heading
            double expectedNewHeading = lastImuYaw + imuOffsetYaw;

            FIX_ANGLES(expectedNewHeading);

            // Save the new drifted offset so that we have same heading
            // as when we moved around...
            imuOffsetYaw -= expectedNewHeading - heading;

            FIX_ANGLES(imuOffsetYaw);

            // std::cout << "imuOffsetYaw=" << imuOffsetYaw*180.0/M_PI << std::endl;
        }

        newImu = false;
    }

    if (newOdo)
    {
        newOdo = false;

        if (onlyOdo == 1)
        {
            // Override the IMU heading with IMU heading
            // for this special test mode.

            heading = lastOdoHeading;

            FIX_ANGLES(heading);
        }

        // Re-calculate dx and dy in the "robot heading"
        // distance comes from "wheels" and that is in another
        // co-ordinate system
        distance = sqrt(dxTot * dxTot + dyTot * dyTot);

        if (movingStatus == 1)
        {
            // Forward
            xdist = distance * cos(heading);
            ydist = distance * sin(heading);
        }
        else
        {
            // Backward
            xdist = distance * cos(heading + M_PI);
            ydist = distance * sin(heading + M_PI);
        }

        xpos = xpos + xdist;
        ypos = ypos + ydist;

        dxTot = 0.0;
        dyTot = 0.0;
        dhTot = 0.0;
    }
    // std::cout << "Pos=(" << xpos << ", " << ypos  << ") h=" << heading*180.0/M_PI << " imuO=" <<
    // imuOffsetYaw*180.0/M_PI << std::endl;

    return true;
}

bool IMUPosition::publishData(ros::Duration dt, ros::Time currentTime)
{
    ////////////////////////////////////////////////////////////
    // PUBLISH POSE (ROBOT)
    ////////////////////////////////////////////////////////////

    // Set this into the pose
    robotPose.pose.position.x = xpos;
    robotPose.pose.position.y = ypos;
    robotPose.pose.position.z = 0.0;

    tf::Quaternion qyaw = tf::createQuaternionFromYaw(heading);

    robotPose.pose.orientation.x = qyaw.x();
    robotPose.pose.orientation.y = qyaw.y();
    robotPose.pose.orientation.z = qyaw.z();
    robotPose.pose.orientation.w = qyaw.w();

    // Publish the pose
    posePub.publish(robotPose);

    ////////////////////////////////////////////////////////////
    // PUBLISH ODO POSITION/HEADING
    ////////////////////////////////////////////////////////////

    visualization_msgs::Marker odoPos;
    // Set the frame ID and timestamp.
    odoPos.header.frame_id = "/odom_combined";
    odoPos.header.stamp = currentTime;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    odoPos.ns = "robot_odopos";
    odoPos.id = 3;

    odoPos.type = visualization_msgs::Marker::MESH_RESOURCE;
    odoPos.mesh_resource = "package://am_description/urdf/am_chassis.stl";

    // Set the marker action.  Options are ADD and DELETE
    odoPos.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    odoPos.pose.position.x = odom.pose.pose.position.x;
    odoPos.pose.position.y = odom.pose.pose.position.y;
    odoPos.pose.position.z = 0.0;
    odoPos.pose.orientation = tf::createQuaternionMsgFromYaw(lastOdoHeading + M_PI / 2.0);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    odoPos.scale.x = 0.001;
    odoPos.scale.y = 0.001;
    odoPos.scale.z = 0.001;

    // Set the color -- be sure to set alpha to something non-zero!
    odoPos.color.r = 1.0f;
    odoPos.color.g = 0.0f;
    odoPos.color.b = 1.0f;
    odoPos.color.a = 0.5;

    helperPub.publish(odoPos);

    ////////////////////////////////////////////////////////////
    // PUBLISH IMU POSITION/HEADING
    ////////////////////////////////////////////////////////////

    visualization_msgs::Marker imuPos;
    // Set the frame ID and timestamp.
    imuPos.header.frame_id = "/odom_combined";
    imuPos.header.stamp = currentTime;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    imuPos.ns = "robot_imupos";
    imuPos.id = 3;

    imuPos.type = visualization_msgs::Marker::MESH_RESOURCE;
    imuPos.mesh_resource = "package://am_description/urdf/am_chassis.stl";

    // Set the marker action.  Options are ADD and DELETE
    imuPos.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    imuPos.pose.position.x = xpos;
    imuPos.pose.position.y = ypos;
    imuPos.pose.position.z = 0.0;

    imuPos.pose.orientation = tf::createQuaternionMsgFromYaw(heading + M_PI / 2.0);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    imuPos.scale.x = 0.001;
    imuPos.scale.y = 0.001;
    imuPos.scale.z = 0.001;

    // Set the color -- be sure to set alpha to something non-zero!
    imuPos.color.r = 1.0f;
    imuPos.color.g = 0.0f;
    imuPos.color.b = 0.0f;
    imuPos.color.a = 0.5;

    helperPub.publish(imuPos);
    ////////////////////////////////////////////////////////////
    // PUBLISH ODOM
    ////////////////////////////////////////////////////////////

    // Odometry message over ROS
    nav_msgs::Odometry pubOdom;
    pubOdom.header.stamp = currentTime;
    pubOdom.header.frame_id = posTopicName;
    pubOdom.child_frame_id = "base_link";

    // Set the position
    pubOdom.pose.pose.position.x = robotPose.pose.position.x;
    pubOdom.pose.pose.position.y = robotPose.pose.position.y;
    pubOdom.pose.pose.position.z = robotPose.pose.position.z;
    pubOdom.pose.pose.orientation = robotPose.pose.orientation;

    // Velocities are not changed...
    pubOdom.twist = odom.twist;

    // Publish the message
    odomPub.publish(pubOdom);

    ////////////////////////////////////////////////////////////
    // PUBLISH TF
    ////////////////////////////////////////////////////////////

    // Calculate the TF from the pose...
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(robotPose.pose.position.x, robotPose.pose.position.y, robotPose.pose.position.z));
    tf::Quaternion q;
    tf::quaternionMsgToTF(robotPose.pose.orientation, q);
    transform.setRotation(q);

    // Send the TF if publishing to odom combined
    if (posTopicName == "odom_combined")
    {
        br.sendTransform(tf::StampedTransform(transform, currentTime, "odom_combined", "base_link"));
    }

    return true;
}
}
