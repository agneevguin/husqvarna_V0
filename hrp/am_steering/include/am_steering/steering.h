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
#include <std_msgs/Int16.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <am_driver/SensorStatus.h>
#include <am_driver/Loop.h>
#include <am_driver/ConfinementStatus.h>

namespace Husqvarna
{

typedef struct
{
    int command;
    double speed;
    double distance;
    double angle;
    int collisionState;
} command_t;

class Steering
{
public:
    Steering(const ros::NodeHandle& nodeh);
    ~Steering();

    bool setup();
    bool update(ros::Duration dt);

private:
    void stateIdle(ros::Duration dt);
    void stateMoveTo(ros::Duration dt);
    void stateMoveCte(ros::Duration dt);
    void stateStraight(ros::Duration dt);
    void stateTurnTo(ros::Duration dt);
    void stateSequence(ros::Duration dt);
    void stateTracking(ros::Duration dt);
    void stateWait(ros::Duration dt);
    void stateFollowLoop(ros::Duration dt);

    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void imuEulerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& j);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void statusCallback(const am_driver::SensorStatus::ConstPtr& msg);
    void loopCallback(const am_driver::Loop::ConstPtr& msg);
    void confinementCallback(const am_driver::ConfinementStatus::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

    double calculateTargetHeading();

    // ROS data
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Publisher uwb_cmd_pub;
    ros::Publisher state_pub;

    ros::Subscriber mode_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber imu_euler_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber status_sub;
    ros::Subscriber loop_sub;
    ros::Subscriber confinement_sub;
    ros::Subscriber probmap_sub;
    ros::Subscriber path_sub;
    ros::Subscriber mapcollision_sub;

    ros::Subscriber encoder_sub;

    tf::TransformListener listener;
    tf::TransformBroadcaster br;

    // Copy of last command
    uint16_t nextMode;
    bool newGoal;

    // Copy of last odom position
    nav_msgs::Odometry odom;

    // IMU data
    bool newImuData;
    bool isPaused;
    bool wasPaused;
    double roll, pitch, yaw;

    int state;
    int doneState;
    int breakState;
    int collisionState;

    // Joy
    bool waitingForRelease;
    bool joyDisabled;

    // Navigations stuff
    geometry_msgs::PointStamped target_pos;
    geometry_msgs::PoseStamped nav_goal;
    geometry_msgs::PointStamped start_pos;

    // Straight goal
    double distance;
    double speed;
    bool newOdomData;
    nav_msgs::Odometry previousOdom;
    double odoHeading;

    double wantedHeading;
    double currentHeading;
    double turnAngle;

    double extraOffset;
    double calibrationLength;

    bool statusCollision;
    bool statusOutside;
    bool statusConOutside;

    ros::Duration waitUntil;

    // PID

    int pidTuneMode;

    double err;
    double err_old;
    double P_err;
    double I_err;
    double D_err;

	double maxCTE;
    double Pg;
    double Ig;
    double Dg;

    double Pg_turn;
    double Ig_turn;
    double Dg_turn;

    double Pg_loop;
    double Ig_loop;
    double Dg_loop;

    // Seqence (max 20 commands)
    command_t sequence[20];
    int commandIndex;
    double savedHeading;
    double savedOdoHeading;

    double nextAngle;
    double nextDistance;

    // LOOP data
    bool newLoopData;
    am_driver::Loop loopData;
    bool followRight;
    double wantedLoopSignal;
    double loopSpeed;

    double loopSignalSetPoint;

    // Path planning
    int pathTargetIndex;
    nav_msgs::Path path;

    boost::mutex pathMutex;

    // CTE move-to
    double cte;
    double cte_diff;
    double cte_deriv;
    double cte_last;

    double cte_kp;
    double cte_ki;
    double cte_kd;
    double cte_limit;
    double cte_wantedSpeed;
    bool cte_lastSegment;

    // IMU or ODO?
    bool useImu;

    bool nonStopCte;
};

typedef boost::shared_ptr<Steering> SteeringPtr;
}

#endif
