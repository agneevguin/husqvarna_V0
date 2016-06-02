/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_steering/steering.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <boost/foreach.hpp>

namespace Husqvarna
{

// States
#define AM_STEER_S_NONE (-1)
#define AM_STEER_S_IDLE (0)
#define AM_STEER_S_MOVE_TO (1)
#define AM_STEER_S_STRAIGHT (2)
#define AM_STEER_S_TURN (3)
#define AM_STEER_S_SEQUENCE (4)
#define AM_STEER_S_TRACKING (5)
#define AM_STEER_S_WAIT (6)
#define AM_STEER_S_FOLLOW_LOOP (7)
#define AM_STEER_S_MOVE_TO_CTE (8)

#define AM_CMD_NONE (0)
#define AM_CMD_STRAIGHT (1)
#define AM_CMD_TURN (2)
#define AM_CMD_SAVE_HEADING (3)
#define AM_CMD_REPEAT (4)
#define AM_CMD_START_POS_ESTIMATION (5)
#define AM_CMD_SAVE_POS_ESTIMATION (6)
#define AM_CMD_END_POS_ESTIMATION (7)
#define AM_CMD_WAIT (8)
#define AM_CMD_RANDOMIZE_ANGLE (9)
#define AM_CMD_MOVE_TO (10)
#define AM_CMD_NEXT_TARGET (11)
#define AM_CMD_CALCULATE_TURN_TO_TARGET (12)
#define AM_CMD_START_SAMPLE_POS_ESTIMATION (13)
#define AM_CMD_MOVE_TO_CTE (14)



#define FIX_ANGLES(a)       \
    if (a > M_PI * 2.0)     \
    {                       \
        a = a - M_PI * 2.0; \
    }                       \
    else if (a < 0.0)       \
    {                       \
        a = a + M_PI * 2.0; \
    }

#define HVA_UWB_CMD_NONE (0)
#define HVA_UWB_CMD_START_POS_ESTIMATION (1)
#define HVA_UWB_CMD_SAVE_POS_ESTIMATION (2)
#define HVA_UWB_CMD_END_POS_ESTIMATION (3)
#define HVA_UWB_CMD_START_SAMPLE_POS_ESTIMATION (4)
#define AM_CMD_PAUSE (15)
#define AM_CMD_RESUME (16)

#define pidTuneFirst 0
#define pidCte 1
#define pidStraight 2
#define pidLoop 3
#define pidTurn 4
#define pidTuneLast 5

Steering::Steering(const ros::NodeHandle& nodeh)
{
    // Init attributes
    this->nh = nodeh;

    // Setup some ROS stuff
    mode_sub = nh.subscribe("cmd_mode", 5, &Steering::modeCallback, this);
    odom_sub = nh.subscribe("odom_combined", 1, &Steering::odomCallback, this);
    goal_sub = nh.subscribe("move_base_simple/goal", 1, &Steering::goalCallback, this);
    // imu_euler_sub = nh.subscribe("imu_euler", 1, &Steering::imuEulerCallback, this);
    imu_sub = nh.subscribe("imu", 1, &Steering::imuCallback, this);
    joy_sub = nh.subscribe("nano2", 1, &Steering::joyCallback, this);
    status_sub = nh.subscribe("sensor_status", 1, &Steering::statusCallback, this);
    loop_sub = nh.subscribe("loop", 1, &Steering::loopCallback, this);

    // Collision hooks to same callback...
    confinement_sub = nh.subscribe("confinement", 1, &Steering::confinementCallback, this);
    mapcollision_sub = nh.subscribe("map_collision", 1, &Steering::confinementCallback, this);

    path_sub = nh.subscribe("coverage_plan", 1, &Steering::pathCallback, this);

    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    uwb_cmd_pub = nh.advertise<std_msgs::UInt16>("beacon_cmd", 1);

    state_pub = nh.advertise<std_msgs::Int16>("steering_state", 1);

    // Parameters
    ros::NodeHandle n_private("~");

    /*	double def_stdDev = 1.0;
            n_private.param("standard_deviation", stdDev, def_stdDev);
            ROS_INFO("Standard deviation: %f", stdDev);

            double def_offset = 0.2;
            n_private.param("offset", offset, def_offset);
            ROS_INFO("Offset: %f", offset);

            double def_scaleFactor = 0.8;
            n_private.param("scale_factor", scaleFactor, def_scaleFactor);
            ROS_INFO("Scale factor: %f", scaleFactor);
    */

    state = AM_STEER_S_IDLE;
    doneState = AM_STEER_S_IDLE;

    newGoal = false;
    newOdomData = false;
    newImuData = false;

    isPaused = false;
    wasPaused = false;
    speed = 0.3;

    double def_Pg = 2.8;
    n_private.param("Pg", Pg, def_Pg);
    ROS_INFO("Pg: %f", Pg);

    double def_Ig = 0.0;
    n_private.param("Ig", Ig, def_Ig);
    ROS_INFO("Ig: %f", Ig);

    double def_Dg = 0.1;
    n_private.param("Dg", Dg, def_Dg);
    ROS_INFO("Dg: %f", Dg);

    Pg_turn = 2.283465;
    Ig_turn = 0.0;
    Dg_turn = 2.763779;

    loopSpeed = 0.28;
    Pg_loop = 0.001102;
    Ig_loop = 0.0;
    Dg_loop = 0.0;

    waitingForRelease = false;
    joyDisabled = true;

	maxCTE = 1.0;
    extraOffset = 0.0;
    calibrationLength = 1.0;

    breakState = AM_STEER_S_IDLE;
    collisionState = AM_STEER_S_NONE;

    statusOutside = false;
    statusConOutside = false;
    statusCollision = false;

    newLoopData = false;

    loopSignalSetPoint = 13700;

    nextAngle = 0.0;

    pathTargetIndex = 0;

    nextDistance = 0.0;

    cte_diff = 0.0;
    cte_deriv = 0.0;
    cte = 0.0;
    cte_last = 0.0;

    cte_kp = 22.83;
    cte_ki = 0.0157;
    cte_kd = 5.35;
    cte_limit = 180.0;

    bool def_useImu = false;
    useImu = false;
    n_private.param("useImu", useImu, def_useImu);
    ROS_INFO("UseImu: %d", useImu);

    nonStopCte = false;
}

Steering::~Steering()
{
}

void Steering::modeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    
    if (msg->data == 0x50)  //Pause
    {
        ROS_INFO("Steering::Pause");
        isPaused = true;
        wasPaused = false;
    }
    else if (msg->data == 0x51) //Resume
    {
        ROS_INFO("Steering::Resume");
        isPaused = false;
        wasPaused = false;
    }
    else if (msg->data < 0x90)  // DO not listen on "larger commands"
    {
        nextMode = msg->data;
        ROS_INFO("Steering::cmd_mode: %d", msg->data);
    }
}

void Steering::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    nav_goal = *msg;
    newGoal = true;
    ROS_INFO("Steering::New navigation goal!");
}

void Steering::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (useImu)
    {
        // This is the real sensor data...convert to "euler"
        // as this is the only thing we are using in this node

        double r, p, y;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->orientation, q);
        tf::Matrix3x3(q).getRPY(r, p, y);

        roll = r;
        pitch = p;
        yaw = y;

        // Convert to 0...360 (always!)
        if (yaw < 0)
        {
            yaw = 2.0 * M_PI + yaw;
        }

        newImuData = true;
    }
}

void Steering::imuEulerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    if (useImu)
    {
        // Get the ROLL PITCH YAW from the IMU
        roll = msg->vector.x;
        pitch = msg->vector.y;

        yaw = msg->vector.z;

        // Convert to 0...360 (always!)
        if (yaw < 0)
        {
            yaw = 2.0 * M_PI + yaw;
        }

        newImuData = true;
    }
}

void Steering::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Copy & save for use in update
    odom = *msg;

    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    odoHeading = tf::getYaw(pose.getRotation());

    newOdomData = true;

    if (!useImu)
    {
        /* Use odom_imu as IMU, if IMU is unavailable this will contain odom heading */
        newImuData = true;
        yaw = odoHeading;
    }
}

void Steering::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
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
        // Pressing S[2]
        if (j->buttons[25 + 2] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("Steering::NANO KONTROL enabled!");
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing S[2]
        if (j->buttons[25 + 2] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("Steering::NANO KONTROL disabled!");
            return;
        }
    }

    // TRACK RIGHT
    if (j->buttons[10] == 1)
    {
        // Start loop follow
        nextMode = 0x15;

        // The idle state will change to correct state
        state = AM_STEER_S_IDLE;
        doneState = AM_STEER_S_IDLE;

        if (loopSignalSetPoint == 0)
        {
            wantedLoopSignal = loopData.frontCenter;
        }
        else
        {
            wantedLoopSignal = loopSignalSetPoint;
        }
        ROS_INFO("Wanted Loop Level:%f", wantedLoopSignal);

        waitingForRelease = true;

        ROS_INFO("STEERING - FollowLoop RIGHT");
    }
    // TRACK LEFT
    else if (j->buttons[9] == 1)
    {
        // Start loop follow
        nextMode = 0x14;

        // The idle state will change to correct state
        state = AM_STEER_S_IDLE;
        doneState = AM_STEER_S_IDLE;

        if (loopSignalSetPoint == 0)
        {
            wantedLoopSignal = loopData.frontCenter;
        }
        else
        {
            wantedLoopSignal = loopSignalSetPoint;
        }
        ROS_INFO("Wanted Loop Level:%f", wantedLoopSignal);

        waitingForRelease = true;
        ROS_INFO("STEERING - FollowLoop LEFT");
    }
    // STOP
    else if (j->buttons[6] == 1)
    {
        // Stop all we are doing...
        nextMode = 0x0;

        geometry_msgs::Twist vel;

        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;

        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;

        cmd_pub.publish(vel);

        state = AM_STEER_S_IDLE;
        doneState = AM_STEER_S_IDLE;

        waitingForRelease = true;

        ROS_INFO("STEERING - STOP!!!");
    }
    /* Marker Right */
    else if (j->buttons[2] == 1)
    {
        pidTuneMode++;
        if (pidTuneMode >= pidTuneLast)
        {
            pidTuneMode = pidTuneFirst + 1;
        }
        std::cout << "pidTuneMode: " << pidTuneMode << ", [cte=1, straight, loop, turn]" << std::endl;
    }
    /* Marker Left */
    else if (j->buttons[3] == 1)
    {
        pidTuneMode--;
        if (pidTuneMode <= pidTuneFirst)
        {
            pidTuneMode = pidTuneLast - 1;
        }
        std::cout << "pidTuneMode: " << pidTuneMode << ", [cte=1, straight, loop, turn]" << std::endl;
    }
    /* cycle */
    else if (j->buttons[0] == 1)
    {
        if (isPaused)
        {
            isPaused = false;
            std::cout << "Steering Resume" << std::endl;
        }
        else
        {
            isPaused = true;
            std::cout << "Steering Pause" << std::endl;
        }
        
        wasPaused = false;
       
    }
    else
    {
        double l = j->axes[4] * 360.0;

        if (pidTuneMode == pidCte)
        {
            double p = j->axes[5] * 100.0;
            double i = j->axes[6] * 1.0;
            double d = j->axes[7] * 10.00;

            if (p != 0)
            {
                cte_kp = p;
                ROS_INFO("Steering::cte_kp = %f", cte_kp);
            }

            if (i != 0)
            {
                cte_ki = i;
                ROS_INFO("Steering::cte_ki = %f", cte_ki);
            }

            if (d != 0)
            {
                cte_kd = d;
                ROS_INFO("Steering::cte_kd = %f", cte_kd);
            }
        }
        else if (pidTuneMode == pidStraight)
        {
            double p = j->axes[5] * 100.0;
            double i = j->axes[6] * 1.0;
            double d = j->axes[7] * 10.00;

            if (p != 0)
            {
                Pg = p;
                ROS_INFO("Steering::Pg = %f", Pg);
            }

            if (i != 0)
            {
                Ig = i;
                ROS_INFO("Steering::Ig = %f", Ig);
            }

            if (d != 0)
            {
                Dg = d;
                ROS_INFO("Steering::Dg = %f", Dg);
            }
        }
        else if (pidTuneMode == pidTurn)
        {
            double p = j->axes[5] * 100.0;
            double i = j->axes[6] * 1.0;
            double d = j->axes[7] * 10.00;

            if (p != 0)
            {
                Pg_turn = p;
                ROS_INFO("Steering::Pg_turn = %f", Pg_turn);
            }

            if (i != 0)
            {
                Ig_turn = i;
                ROS_INFO("Steering::Ig_turn = %f", Ig_turn);
            }

            if (d != 0)
            {
                Dg_turn = d;
                ROS_INFO("Steering::Dg_turn = %f", Dg_turn);
            }
        }
        else if (pidTuneMode == pidLoop)
        {
            double p = j->axes[5] * 100.0;
            double i = j->axes[6] * 1.0;
            double d = j->axes[7] * 10.00;

            if (p != 0)
            {
                Pg_loop = p;
                ROS_INFO("Steering::Pg_loop = %f", Pg_loop);
            }

            if (i != 0)
            {
                Ig_loop = i;
                ROS_INFO("Steering::Ig_loop = %f", Ig_loop);
            }

            if (d != 0)
            {
                Dg_loop = d;
                ROS_INFO("Steering::Dg_loop = %f", Dg_loop);
            }
        }
        if (l != 0)
        {
            cte_limit = l;
            ROS_INFO("Steering::cte_limit = %f", cte_limit);
        }

        // Loop follow speed
        double s = j->axes[0] * 0.5;
        if (s != 0)
        {
            loopSpeed = s;
            ROS_INFO("Steering::Loop Speed = %f", loopSpeed);
        }

        double sp = 10000 + j->axes[1] * 5000;
        if (sp != 10000)
        {
            loopSignalSetPoint = sp;
            ROS_INFO("Steering::New setpoint at = %f", loopSignalSetPoint);
        }
        else
        {
            loopSignalSetPoint = 0;
        }

        // Control the "length" for calibration sequence
        double calLength = j->axes[2] * 10.0;
        if (calLength != 0)
        {
            calibrationLength = calLength;
            ROS_INFO("Steering::Calibration Length = %f", calibrationLength);
        }
    }
}

void Steering::statusCallback(const am_driver::SensorStatus::ConstPtr& msg)
{
    // Sensor status
    // std::cout << "Sensor Status:" << msg->sensorStatus << std::endl;

    // 0x40 - In Charging Station
    // 0x04 - Collision
    // 0x02 - Out of area

    statusCollision = (msg->sensorStatus & 0x04) | (msg->sensorStatus & 0x40);
    statusOutside = (msg->sensorStatus & 0x02);

    // std::cout << "statusCollision:" << statusCollision << std::endl;
    // std::cout << "statusOutside:" << statusOutside << std::endl;
}

void Steering::loopCallback(const am_driver::Loop::ConstPtr& msg)
{
    newLoopData = true;
    loopData = *msg;
}

void Steering::confinementCallback(const am_driver::ConfinementStatus::ConstPtr& msg)
{
    // Super simple collision with the confinement data
    if ((msg->frontLeftCollision < 0) || (msg->frontRightCollision < 0) || (msg->rearLeftCollision < 0) ||
        (msg->rearRightCollision < 0))
    {
        statusConOutside = true;
    }
    else
    {
        statusConOutside = 0;
    }
}

void Steering::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    // Copy & save
    // ROS_INFO("Copying path");
    pathMutex.lock();
    path = *msg;
    pathMutex.unlock();
}

bool Steering::setup()
{
    ROS_INFO("Steering::setup()");

    return true;
}

void Steering::stateIdle(ros::Duration dt)
{
    // Something to do?

    if (nextMode == 0x11)
    {
        //
        // Set a target in front of us...
        //
        nextMode = 0;

        // TEST!
        // Find a new position some distance (2.0m) forward

        geometry_msgs::PointStamped fwd_point;
        fwd_point.header.frame_id = "base_link";
        fwd_point.header.stamp = ros::Time();
        fwd_point.point.x = 3.0;
        fwd_point.point.y = 0.0;
        fwd_point.point.z = 0.0;

        try
        {
            listener.transformPoint("odom_combined", fwd_point, target_pos);

            ROS_INFO("fwd_point: (%.2f, %.2f. %.2f) -----> target_point: (%.2f, %.2f, %.2f) at time %.2f",
                     fwd_point.point.x,
                     fwd_point.point.y,
                     fwd_point.point.z,
                     target_pos.point.x,
                     target_pos.point.y,
                     target_pos.point.z,
                     target_pos.header.stamp.toSec());
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"odom_combined\": %s",
                      ex.what());
        }

        state = AM_STEER_S_MOVE_TO;
        doneState = AM_STEER_S_IDLE;
    }
    else if (nextMode == 0x10)
    {
        nextMode = 0x00;

        // Set a "distance" and a "drive straight" goal...
        distance = 2.0;
        speed = 0.3;
        previousOdom = odom;

        wantedHeading = yaw;

        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        state = AM_STEER_S_STRAIGHT;
        doneState = AM_STEER_S_IDLE;
    }
    else if (nextMode == 0x13)
    {
        nextMode = 0x00;

        // Turn 90 degrees
        turnAngle = M_PI / 2.0;
        wantedHeading = yaw + turnAngle;
        if (wantedHeading > 2.0 * M_PI)
        {
            wantedHeading = wantedHeading - 2.0 * M_PI;
        }
        if (wantedHeading < 0)
        {
            wantedHeading = wantedHeading + 2.0 * M_PI;
        }

        std::cout << "Turn from:" << yaw * 180.0 / M_PI << " to:" << wantedHeading * 180.0 / M_PI << std::endl;

        // Reset PID
        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        state = AM_STEER_S_TURN;
        doneState = AM_STEER_S_IDLE;
    }
    else if (nextMode == 0x12)
    {
        nextMode = 0x00;

        // Turn 90 degrees
        turnAngle = -M_PI / 2.0;
        wantedHeading = yaw + turnAngle;
        if (wantedHeading > 2.0 * M_PI)
        {
            wantedHeading = wantedHeading - 2.0 * M_PI;
        }
        if (wantedHeading < 0)
        {
            wantedHeading = wantedHeading + 2.0 * M_PI;
        }

        std::cout << "Turn from:" << yaw * 180.0 / M_PI << " to:" << wantedHeading * 180.0 / M_PI << std::endl;

        // Reset PID
        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        state = AM_STEER_S_TURN;
        doneState = AM_STEER_S_IDLE;
    }
    else if (nextMode == 0x14)
    {
        ROS_INFO("STEERING - FollowLoop commanded LEFT");
        nextMode = 0x00;

        distance = 2.0;
        previousOdom = odom;

        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        followRight = false;

        state = AM_STEER_S_FOLLOW_LOOP;
        doneState = AM_STEER_S_IDLE;
    }
    else if (nextMode == 0x15)
    {
        ROS_INFO("STEERING - FollowLoop commanded RIGHT");
        nextMode = 0x00;

        distance = 2.0;
        previousOdom = odom;

        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        followRight = true;

        state = AM_STEER_S_FOLLOW_LOOP;
        doneState = AM_STEER_S_IDLE;
    }
    else if (nextMode == 0x20)
    {
        nextMode = 0;

        double laneSeparation = 0.20;
        double laneLength = 4.0;

        // Build a simple sequence and start executing that...
        sequence[0].command = AM_CMD_SAVE_HEADING;
        sequence[0].speed = 0.0;
        sequence[0].distance = 0.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_IDLE;

        // FORWARD

        sequence[1].command = AM_CMD_STRAIGHT;
        sequence[1].speed = 0.3;
        sequence[1].distance = laneLength;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_IDLE;

        // CHANGE LANE

        sequence[2].command = AM_CMD_TURN;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = -90.0;
        sequence[2].collisionState = AM_STEER_S_IDLE;

        sequence[3].command = AM_CMD_STRAIGHT;
        sequence[3].speed = 0.2;
        sequence[3].distance = laneSeparation;
        sequence[3].angle = -90.0;
        sequence[3].collisionState = AM_STEER_S_IDLE;

        sequence[4].command = AM_CMD_TURN;
        sequence[4].speed = 0.0;
        sequence[4].distance = 0.0;
        sequence[4].angle = -180.0;
        sequence[4].collisionState = AM_STEER_S_IDLE;

        // BACKWARD

        sequence[5].command = AM_CMD_STRAIGHT;
        sequence[5].speed = 0.3;
        sequence[5].distance = laneLength;
        sequence[5].angle = -180.0;
        sequence[5].collisionState = AM_STEER_S_IDLE;

        // CHANGE LANE

        sequence[6].command = AM_CMD_TURN;
        sequence[6].speed = 0.0;
        sequence[6].distance = 0.0;
        sequence[6].angle = -90.0;
        sequence[6].collisionState = AM_STEER_S_IDLE;

        sequence[7].command = AM_CMD_STRAIGHT;
        sequence[7].speed = 0.2;
        sequence[7].distance = laneSeparation;
        sequence[7].angle = -90.0;
        sequence[7].collisionState = AM_STEER_S_IDLE;

        sequence[8].command = AM_CMD_TURN;
        sequence[8].speed = 0.0;
        sequence[8].distance = 0.0;
        sequence[8].angle = 0.0;
        sequence[8].collisionState = AM_STEER_S_IDLE;

        // FORWARD

        sequence[9].command = AM_CMD_STRAIGHT;
        sequence[9].speed = 0.3;
        sequence[9].distance = laneLength;
        sequence[9].angle = 0.0;
        sequence[9].collisionState = AM_STEER_S_IDLE;

        // CHANGE LANE

        sequence[10].command = AM_CMD_TURN;
        sequence[10].speed = 0.0;
        sequence[10].distance = 0.0;
        sequence[10].angle = -90.0;
        sequence[10].collisionState = AM_STEER_S_IDLE;

        sequence[11].command = AM_CMD_STRAIGHT;
        sequence[11].speed = 0.2;
        sequence[11].distance = laneSeparation;
        sequence[11].angle = -90.0;
        sequence[11].collisionState = AM_STEER_S_IDLE;

        sequence[12].command = AM_CMD_TURN;
        sequence[12].speed = 0.0;
        sequence[12].distance = 0.0;
        sequence[12].angle = -180.0;
        sequence[12].collisionState = AM_STEER_S_IDLE;

        // BACKWARD

        sequence[13].command = AM_CMD_STRAIGHT;
        sequence[13].speed = 0.3;
        sequence[13].distance = laneLength;
        sequence[13].angle = -180.0;
        sequence[13].collisionState = AM_STEER_S_IDLE;

        // CHANGE LANE

        sequence[14].command = AM_CMD_TURN;
        sequence[14].speed = 0.0;
        sequence[14].distance = 0.0;
        sequence[14].angle = -90.0;
        sequence[14].collisionState = AM_STEER_S_IDLE;

        sequence[15].command = AM_CMD_STRAIGHT;
        sequence[15].speed = 0.2;
        sequence[15].distance = laneSeparation;
        sequence[15].angle = -90.0;
        sequence[15].collisionState = AM_STEER_S_IDLE;

        sequence[16].command = AM_CMD_TURN;
        sequence[16].speed = 0.0;
        sequence[16].distance = 0.0;
        sequence[16].angle = 0.0;
        sequence[16].collisionState = AM_STEER_S_IDLE;

        // DONE

        sequence[17].command = AM_CMD_NONE;
        sequence[17].speed = 0.0;
        sequence[17].distance = 0.0;
        sequence[17].angle = 0.0;
        sequence[17].collisionState = AM_STEER_S_IDLE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x21)
    {
        nextMode = 0;

        double laneSeparation = 0.12;
        double laneLength = 10.0;

        // Save the heading
        savedHeading = yaw;
        savedOdoHeading = odoHeading;
        std::cout << "SAVED HEADING: " << savedHeading * 180.0 / M_PI << " ODOH: " << savedOdoHeading * 180.0 / M_PI
                  << std::endl;

        // FORWARD
        sequence[0].command = AM_CMD_STRAIGHT;
        sequence[0].speed = 0.3;
        sequence[0].distance = laneLength;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_SEQUENCE;

        // BACK A LITTLE BEFORE CHANGE LANE
        sequence[1].command = AM_CMD_STRAIGHT;
        sequence[1].speed = -0.3;
        sequence[1].distance = 0.3;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_NONE;

        // CHANGE LANE
        sequence[2].command = AM_CMD_TURN;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = -90.0;
        sequence[2].collisionState = AM_STEER_S_IDLE;

        sequence[3].command = AM_CMD_STRAIGHT;
        sequence[3].speed = 0.2;
        sequence[3].distance = laneSeparation;
        sequence[3].angle = -90.0;
        sequence[3].collisionState = AM_STEER_S_IDLE;

        sequence[4].command = AM_CMD_TURN;
        sequence[4].speed = 0.0;
        sequence[4].distance = 0.0;
        sequence[4].angle = -180.0;
        sequence[4].collisionState = AM_STEER_S_IDLE;

        // DRIVE OTHER DIRECTION
        sequence[5].command = AM_CMD_STRAIGHT;
        sequence[5].speed = 0.3;
        sequence[5].distance = laneLength;
        sequence[5].angle = -180.0;
        sequence[5].collisionState = AM_STEER_S_SEQUENCE;

        // BACK A LITTLE BEFORE CHANGE LANE
        sequence[6].command = AM_CMD_STRAIGHT;
        sequence[6].speed = -0.3;
        sequence[6].distance = 0.3;
        sequence[6].angle = -180.0;
        sequence[6].collisionState = AM_STEER_S_NONE;

        // CHANGE LANE
        sequence[7].command = AM_CMD_TURN;
        sequence[7].speed = 0.0;
        sequence[7].distance = 0.0;
        sequence[7].angle = -90.0;
        sequence[7].collisionState = AM_STEER_S_IDLE;

        sequence[8].command = AM_CMD_STRAIGHT;
        sequence[8].speed = 0.2;
        sequence[8].distance = laneSeparation;
        sequence[8].angle = -90.0;
        sequence[8].collisionState = AM_STEER_S_IDLE;

        sequence[9].command = AM_CMD_TURN;
        sequence[9].speed = 0.0;
        sequence[9].distance = 0.0;
        sequence[9].angle = 0.0;
        sequence[9].collisionState = AM_STEER_S_IDLE;

        // REPEAT
        sequence[10].command = AM_CMD_REPEAT;
        sequence[10].speed = 0.0;
        sequence[10].distance = 0.0;
        sequence[10].angle = 0.0;
        sequence[10].collisionState = AM_STEER_S_IDLE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x22)
    {
        nextMode = 0;

        sequence[0].command = AM_CMD_SAVE_HEADING;
        sequence[0].speed = 0.0;
        sequence[0].distance = 0.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_IDLE;

        sequence[1].command = AM_CMD_TURN;
        sequence[1].speed = 0.0;
        sequence[1].distance = 0.0;
        sequence[1].angle = -180.0;
        sequence[1].collisionState = AM_STEER_S_IDLE;

        sequence[2].command = AM_CMD_TURN;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = -0.0;
        sequence[2].collisionState = AM_STEER_S_IDLE;

        sequence[3].command = AM_CMD_NONE;
        sequence[3].speed = 0.0;
        sequence[3].distance = 0.0;
        sequence[3].angle = 0.0;
        sequence[3].collisionState = AM_STEER_S_IDLE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x30)
    {
        nextMode = 0;

        std::cout << "STEER - Enter TRACKING!" << std::endl;
        // Track IMU heading for debug....
        state = AM_STEER_S_TRACKING;
    }
    else if (nextMode == 0x40)
    {
        nextMode = 0;

        // Save the heading
        savedHeading = yaw;

        // ENTER POSITION ESTIMATE MODE
        sequence[0].command = AM_CMD_START_POS_ESTIMATION;
        sequence[0].speed = 0.0;
        sequence[0].distance = 0.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_NONE;

        //
        // SPOT 1
        //
        // WAIT for a while
        sequence[1].command = AM_CMD_WAIT;
        sequence[1].speed = 0.0;
        sequence[1].distance = 3.0; // Seconds of wait
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_NONE;

        // START SAMPLE POSITION
        sequence[2].command = AM_CMD_START_SAMPLE_POS_ESTIMATION;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = 0.0;
        sequence[2].collisionState = AM_STEER_S_NONE;

        // WAIT for a while
        sequence[3].command = AM_CMD_WAIT;
        sequence[3].speed = 0.0;
        sequence[3].distance = 5.0; // Seconds of wait
        sequence[3].angle = 0.0;
        sequence[3].collisionState = AM_STEER_S_NONE;

        // SAVE POSITION ESTIMATE MODE
        sequence[4].command = AM_CMD_SAVE_POS_ESTIMATION;
        sequence[4].speed = 0.0;
        sequence[4].distance = 0.0;
        sequence[4].angle = 0.0;
        sequence[4].collisionState = AM_STEER_S_NONE;

        //
        // MOVING
        //

        // BACKWARD
        sequence[5].command = AM_CMD_STRAIGHT;
        sequence[5].speed = -0.3;
        sequence[5].distance = calibrationLength;
        sequence[5].angle = 0.0;
        sequence[5].collisionState = AM_STEER_S_NONE;

        //
        // SPOT 2
        //

        // START SAMPLE POSITION
        sequence[6].command = AM_CMD_START_SAMPLE_POS_ESTIMATION;
        sequence[6].speed = 0.0;
        sequence[6].distance = 0.0;
        sequence[6].angle = 0.0;
        sequence[6].collisionState = AM_STEER_S_NONE;

        // WAIT for a while
        sequence[7].command = AM_CMD_WAIT;
        sequence[7].speed = 0.0;
        sequence[7].distance = 5.0; // Seconds of wait
        sequence[7].angle = 0.0;
        sequence[7].collisionState = AM_STEER_S_NONE;

        // SAVE POSITION ESTIMATE MODE
        sequence[8].command = AM_CMD_SAVE_POS_ESTIMATION;
        sequence[8].speed = 0.0;
        sequence[8].distance = 0.0;
        sequence[8].angle = 0.0;
        sequence[8].collisionState = AM_STEER_S_NONE;

        //
        // MOVING
        //

        // TURN
        sequence[9].command = AM_CMD_TURN;
        sequence[9].speed = 0.0;
        sequence[9].distance = 0.0;
        sequence[9].angle = -90.0;
        sequence[9].collisionState = AM_STEER_S_NONE;

        // FORWARD
        sequence[10].command = AM_CMD_STRAIGHT;
        sequence[10].speed = 0.3;
        sequence[10].distance = calibrationLength;
        sequence[10].angle = -90.0;
        sequence[10].collisionState = AM_STEER_S_NONE;

        //
        // SPOT 3
        //

        // START SAMPLE POSITION
        sequence[11].command = AM_CMD_START_SAMPLE_POS_ESTIMATION;
        sequence[11].speed = 0.0;
        sequence[11].distance = 0.0;
        sequence[11].angle = 0.0;
        sequence[11].collisionState = AM_STEER_S_NONE;

        // WAIT for a while
        sequence[12].command = AM_CMD_WAIT;
        sequence[12].speed = 0.0;
        sequence[12].distance = 5.0; // Seconds of wait
        sequence[12].angle = 0.0;
        sequence[12].collisionState = AM_STEER_S_NONE;

        // SAVE POSITION ESTIMATE MODE
        sequence[13].command = AM_CMD_SAVE_POS_ESTIMATION;
        sequence[13].speed = 0.0;
        sequence[13].distance = 0.0;
        sequence[13].angle = 0.0;
        sequence[13].collisionState = AM_STEER_S_NONE;

        //
        // FINISH UP
        //

        // END POSITION ESTIMATE MODE
        sequence[14].command = AM_CMD_END_POS_ESTIMATION;
        sequence[14].speed = 0.0;
        sequence[14].distance = 0.0;
        sequence[14].angle = 0.0;
        sequence[14].collisionState = AM_STEER_S_NONE;

        // DONE
        sequence[15].command = AM_CMD_NONE;
        sequence[15].speed = 0.0;
        sequence[15].distance = 0.0;
        sequence[15].angle = 0.0;
        sequence[15].collisionState = AM_STEER_S_NONE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x41)
    {
        // Simple RANDOM MODE
        std::cout << "STEER - SIMPLE RANDOM MODE!" << std::endl;

        nextMode = 0;



        // Now randomize our turn
        sequence[0].command = AM_CMD_RANDOMIZE_ANGLE;
        sequence[0].speed = 0.0;
        sequence[0].distance = 90.0;
        sequence[0].angle = 180.0;
        sequence[0].collisionState = AM_STEER_S_NONE;

        // Execute the turn
        sequence[1].command = AM_CMD_TURN;
        sequence[1].speed = 0.0;
        sequence[1].distance = 0.0;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_SEQUENCE;
        
        // Save heading...
        sequence[2].command = AM_CMD_SAVE_HEADING;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = 0.0;
        sequence[2].collisionState = AM_STEER_S_IDLE;
        
        // Drive straight (very loooong distance)
        sequence[3].command = AM_CMD_STRAIGHT;
        sequence[3].speed = 0.35;
        sequence[3].distance = 1000.0;
        sequence[3].angle = 0.0;
        sequence[3].collisionState = AM_STEER_S_SEQUENCE;

        // We come here because we collided...
        // BACK A LITTLE BEFORE CHANGE LANE
        // Do not allow collisions to change state
        sequence[4].command = AM_CMD_STRAIGHT;
        sequence[4].speed = -0.3;
        sequence[4].distance = 0.5;
        sequence[4].angle = 0.0;
        sequence[4].collisionState = AM_STEER_S_NONE;



        // REPEAT
        sequence[5].command = AM_CMD_REPEAT;
        sequence[5].speed = 0.0;
        sequence[5].distance = 0.0;
        sequence[5].angle = 0.0;
        sequence[5].collisionState = AM_STEER_S_SEQUENCE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x42)
    {
        // Challenge the border
        std::cout << "STEER - BORDER CHALLENGE!" << std::endl;

        // Save the point of where we start and use that
        // as our point-of-return
        target_pos.point.x = odom.pose.pose.position.x;
        target_pos.point.y = odom.pose.pose.position.y;
        target_pos.point.z = odom.pose.pose.position.z;

        nextMode = 0;

        // Save heading...
        sequence[0].command = AM_CMD_SAVE_HEADING;
        sequence[0].speed = 0.0;
        sequence[0].distance = 0.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_IDLE;

        // Drive straight (very loooong distance)
        sequence[1].command = AM_CMD_STRAIGHT;
        sequence[1].speed = 0.5;
        sequence[1].distance = 1000.0;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_SEQUENCE;

        // We come here because we collided...
        // BACK A LITTLE BEFORE CHANGE LANE
        // Do not allow collisions to change state
        sequence[2].command = AM_CMD_STRAIGHT;
        sequence[2].speed = -0.3;
        sequence[2].distance = 0.5;
        sequence[2].angle = 0.0;
        sequence[2].collisionState = AM_STEER_S_NONE;

        // Turn back to the goal point
        sequence[3].command = AM_CMD_MOVE_TO;
        sequence[3].speed = 0.3;
        sequence[3].distance = 0.0;
        sequence[3].angle = 0.0;
        sequence[3].collisionState = AM_STEER_S_NONE;

        // Now randomize our turn
        sequence[4].command = AM_CMD_RANDOMIZE_ANGLE;
        sequence[4].speed = 0.0;
        sequence[4].distance = 90.0;
        sequence[4].angle = 180.0;
        sequence[4].collisionState = AM_STEER_S_NONE;

        // Execute the turn
        sequence[5].command = AM_CMD_TURN;
        sequence[5].speed = 0.0;
        sequence[5].distance = 0.0;
        sequence[5].angle = 0.0;
        sequence[5].collisionState = AM_STEER_S_SEQUENCE;

        // REPEAT
        sequence[6].command = AM_CMD_REPEAT;
        sequence[6].speed = 0.0;
        sequence[6].distance = 0.0;
        sequence[6].angle = 0.0;
        sequence[6].collisionState = AM_STEER_S_SEQUENCE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x43)
    {
        std::cout << "STEER - FOLLOW PATH (ODOM_COMBINED)!" << std::endl;

        pathTargetIndex = 0;
        nonStopCte = false;

        // Save initial "start_position" (i.e. the next target will use this)
        target_pos.point.x = odom.pose.pose.position.x;
        target_pos.point.y = odom.pose.pose.position.y;
        target_pos.point.z = odom.pose.pose.position.z;

        nextMode = 0;

        // Select next path
        sequence[0].command = AM_CMD_NEXT_TARGET;
        sequence[0].speed = 0.0;
        sequence[0].distance = 0.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_NONE;

        // Save heading...
        sequence[1].command = AM_CMD_SAVE_HEADING;
        sequence[1].speed = 0.0;
        sequence[1].distance = 0.0;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_IDLE;

        // Calculate heading (and distance) to the target
        sequence[2].command = AM_CMD_CALCULATE_TURN_TO_TARGET;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = 0.0;
        sequence[2].collisionState = AM_STEER_S_NONE;

        // Execute the turn
        sequence[3].command = AM_CMD_TURN;
        sequence[3].speed = 0.0;
        sequence[3].distance = 0.0;
        sequence[3].angle = 0.0;
        sequence[3].collisionState = AM_STEER_S_SEQUENCE;

        // WAIT for a while
        sequence[4].command = AM_CMD_WAIT;
        sequence[4].speed = 0.0;
        sequence[4].distance = 0.0; // Seconds of wait
        sequence[4].angle = 0.0;
        sequence[4].collisionState = AM_STEER_S_IDLE;

        // Save heading...
        sequence[5].command = AM_CMD_SAVE_HEADING;
        sequence[5].speed = 0.0;
        sequence[5].distance = 0.0;
        sequence[5].angle = 0.0;
        sequence[5].collisionState = AM_STEER_S_IDLE;

        // Start to move to the path
        sequence[6].command = AM_CMD_MOVE_TO_CTE;
        sequence[6].speed = 0.3;
        sequence[6].distance = 0.0;
        sequence[6].angle = 0.0;
        sequence[6].collisionState = AM_STEER_S_NONE;

        // REPEAT
        sequence[7].command = AM_CMD_REPEAT;
        sequence[7].speed = 0.0;
        sequence[7].distance = 0.0;
        sequence[7].angle = 0.0;
        sequence[7].collisionState = AM_STEER_S_SEQUENCE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x44)
    {
        std::cout << "STEER - FOLLOW PATH NO STOP (ODOM_COMBINED)!" << std::endl;

        pathTargetIndex = 0;
        nonStopCte = true;

        // Save initial "start_position" (i.e. the next target will use this)
        target_pos.point.x = odom.pose.pose.position.x;
        target_pos.point.y = odom.pose.pose.position.y;
        target_pos.point.z = odom.pose.pose.position.z;

        nextMode = 0;

        // Select next path
        sequence[0].command = AM_CMD_NEXT_TARGET;
        sequence[0].speed = 0.0;
        sequence[0].distance = 0.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_IDLE;

        // Start to move to the path
        sequence[1].command = AM_CMD_MOVE_TO_CTE;
        sequence[1].speed = 0.3;
        sequence[1].distance = 0.0;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_IDLE;

        // REPEAT
        sequence[2].command = AM_CMD_REPEAT;
        sequence[2].speed = 0.0;
        sequence[2].distance = 0.0;
        sequence[2].angle = 0.0;
        sequence[2].collisionState = AM_STEER_S_SEQUENCE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }
    else if (nextMode == 0x45)
    {
        std::cout << "STEER - LEAVE CHARGE STATION!" << std::endl;

        nextMode = 0;

        // Save the heading
        savedHeading = yaw;
        
        // BACKWARD
        sequence[0].command = AM_CMD_STRAIGHT;
        sequence[0].speed = -0.3;
        sequence[0].distance = 1.0;
        sequence[0].angle = 0.0;
        sequence[0].collisionState = AM_STEER_S_NONE;
     
        // DONE
        sequence[1].command = AM_CMD_NONE;
        sequence[1].speed = 0.0;
        sequence[1].distance = 0.0;
        sequence[1].angle = 0.0;
        sequence[1].collisionState = AM_STEER_S_NONE;

        // Start....
        commandIndex = 0;
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        std::cout << "Start sequence:" << std::endl;
    }


    // Override...
    if (newGoal)
    {
        newGoal = false;
        target_pos.point.x = nav_goal.pose.position.x;
        target_pos.point.y = nav_goal.pose.position.y;
        target_pos.point.z = nav_goal.pose.position.z;

        state = AM_STEER_S_MOVE_TO;
        doneState = AM_STEER_S_IDLE;
    }
}

void Steering::stateMoveTo(ros::Duration dt)
{
    ///////////////////////////////////////////////////////////////////
    // Calculate the ROBOT position
    ///////////////////////////////////////////////////////////////////
    geometry_msgs::PointStamped base_pos;
    base_pos.header.frame_id = "base_link";
    base_pos.header.stamp = ros::Time();
    base_pos.point.x = 0.0;
    base_pos.point.y = 0.0;
    base_pos.point.z = 0.0;

    geometry_msgs::PointStamped robot_pos;

    try
    {
        listener.transformPoint("odom_combined", base_pos, robot_pos);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"odom_combined\": %s",
                  ex.what());
        return;
    }

    ///////////////////////////////////////////////////////////////////
    // Calculate the DISTANCE to the target
    ///////////////////////////////////////////////////////////////////
    double distanceLeft = sqrt((robot_pos.point.x - target_pos.point.x) * (robot_pos.point.x - target_pos.point.x) +
                               (robot_pos.point.y - target_pos.point.y) * (robot_pos.point.y - target_pos.point.y));

    // Control/regulate the moving...
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    // Assume straight first...
    vel.linear.x = speed;
    vel.angular.z = 0.0;

    ///////////////////////////////////////////////////////////////////
    // Figure out if we need to "steer" some...
    ///////////////////////////////////////////////////////////////////
    geometry_msgs::PointStamped fwd_vector;
    fwd_vector.header.frame_id = "base_link";
    fwd_vector.header.stamp = ros::Time();
    fwd_vector.point.x = 0.5;
    fwd_vector.point.y = 0.0;
    fwd_vector.point.z = 0.0;

    geometry_msgs::PointStamped fwd_pos;
    try
    {
        listener.transformPoint("odom_combined", fwd_vector, fwd_pos);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"odom_combined\": %s",
                  ex.what());
    }

    // Now, we have a "fwd_pos" (2.0m in front of the robot)
    // we also have a "robot_pos" and a "target_pos"
    tf::Vector3 robot_direction(
        fwd_pos.point.x - robot_pos.point.x, fwd_pos.point.y - robot_pos.point.y, fwd_pos.point.z - robot_pos.point.z);
    tf::Vector3 target_direction(target_pos.point.x - robot_pos.point.x,
                                 target_pos.point.y - robot_pos.point.y,
                                 target_pos.point.z - robot_pos.point.z);

    // Get the angle between the vectors
    double angle = tf::tfAngle(robot_direction, target_direction);
    //~ std::cout << "Angle: " << angle << ", Speed: " << speed << std::endl;

    tf::Vector3 target_local;

    try
    {
        tf::StampedTransform transform;
        listener.lookupTransform("/base_link", "/target_position", ros::Time(0), transform);

        target_local = transform.getOrigin();
        // ROS_INFO("local_target: (%.2f, %.2f. %.2f)", target_local.x(), target_local.y(), target_local.z());
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"odom_combined\": %s",
                  ex.what());
        return;
    }

    // Difference in "y" means we need to steer...
    // y>0 => we are aiming right, i.e. steer left...

    if (fabs(angle) > 0.01)
    {
        if (target_local.y() > 0)
        {
            vel.angular.z = angle * 5.0;
        }
        else
        {
            vel.angular.z = -angle * 5.0;
        }
    }
    else
    {
        // Steer straight...no update!
        vel.angular.z = 0;
    }

    // Special case...if we are CLOSE to the target...
    if (distanceLeft < 0.3)
    {
        // Change to "STRAIGHT" the last distance of the target...
        wantedHeading = yaw;
        distance = distanceLeft;
        previousOdom = odom;
        speed = 0.3;

        state = AM_STEER_S_STRAIGHT;

        // Smooth out the steering a lot...
        // vel.angular.z = vel.angular.z / 10.0;
        // std::cout << "---- STEERING DAMPENING" << std::endl;
    }

    ///////////////////////////////////////////////////////////////////
    // Break if we got a new command...
    ///////////////////////////////////////////////////////////////////
    if (nextMode != 0)
    //if (nextMode == 1)
    {
        ROS_INFO("Steering::stateMoveTo::==>BREAK!, nextMode = %d", nextMode);
        
        nextMode = 0;

        ROS_INFO("==>BREAK!");
        state = breakState;

        // Stop
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }

    ///////////////////////////////////////////////////////////////////
    // Break if close to target point...
    ///////////////////////////////////////////////////////////////////
    if (distanceLeft < 0.10)
    {
        ROS_INFO("==>TARGET REACHED!");
        state = doneState;

        // Stop
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }

    ///////////////////////////////////////////////////////////////////
    // PUBLISH
    ///////////////////////////////////////////////////////////////////
    cmd_pub.publish(vel);
}

void Steering::stateMoveCte(ros::Duration dt)
{

    double minSpeed = 0.0;

    ///////////////////////////////////////////////////////////////////
    // Calculate the ROBOT position
    ///////////////////////////////////////////////////////////////////
    geometry_msgs::PointStamped base_pos;
    base_pos.header.frame_id = "cutting_disc_center";
    base_pos.header.stamp = ros::Time();
    base_pos.point.x = 0.0;
    base_pos.point.y = 0.0;
    base_pos.point.z = 0.0;

    geometry_msgs::PointStamped robot_pos;

    try
    {
        listener.transformPoint("odom_combined", base_pos, robot_pos);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR(
            "Received an exception trying to transform a point from \"cutting_disc_center\" to \"odom_combined\": %s",
            ex.what());
        return;
    }

    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    // Assume straight first...
    vel.angular.z = 0.0;

    ///////////////////////////////////////////////////////////////////
    // CTE
    ///////////////////////////////////////////////////////////////////
	


    // How far into the trajectory (line) are we?
    double dx = target_pos.point.x - start_pos.point.x;
    double dy = target_pos.point.y - start_pos.point.y;

    // std::cout << "dx: " << dx << " dy:" << dy << std::endl;

    // Get the distance from the "path", i.e. a "line between target_pos and start_pos"
    double drx = robot_pos.point.x - start_pos.point.x;
    double dry = robot_pos.point.y - start_pos.point.y;

	
    // std::cout << "drx: " << drx << " dry:" << dy << std::endl;

    // U is the robot estimate projects onto the path segment
    double u = (drx * dx + dry * dy) / (dx * dx + dy * dy);
    
    // Only do control if not completed...
    if (u <= 1 && !isnanf(u))
    {
        // std::cout << "u: " << u << std::endl;

        // cte is the estimate projected onto the normal of the path segment
        cte = (dry * dx - drx * dy) / (dx * dx + dy * dy);
        
        double dtx = target_pos.point.x - robot_pos.point.x;
		double dty = target_pos.point.y - robot_pos.point.y;
        // If Robot is too far from the start pos, reset startPos to current position
		if (fabs(cte) > 0.3 && (fabs(dtx) > 0.5 && fabs(dty) > 0.5))
		{
			ROS_ERROR("Steering::CTE::Too large error: %f, dtx: %f, dty: %f", cte, dtx, dty);
			start_pos.point.x = robot_pos.point.x;
			start_pos.point.y = robot_pos.point.y;
			return;
		}
        //dx, dy zero would mean that we are there => move on...

        // The CTE will depend on the "length" of the path,
        // for regulation params we "assume" a standard length
        // hence we "rescale" the CTE
        double len = sqrt(dx * dx + dy * dy);
        cte = cte * (len / 3.0);

        // integral part
        cte_diff += cte;

        // derivate part
        if (cte_last == 0.0)
        {
            cte_deriv = 0.0;
            cte_last = cte;
        }
        else
        {
            cte_deriv = (cte - cte_last) / dt.toSec();
            if (isnanf(cte_deriv))
            {
                cte_deriv = 0;
                ROS_ERROR("Steering::CTE isnanf(cte_deriv) => set cte_deriv to zero!");
            }
            cte_last = cte;
        }

        //std::cout << "CTE: " << cte << " CTE_DIFF:" << cte_diff << " CTE_DERIV:" << cte_deriv << std::endl;

        // Calculate steering, PID
        double steerAngle = -cte_kp * cte - cte_ki * cte_diff - cte_kd * cte_deriv;

        // std::cout << "P: " << -cte_kp*cte << " I:" << -cte_ki*cte_diff << " D:" << cte_kd*cte_deriv << " steer:" <<
        // -cte_kp*cte - cte_ki*cte_diff - cte_kd*cte_deriv << std::endl;

        // Limit steering
        if (steerAngle * 180.0 / M_PI > cte_limit)
        {
            steerAngle = cte_limit * M_PI / 180.0;
        }
        if (steerAngle * 180.0 / M_PI < -cte_limit)
        {
            steerAngle = -cte_limit * M_PI / 180.0;
        }
        
        // std::cout << "STEER: " << steerAngle << "(" << steerAngle*180.0/M_PI << ")" << std::endl;

        vel.angular.z = steerAngle;

        ///////////////////////////////////////////////////////////////////
        // Calculate angle to target
        ///////////////////////////////////////////////////////////////////

        double angleToNextTarget = 0.0;

        double fixedRobotHeading = yaw;
        FIX_ANGLES(fixedRobotHeading);

        int peekPosIndex = pathTargetIndex;
        if (peekPosIndex < path.poses.size())
        {
            geometry_msgs::PointStamped peekPos;
            peekPos.point.x = path.poses[peekPosIndex].pose.position.x;
            peekPos.point.y = path.poses[peekPosIndex].pose.position.y;
            peekPos.point.z = path.poses[peekPosIndex].pose.position.z;

            double x1 = robot_pos.point.x;
            double y1 = robot_pos.point.y;
            double x2 = peekPos.point.x;
            double y2 = peekPos.point.y;

            dx = x2 - x1;
            dy = y2 - y1;

            double distanceToNextTarget = sqrt(dx * dx + dy * dy);

            double headingTowardsNextTarget = atan2(dy, dx);
            FIX_ANGLES(headingTowardsNextTarget);

            double diffAngleNextTarget = headingTowardsNextTarget - fixedRobotHeading;
            if (diffAngleNextTarget > M_PI)
            {
                diffAngleNextTarget = diffAngleNextTarget - 2 * M_PI;
            }
            if (diffAngleNextTarget < -M_PI)
            {
                diffAngleNextTarget = diffAngleNextTarget + 2 * M_PI;
            }

            angleToNextTarget = fabs(diffAngleNextTarget) * 180.0 / M_PI;
            // std::cout << "angleToNextTarget: " << angleToNextTarget << std::endl;
        }

        ///////////////////////////////////////////////////////////////////
        // Ramp up speed...
        ///////////////////////////////////////////////////////////////////

        dx = target_pos.point.x - robot_pos.point.x;
        dy = target_pos.point.y - robot_pos.point.y;

        double distanceToThisTarget = sqrt(dx * dx + dy * dy);

        double headingTowardsThisTarget = atan2(dy, dx);
        FIX_ANGLES(headingTowardsThisTarget);

        double diffAngleThisTarget = headingTowardsThisTarget - fixedRobotHeading;
        if (diffAngleThisTarget > M_PI)
        {
            diffAngleThisTarget = diffAngleThisTarget - 2 * M_PI;
        }
        if (diffAngleThisTarget < -M_PI)
        {
            diffAngleThisTarget = diffAngleThisTarget + 2 * M_PI;
        }

        double angleToThisTarget = fabs(diffAngleThisTarget) * 180.0 / M_PI;
        // std::cout << "angleToThisTarget: " << angleToThisTarget << std::endl;

        // Ramp if we are not about to slow down...
        if ((distanceToThisTarget >= 0.35) && (angleToThisTarget > 45))
        {
            // Large angle...make sure we do have a minimal FWD motion
            if (speed < 0.04)
            {
                speed = 0.04;
            }
        }
        else 
        //if ((distanceToThisTarget >= 0.35) && (angleToThisTarget <= 10))
        if ((distanceToThisTarget >= 0.35) && (fabs(cte) <= 0.005))
        {
            // Ramp...
            //speed += 0.003;
            speed += 0.01;

            if (speed > cte_wantedSpeed)
            {
                speed = cte_wantedSpeed;
            }
        }

        ///////////////////////////////////////////////////////////////////
        // Slow down...
        ///////////////////////////////////////////////////////////////////

        if (nonStopCte == false)
        {
            if (distanceToThisTarget < 0.35)
            {
                speed = speed - 0.01;

                if (speed < minSpeed)
                {
                    speed = minSpeed;
                }
            }
        }
        else
        {
            if (distanceToThisTarget < 0.35)
            {
                double breakFactor = 0.01;
                // Check into the future...do we have a "straight angle" to next
                // segment? If so, the do not slow down...

                // Scale 0.01 (max) ... 0.000
                if (angleToNextTarget >= 120)
                {
                    breakFactor = 0.012;
                    minSpeed = 0.05;
                }
                else if (angleToNextTarget >= 45)
                {
                    breakFactor = 0.008;
                    minSpeed = 0.05;
                }
                else
                {
                    breakFactor = 0.005 * (angleToNextTarget / 45.0);
                    minSpeed = 0.08;
                }

                // std::cout << "BREAK FACTOR: " << breakFactor << std::endl;

                speed = speed - breakFactor;

                if (speed < minSpeed)
                {
                    speed = minSpeed;
                }
            }
        }

        // Set the wanted speed
        vel.linear.x = speed;
    }
    ///////////////////////////////////////////////////////////////////
    // Break if close to target point...
    ///////////////////////////////////////////////////////////////////
    // if ((u > 1.0) || (left_dist < 0.05))
    if (u > 1.0)
    {
        ROS_INFO("==>TARGET REACHED!");

        if ((nonStopCte == false) || (cte_lastSegment == true))
        {
            // Lookup the "base_link" and see how much further we need to travel
            geometry_msgs::PointStamped base_link;
            base_link.header.frame_id = "cutting_disc_center";
            base_link.header.stamp = ros::Time();
            base_link.point.x = 0.0;
            base_link.point.y = 0.0;
            base_link.point.z = 0.0;

            geometry_msgs::PointStamped current_pos;

            try
            {
                listener.transformPoint("odom_combined", base_link, current_pos);
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR(
                    "Received an exception trying to transform a point from \"base_link\" to \"odom_combined\": %s",
                    ex.what());
                return;
            }

            double ldx = target_pos.point.x - current_pos.point.x;
            double ldy = target_pos.point.y - current_pos.point.y;
            double leftToDrive = sqrt(ldx * ldx + ldy * ldy);

            // Change to "STRAIGHT" the last distance of the target...
            wantedHeading = yaw;
            distance = leftToDrive;
            previousOdom = odom;
            speed = 0.2;

            state = AM_STEER_S_STRAIGHT;
        }
        else
        {
            // Fake the "start position" so that we draw a line from where we are
            // to the next target
            // target_pos.point.x = robot_pos.point.x;
            // target_pos.point.y = robot_pos.point.y;
            // Reached our goal..
            state = doneState;
        }
    }

    ///////////////////////////////////////////////////////////////////
    // Break if we got a new command...
    ///////////////////////////////////////////////////////////////////
    if (nextMode != 0)
    {
        ROS_INFO("Steering::stateMoveTo::==>BREAK!, nextMode = %d", nextMode);
        nextMode = 0;
        state = breakState;

        // Stop
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }

    // std::cout << "----------------" << std::endl;
    // std::cout << "SPEED: " << vel.linear.x << std::endl;
    // std::cout << "TURN: " << vel.angular.z << std::endl;

    ///////////////////////////////////////////////////////////////////
    // PUBLISH
    ///////////////////////////////////////////////////////////////////
    if (isnanf(vel.linear.x) || isnanf(vel.angular.z) )
    {
        vel.linear.x = 0;
        vel.angular.z = 0;
        ROS_ERROR("Steering::CTE nan value!");
    }
    cmd_pub.publish(vel);
}

void Steering::stateStraight(ros::Duration dt)
{
    if (newOdomData)
    {
        geometry_msgs::Twist vel;

        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;

        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;

        // Assume straight first...
        vel.linear.x = speed;
        vel.angular.z = 0.0;

        newOdomData = false;

        // Calculate the distance traveled

        double dx = odom.pose.pose.position.x - previousOdom.pose.pose.position.x;
        double dy = odom.pose.pose.position.y - previousOdom.pose.pose.position.y;
        double deltaDist = sqrt(dx * dx + dy * dy);
        distance = distance - deltaDist;

        previousOdom = odom;

        //~ std::cout << "deltaDist: " << deltaDist << std::endl; //<< " Left: " << lastEncoder.lwheel << " Right: " <<
        //lastEncoder.rwheel <<  std::endl;

        // double diff = 0.0;

        if (newImuData)
        {
            newImuData = false;

            currentHeading = yaw;

            // Convert to 0..360
            if (currentHeading > 2.0 * M_PI)
            {
                currentHeading = currentHeading - 2.0 * M_PI;
            }
            if (currentHeading < 0)
            {
                currentHeading = currentHeading + 2.0 * M_PI;
            }

            //
            // PID of wanted heading vs current(IMU) heading
            //
            err_old = P_err;
            err = (wantedHeading + extraOffset) - currentHeading;

            // If the error is bigger than 180, we are on other side...
            if (err < -M_PI)
            {
                // std::cout << "err < -180" << std::endl;
                err = err + 2 * M_PI;
            }

            // If the error is bigger than 180, we are on other side...
            if (err > M_PI)
            {
                // std::cout << "err > 180" << std::endl;
                err = err - 2 * M_PI;
            }

            P_err = err;
            I_err = I_err + err_old;
            D_err = err - err_old;

            vel.angular.z = Pg * P_err + Ig * I_err + Dg * D_err;
        }

        if (distance < 0.2)
        {
            if (speed > 0)
            {
                speed = speed - 0.01;

                if (speed < 0.08)
                {
                    speed = 0.08;
                }
            }
            else
            {
                speed = speed + 0.01;

                if (speed > -0.08)
                {
                    speed = -0.08;
                }
            }

            // ROS_INFO("==>SLOW DOWN: %f", speed);
            vel.linear.x = speed;
        }

        // Start BREAKING early as we have a retardation phase in the
        // am_driver controller.
        if (distance < 0.00)
        {
            // Reached our goal..
            // stop here.
            state = doneState;
            // Stop
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;

            //~ std::cout << "Distance left: " << distance << " Err: " << err*180.0/M_PI  << " Yaw:" << yaw*180.0/M_PI
            //<< " Steering angle: " << vel.angular.z*180.0/M_PI << std::endl;
            //~ std::cout << "Stop!" << std::endl;
        }

        ///////////////////////////////////////////////////////////////////
        // Break if we got a new command...
        ///////////////////////////////////////////////////////////////////
        if (nextMode != 0)
        {
            ROS_INFO("Steering::stateStraight::==>BREAK!, nextMode = %d", nextMode);
        
            nextMode = 0;
            state = breakState;

            // Stop
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
        }

        ///////////////////////////////////////////////////////////////////
        // If we hit something...take next state
        ///////////////////////////////////////////////////////////////////
        if (statusCollision || statusOutside || statusConOutside)
        {
            
            if (collisionState != AM_STEER_S_NONE)
            {
                ROS_INFO("==>HANDLED COLLISION! statusCollision=%d, statusOutside=%d, statusConOutside=%d",statusCollision,statusOutside, statusConOutside );
                state = collisionState;

                // Stop
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
            }
            else
            {
                //ROS_INFO("==>UNHANDLED COLLISION!, state = %d", state);
            }
        }

        ///////////////////////////////////////////////////////////////////
        // PUBLISH
        ///////////////////////////////////////////////////////////////////
        //~ std::cout << "Distance left: " << distance << " Err: " << err*180.0/M_PI  << " Yaw:" << yaw*180.0/M_PI << "
        //Steering angle: " << vel.angular.z*180.0/M_PI << std::endl;
        cmd_pub.publish(vel);
    }
}

void Steering::stateTurnTo(ros::Duration dt)
{
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    if (newImuData)
    {
        newImuData = false;

        currentHeading = yaw;

        // Convert to 0..360
        if (currentHeading > 2.0 * M_PI)
        {
            currentHeading = currentHeading - 2.0 * M_PI;
        }
        if (currentHeading < 0)
        {
            currentHeading = currentHeading + 2.0 * M_PI;
        }

        //
        // PID of wanted heading vs current(IMU) heading
        //
        err_old = P_err;
        err = wantedHeading - currentHeading;

        // Are we turning left or right?
        if (turnAngle > 0)
        {
            // LEFT

            // If the error is bigger than 180, we are on other side...
            if (err < -M_PI)
            {
                // std::cout << "LEFT: err < -180" << std::endl;
                err = err + 2 * M_PI;
            }
        }
        else
        {
            // RIGHT

            // If the error is bigger than 180, we are on other side...
            if (err > M_PI)
            {
                // std::cout << "RIGHT: err > 180" << std::endl;
                err = err - 2 * M_PI;
            }
        }

        P_err = err;
        I_err = I_err + err_old;
        D_err = err - err_old;

        //~ std::cout << "P_err: " << P_err << " I_err: " << I_err  << " D_err:" << D_err << std::endl;

        vel.angular.z = Pg_turn * P_err + Ig_turn * I_err + Dg_turn * D_err;
        // vel.angular.z = Pg*P_err + Ig*I_err + Dg*D_err;

        //~ std::cout << "Angular: " << vel.angular.z << std::endl;

        // MAX Limit...
        if (vel.angular.z > 0.50)
        {
            vel.angular.z = 0.50;
        }
        else if (vel.angular.z < -0.50)
        {
            vel.angular.z = -0.50;
        }

        // MIN Limit
        /*
                        if ((vel.angular.z > 0.0) && (vel.angular.z < 0.20))
                        {
                                vel.angular.z = 0.20;
                        }
                        else if ((vel.angular.z < 0.0) && (vel.angular.z > -0.20))
                        {
                                vel.angular.z = -0.20;
                        }
        */

        if (fabs(err) * 180.0 / M_PI < 1.0)
        {
            // Reached our goal..
            // stop here.
            state = doneState;
            // Stop
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;

            std::cout << "Done turning - err: " << err * 180.0 / M_PI << " Current:" << currentHeading * 180.0 / M_PI
                      << " Wanted:" << wantedHeading * 180.0 / M_PI
                      << " Steering angle: " << vel.angular.z * 180.0 / M_PI << std::endl;
        }

        ///////////////////////////////////////////////////////////////////
        // Break if we got a new command...
        ///////////////////////////////////////////////////////////////////
        if (nextMode != 0)
        {
            ROS_INFO("Steering::stateTurnTo::==>BREAK!, nextMode = %d", nextMode);
            nextMode = 0;
            state = breakState;

            // Stop
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
        }

        ///////////////////////////////////////////////////////////////////
        // PUBLISH
        ///////////////////////////////////////////////////////////////////
        // std::cout << "Turning err: " << err*180.0/M_PI << " Current:" << currentHeading*180.0/M_PI << " Wanted:" <<
        // wantedHeading*180.0/M_PI << " Steering angle: " << vel.angular.z*180.0/M_PI << std::endl;
        cmd_pub.publish(vel);
    }
}

void Steering::stateSequence(ros::Duration dt)
{
    // Get the next command and start that
    int nextCmd = sequence[commandIndex].command;
    std_msgs::UInt16 cmd;
    double offset;
    double spread;

    if (nextMode != 0x0)
    {
        // Something else...jump to IDLE.
        nextMode = 0;
        state = AM_STEER_S_IDLE;
        std::cout << "STEER-SEQ: ABORTED!" << std::endl;
        return;
    }

    breakState = AM_STEER_S_IDLE;
    collisionState = sequence[commandIndex].collisionState;
    // std::cout << "STEER-SEQ: cmd: " << sequence[commandIndex].command << " dist: " << sequence[commandIndex].distance
    // << " angle: " << sequence[commandIndex].angle << std::endl;
    //std::cout << "STEER-SEQ: -----------" << std::endl;

    switch (nextCmd)
    {
    case AM_CMD_NONE:
        state = AM_STEER_S_IDLE;
      //  std::cout << "STEER-SEQ: End...goto IDLE!" << std::endl;
        break;

    case AM_CMD_SAVE_HEADING:
        // Save the heading
        savedHeading = yaw;

        // Change state and return here after that...
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

       // std::cout << "STEER-SEQ: SAVE heading: " << savedHeading * 180.0 / M_PI << std::endl;
        break;

    case AM_CMD_STRAIGHT:
        // Set a "distance" and a "drive straight" goal...
        distance = sequence[commandIndex].distance;

        if (distance == 0.0)
        {
            distance = nextDistance;
        }

        speed = sequence[commandIndex].speed;
        previousOdom = odom;

        // Use the last saved heading...
        wantedHeading = savedHeading + sequence[commandIndex].angle * M_PI / 180.0;

        // Reset PID
        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        // Change state and return here after that...
        state = AM_STEER_S_STRAIGHT;
        doneState = AM_STEER_S_SEQUENCE;

       // std::cout << "STEER-SEQ: STRAIGHT: " << distance << " direction: " << wantedHeading * 180.0 / M_PI << std::endl;
        break;

    case AM_CMD_TURN:

        // Save for regulator
        currentHeading = 0;

        // Turn to the SAVED angle plus what is in sequence
        if (sequence[commandIndex].angle == 0.0)
        {
            wantedHeading = savedHeading + nextAngle * M_PI / 180.0;
            nextAngle = 0.0;
        }
        else
        {
            wantedHeading = savedHeading + sequence[commandIndex].angle * M_PI / 180.0;
        }

        // Convert to 0..360
        if (wantedHeading > 2.0 * M_PI)
        {
            wantedHeading = wantedHeading - 2.0 * M_PI;
        }
        if (wantedHeading < 0)
        {
            wantedHeading = wantedHeading + 2.0 * M_PI;
        }

        turnAngle = wantedHeading - yaw;
        if (turnAngle > M_PI)
        {
            turnAngle = turnAngle - 2 * M_PI;
        }
        if (turnAngle < -M_PI)
        {
            turnAngle = turnAngle + 2 * M_PI;
        }

        // Reset PID
        err = 0.0;
        err_old = 0.0;
        P_err = 0.0;
        I_err = 0.0;
        D_err = 0.0;

        // Change state and return here after that...
        state = AM_STEER_S_TURN;
        doneState = AM_STEER_S_SEQUENCE;

       // std::cout << "STEER-SEQ: TURN from:" << yaw * 180.0 / M_PI << " to:" << wantedHeading * 180.0 / M_PI
        //          << " turnAngle:" << turnAngle * 180.0 / M_PI << std::endl;

        break;

    case AM_CMD_REPEAT:
        // Set to -1 (will be increased after switch case)
        commandIndex = -1;
    //    std::cout << "STEER-SEQ: REPEAT" << std::endl;
        break;

    case AM_CMD_START_POS_ESTIMATION:
   //     std::cout << "STEER-SEQ: AM_CMD_START_POS_ESTIMATION" << std::endl;
        cmd.data = HVA_UWB_CMD_START_POS_ESTIMATION;
        uwb_cmd_pub.publish(cmd);
        break;

    case AM_CMD_SAVE_POS_ESTIMATION:
    //    std::cout << "STEER-SEQ: AM_CMD_SAVE_POS_ESTIMATION" << std::endl;
        cmd.data = HVA_UWB_CMD_SAVE_POS_ESTIMATION;
        uwb_cmd_pub.publish(cmd);
        break;

    case AM_CMD_END_POS_ESTIMATION:
     //   std::cout << "STEER-SEQ: AM_CMD_END_POS_ESTIMATION" << std::endl;
        cmd.data = HVA_UWB_CMD_END_POS_ESTIMATION;
        uwb_cmd_pub.publish(cmd);
        break;

    case AM_CMD_START_SAMPLE_POS_ESTIMATION:
    //    std::cout << "STEER-SEQ: AM_CMD_START_SAMPLE_POS_ESTIMATION" << std::endl;
        cmd.data = HVA_UWB_CMD_START_SAMPLE_POS_ESTIMATION;
        uwb_cmd_pub.publish(cmd);
        break;

    case AM_CMD_WAIT:
        waitUntil = ros::Duration((float)sequence[commandIndex].distance);
        speed = sequence[commandIndex].speed;

        // Change state and return here after that...
        state = AM_STEER_S_WAIT;
        doneState = AM_STEER_S_SEQUENCE;

    //    std::cout << "STEER-SEQ: WAIT: " << waitUntil.toSec() << " seconds." << std::endl;
        break;

    case AM_CMD_RANDOMIZE_ANGLE:

        offset = sequence[commandIndex].distance;
        spread = sequence[commandIndex].angle;
        // Random angle is offset +/- spread deg
        nextAngle = offset + ((rand() % 1000) / 1000.0) * spread;

    //    std::cout << "STEER-SEQ: RANDOMIZE ANGLE: " << nextAngle << " degrees." << std::endl;

        // Radians
        // nextAngle = nextAngle*M_PI/180.0;

        // Change state and return here after that...
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;
        break;

    case AM_CMD_MOVE_TO:
        speed = sequence[commandIndex].speed;
        // Goal is set outside this...just execute!
        state = AM_STEER_S_MOVE_TO;
        doneState = AM_STEER_S_SEQUENCE;
        break;

    case AM_CMD_MOVE_TO_CTE:
        cte_wantedSpeed = sequence[commandIndex].speed;
        cte_diff = 0.0;
        cte_deriv = 0.0;
        cte = 0.0;
        cte_last = 0.0;
        // Goal is set outside this...just execute!
        state = AM_STEER_S_MOVE_TO_CTE;
        doneState = AM_STEER_S_SEQUENCE;
        break;

    case AM_CMD_NEXT_TARGET:
        // Load the next in the path
        if (pathTargetIndex >= path.poses.size())
        {
            std::cout << "STEER-SEQ: Path DONE...no more waypoints!" << std::endl;
            state = AM_STEER_S_IDLE;
            doneState = AM_STEER_S_IDLE;
        }
        else
        {
            // Use "old target" as start_pos
            start_pos.point.x = target_pos.point.x;
            start_pos.point.y = target_pos.point.y;
            start_pos.point.z = target_pos.point.z;

            // set "new target" from path
            target_pos.point.x = path.poses[pathTargetIndex].pose.position.x;
            target_pos.point.y = path.poses[pathTargetIndex].pose.position.y;
            target_pos.point.z = path.poses[pathTargetIndex].pose.position.z;
            pathTargetIndex++;
            // Change state and return here after that...
            state = AM_STEER_S_SEQUENCE;
            doneState = AM_STEER_S_SEQUENCE;

            if (pathTargetIndex >= path.poses.size())
            {
                cte_lastSegment = true;
       //         std::cout << "STEER-SEQ: ALMOST DONE...on the last segment!" << std::endl;
            }
            else
            {
                cte_lastSegment = false;
            }

    //        std::cout << "STEER-SEQ: Next waypoint (" << target_pos.point.x << ", " << target_pos.point.y << ")"
     //                 << std::endl;
        }
        break;

    case AM_CMD_CALCULATE_TURN_TO_TARGET:
        // Calculate the heading towards the target
        nextAngle = calculateTargetHeading();
  //      std::cout << "STEER-SEQ: TURN_TO ANGLE: " << nextAngle << " degrees." << std::endl;

        // Change state and return here after that...
        state = AM_STEER_S_SEQUENCE;
        doneState = AM_STEER_S_SEQUENCE;

        break;

    default:
        state = AM_STEER_S_IDLE;
        std::cout << "STEER-SEQ: Unknown command...goto IDLE!" << std::endl;
        break;
    }

    // Next command
    commandIndex++;
}

double Steering::calculateTargetHeading()
{

    geometry_msgs::PointStamped base_pos;
    base_pos.header.frame_id = "base_link";
    base_pos.header.stamp = ros::Time();
    base_pos.point.x = 0.0;
    base_pos.point.y = 0.0;
    base_pos.point.z = 0.0;

    geometry_msgs::PointStamped robot_pos;

    try
    {
        listener.transformPoint("odom_combined", base_pos, robot_pos);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"odom_combined\": %s",
                  ex.what());
        return 0.0;
    }

    nextDistance = sqrt((robot_pos.point.x - target_pos.point.x) * (robot_pos.point.x - target_pos.point.x) +
                        (robot_pos.point.y - target_pos.point.y) * (robot_pos.point.y - target_pos.point.y));

    double x1 = robot_pos.point.x;
    double y1 = robot_pos.point.y;
    double x2 = target_pos.point.x;
    double y2 = target_pos.point.y;

    double heading_towards = atan2(y2 - y1, x2 - x1);
    FIX_ANGLES(heading_towards);

    std::cout << "ROBOT H: " << yaw * 180.0 / M_PI << std::endl;
    std::cout << "TOWARDS: " << heading_towards * 180.0 / M_PI << std::endl;

    // Remove where we are now
    heading_towards = heading_towards - odoHeading;
    // FIX_ANGLES(heading_towards);

    std::cout << "DELTA TURN: " << heading_towards * 180.0 / M_PI << std::endl;

    return heading_towards * 180.0 / M_PI;
}

void Steering::stateTracking(ros::Duration dt)
{
    std::cout << "STEER-TRACK: roll:" << roll * 180.0 / M_PI << " pitch:" << pitch * 180.0 / M_PI
              << " yaw: " << yaw * 180.0 / M_PI << std::endl;

    if (nextMode == 0x30)
    {
        nextMode = 0;

        std::cout << "STEER - Leave TRACKING!" << std::endl;
        state = AM_STEER_S_IDLE;
    }
}

void Steering::stateWait(ros::Duration dt)
{
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    // Always standstill in WAIT...
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;

    ///////////////////////////////////////////////////////////////////
    // PUBLISH
    ///////////////////////////////////////////////////////////////////
    cmd_pub.publish(vel);

    ///////////////////////////////////////////////////////////////////
    // Break if time is up
    ///////////////////////////////////////////////////////////////////
    waitUntil -= dt;
    if (waitUntil.toSec() <= 0)
    {
        ROS_INFO("Steering::stateWait::==>BREAK!, timeout!");
        state = doneState;
    }

    ///////////////////////////////////////////////////////////////////
    // Break if we got a new command...
    ///////////////////////////////////////////////////////////////////
    if (nextMode != 0)
    {
        ROS_INFO("Steering::stateWait::==>BREAK!, nextMode = %d", nextMode);
        nextMode = 0;

        
        state = breakState;
    }
}

void Steering::stateFollowLoop(ros::Duration dt)
{
    geometry_msgs::Twist vel;
    bool newVelocity = false;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    // Assume straight first...
    vel.linear.x = loopSpeed;
    vel.angular.z = 0.0;

    if (newOdomData)
    {
        newOdomData = false;
        newVelocity = true;

        // Calculate the distance traveled
        double dx = odom.pose.pose.position.x - previousOdom.pose.pose.position.x;
        double dy = odom.pose.pose.position.y - previousOdom.pose.pose.position.y;
        double deltaDist = sqrt(dx * dx + dy * dy);
        previousOdom = odom;

        // Lower this one...
        distance = distance - deltaDist;

        // Done?
        //~ if (distance < 0.02)
        //~ {
        //~ // Reached our goal..
        //~ // stop here.
        //~ state = doneState;
        //~
        //~ // Stop
        //~ vel.linear.x = 0.0;
        //~ vel.angular.z = 0.0;
        //~
        //~ std::cout << "Distance left: " << distance << std::endl;
        //~ std::cout << "Stop!" << std::endl;
        //~ }
    }

    if (newLoopData)
    {
        newLoopData = false;

        // Have we changed setpoint?
        if (loopSignalSetPoint != 0)
        {
            wantedLoopSignal = loopSignalSetPoint;
        }

        // std::cout << "Center: " << loopData.frontCenter << "Right:" << loopData.frontRight << std::endl;

        //
        // PID of wanted heading vs current(IMU) heading
        //
        err_old = P_err;
        err = (wantedLoopSignal - loopData.frontCenter);

        // std::cout << "ERR: " << err << std::endl;

        if (!followRight)
        {
            err = -err;
        }

        P_err = err;
        I_err = I_err + err_old;
        D_err = err - err_old;

        vel.angular.z = Pg_loop * P_err + Ig_loop * I_err + Dg_loop * D_err;
    }

    if (newVelocity)
    {
        ///////////////////////////////////////////////////////////////////
        // PUBLISH
        ///////////////////////////////////////////////////////////////////
        // std::cout << "Distance left: " << distance << " Err: " << err*180.0/M_PI  << " Yaw:" << yaw*180.0/M_PI << "
        // Steering angle: " << vel.angular.z*180.0/M_PI << std::endl;
        cmd_pub.publish(vel);
    }
}

bool Steering::update(ros::Duration dt)
{
    ros::Time current_time = ros::Time::now();

    // Simply skip the state machine if paused, issue a stop speed when entering pause.
    if (isPaused)
    {
        if (wasPaused == false)
        {
            geometry_msgs::Twist vel;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            cmd_pub.publish(vel);
            wasPaused = true;
        }
        return true;
    }
    // Execute a state machine to do steering
    switch (state)
    {
    case AM_STEER_S_IDLE:
    {
        stateIdle(dt);
        break;
    }
    case AM_STEER_S_MOVE_TO:
    {
        stateMoveTo(dt);
        break;
    }
    case AM_STEER_S_STRAIGHT:
    {
        stateStraight(dt);
        break;
    }
    case AM_STEER_S_TURN:
    {
        stateTurnTo(dt);
        break;
    }
    case AM_STEER_S_SEQUENCE:
    {
        stateSequence(dt);
        break;
    }
    case AM_STEER_S_TRACKING:
    {
        stateTracking(dt);
        break;
    }
    case AM_STEER_S_WAIT:
    {
        stateWait(dt);
        break;
    }
    case AM_STEER_S_FOLLOW_LOOP:
    {
        stateFollowLoop(dt);
        break;
    }
    case AM_STEER_S_MOVE_TO_CTE:
    {
        stateMoveCte(dt);
        break;
    }

    default:
        break;
    }

    // PUBLISHING

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(target_pos.point.x, target_pos.point.y, target_pos.point.z));
    tf::Quaternion q = tf::createQuaternionFromYaw(0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, current_time, "odom_combined", "target_position"));

    // Publish status from steering
    std_msgs::Int16 cmd;
    cmd.data = state;
    state_pub.publish(cmd);

    return true;
}
}
