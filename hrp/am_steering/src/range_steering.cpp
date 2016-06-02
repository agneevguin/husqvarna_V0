/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_steering/range_steering.h"
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
#include <std_msgs/UInt16.h>

using namespace std;

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))

#define FIX_ANGLES(a)       \
    if (a > M_PI * 2.0)     \
    {                       \
        a = a - M_PI * 2.0; \
    }                       \
    else if (a < 0.0)       \
    {                       \
        a = a + M_PI * 2.0; \
    }

namespace Husqvarna
{

#define STATE_START_FOLLOW_RADIUS 0
#define STATE_FOLLOW_RADIUS 1
#define STATE_START_TURN 2
#define STATE_TURN 3
#define STATE_START_MOVE_FROM_LOOP 4
#define STATE_MOVE_FROM_LOOP 5

#define STATE_DETECT_BEACONS 6
#define STATE_MEASURE 7
#define STATE_DOCK_STRAIGHT 8
#define STATE_START_TURN_TO 9
////////////////////////////////////////////////////////////////////////
// Particle
////////////////////////////////////////////////////////////////////////
Particle::Particle(double x, double y, double z, double h)
{
    xpos = x;
    ypos = y;
    zpos = z;
    heading = h;

    weight = 1.0;
}

Particle::~Particle()
{
}

////////////////////////////////////////////////////////////////////////
// Beacon
////////////////////////////////////////////////////////////////////////
Beacon::Beacon() : Particle(0.0, 0.0, 0.0, 0.0)
{
    range = 0;
}

Beacon::~Beacon()
{
}

RangeSteering::RangeSteering(const ros::NodeHandle& nodeh)
{
    // Init attributes
    nh = nodeh;

    // Parameters
    ros::NodeHandle n_private("~");

    // Setup some ROS stuff
    joySub = nh.subscribe("nano2", 1, &RangeSteering::joyCallback, this);
    rangeSub = nh.subscribe("uwb", 1, &RangeSteering::rangeCallback, this);
    odomSub = nh.subscribe("odom", 1, &RangeSteering::odomCallback, this);
    odomCombinedSub = nh.subscribe("odom_combined", 1, &RangeSteering::odomCombinedCallback, this);
    //~ beaconPosSub = nh.subscribe("beacon_pos", 1, &RangeSteering::beaconPosCallback, this);

    beaconPub = nh.advertise<visualization_msgs::Marker>("helpers", 1);
    cmdPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    modePub = nh.advertise<std_msgs::UInt16>("cmd_mode", 1);
    statusSub = nh.subscribe("sensor_status", 1, &RangeSteering::statusCallback, this);
    coveragePub = nh.advertise<nav_msgs::Path>("coverage_plan", 1);

    // Special simulation noise
    std::string defRobotId = "DECA0100-100";
    n_private.param("robotId", robotId, defRobotId);
    ROS_INFO("Param: robotId: [%s]", robotId.c_str());
    addBeacons = true;
    // Left
    beaconList[0].id = "DECA0100-101";
    beaconList[0].xpos = 0.0;
    beaconList[0].ypos = 0.50;
    beaconList[0].zpos = 0.0;
    beaconList[0].heading = 0.0;
    beaconList[0].range = 0.0;

    // Rigth
    beaconList[1].id = "DECA0100-102";
    beaconList[1].xpos = 0.0;
    beaconList[1].ypos = -0.50;
    beaconList[1].zpos = 0.0;
    beaconList[1].heading = 0.0;
    beaconList[1].range = 0.0;

    /* Local positioning stuff*/
    bearingEst = 0;

    minRange = 10000;
    dockStart = 1.0; /* When to enter local docking mode */
    newRange = false;
    doControl = false;
    radius = 1.0;
    radiusInc = 0.15;
    speed = 1.0;
    turnSpeed = 1.0;
    leftRange = 0;
    rightRange = 0;
    waitingForRelease = false;
    joyDisabled = true;
    dockStarted = false;
    seekDockStart = false;

    followDirection = 1;
    kpDock = 1.0;
    kdDock = 9.6;
    kiDock = 0.00;

    kp = 0.7;
    kd = 7.25;
    ki = 0.20;

    numInterBeaconSamples = 0;

    /* How much before the baseline between home beacons shall it stop? */
    baselineOffset = 0.5;
    rangeHome = 0;
    errOld = 0.0;
    passed180 = false;

    state = STATE_START_FOLLOW_RADIUS;
    rangeState = STATE_DETECT_BEACONS;
}

RangeSteering::~RangeSteering()
{
}

void RangeSteering::printConfig(void)
{
}

bool RangeSteering::setup()
{
    return true;
}

void RangeSteering::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
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
        // Pressing R[2]
        if (j->buttons[25 + 16 + 2] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("UWBPOS::NANO KONTROL enabled!");
            printConfig();
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing R[2]
        if (j->buttons[25 + 16 + 2] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("UWBPOS::NANO KONTROL disabled!");
            printConfig();
            return;
        }

        // Play
        else if (j->buttons[7] == 1)
        {
            /* When deactive, start cover. if active start dock */

            doControl = true;
            doDock = false;
            seekDockStart = false;
            radius = leftRange;
            followDirection = 1;
            state = STATE_START_FOLLOW_RADIUS;
            waitingForRelease = true;
            return;
        }
        // Stop
        else if (j->buttons[6] == 1)
        {
            stop();
            doControl = false;
            doDock = false;

            waitingForRelease = true;
        }
        // Pressing set
        else if (j->buttons[1] == 1)
        {
            ROS_INFO("Start Docking");

            seekDockStart = true;
            // doControl = false;
            // setDockPath();
            waitingForRelease = true;
        }
        else
        {
            // SLIDERS
            double jRadInc = j->axes[0];
            if (jRadInc != 0)
            {
                radiusInc = jRadInc;
                ROS_INFO("RangeSteering::radiusInc = %f", radiusInc);
            }

            double jSpeed = 2 * j->axes[1] - 1.0;
            if (jSpeed != -1.0)
            {
                speed = jSpeed;
                ROS_INFO("RangeSteering::speed = %f", speed);
            }

            double jRadius = 10 * j->axes[2];
            if (jRadius != 0)
            {
                radius = jRadius;
                ROS_INFO("RangeSteering::radius = %f", radius);
            }

            double jkpd = 2 * j->axes[3];
            if (jkpd != 0)
            {
                kpDock = jkpd;
                ROS_INFO("RangeSteering::kpDock = %f", kpDock);
            }

            //~ double jkid = 0.1*j->axes[3];
            //~ if (jkid != 0)
            //~ {
            //~ ki = jkid;
            //~ ROS_INFO("RangeSteering::ki = %f", ki);
            //~ }

            double jkdd = 20 * j->axes[4];
            if (jkdd != 0)
            {
                kdDock = jkdd;
                ROS_INFO("RangeSteering::kdDock = %f", kdDock);
            }

            double jkp = 2 * j->axes[5];
            if (jkpd != 0)
            {
                kp = jkp;
                ROS_INFO("RangeSteering::kp = %f", kp);
            }

            double jki = 0.1 * j->axes[6];
            if (jki != 0)
            {
                ki = jki;
                ROS_INFO("RangeSteering::ki = %f", ki);
            }

            double jkd = 20 * j->axes[7];
            if (jkd != 0)
            {
                kd = jkd;
                ROS_INFO("RangeSteering::kd = %f", kd);
            }

            // knob 0
            double jturn = j->axes[8 + 0];
            if (jturn != 0)
            {
                turnSpeed = jturn;
                ROS_INFO("RangeSteering::turnSpeed = %f", turnSpeed);
            }
        }
    }
}

void RangeSteering::statusCallback(const am_driver::SensorStatus::ConstPtr& msg)
{
    // Sensor status
    // std::cout << "Sensor Status:" << msg->sensorStatus << std::endl;

    // 0x04 - Collision
    // 0x02 - Out of area

    statusCollision = (msg->sensorStatus & 0x04);
    statusOutside = (msg->sensorStatus & 0x02);

    //~ std::cout << "statusCollision:" << statusCollision << std::endl;
    //~ std::cout << "statusOutside:" << statusOutside << std::endl;
    //~
}

void RangeSteering::odomCombinedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Copy & save for use in update
    odomCombined = *msg;
    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    heading = tf::getYaw(pose.getRotation());
}

void RangeSteering::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Copy & save for use in update
    odom = *msg;

    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    odoHeading = tf::getYaw(pose.getRotation());

    newOdomData = true;
}

void RangeSteering::beaconPosCallback(const am_driver::BeaconPositions::ConstPtr& msg)
{
    /* Charging station position is in the middle between the closest beacons */
    double minDist = 100000;
    if (msg->beaconPosX.size() > 1)
    {
        rangeMutex.lock();
        ;
        /* Check for closest beacons */
        for (int i = 0; i < msg->beaconPosX.size() - 1; i++)
        {
            for (int j = i + 1; j < msg->beaconPosX.size(); j++)
            {
                double x1tmp = msg->beaconPosX[i];
                double x2tmp = msg->beaconPosX[j];
                double y1tmp = msg->beaconPosY[i];
                double y2tmp = msg->beaconPosY[j];
                double dx = x1tmp - x2tmp;
                double dy = y1tmp - y2tmp;
                double diff = dx * dx + dy * dy;
                if (diff < minDist)
                {
                    minDist = diff;
                    x1 = x1tmp;
                    x2 = x2tmp;
                    y1 = y1tmp;
                    y2 = y2tmp;
                    beacon1Id = msg->beaconId[i];
                    beacon2Id = msg->beaconId[j];
                }
            }
        }
        rangeMutex.unlock();
        /* Base station position is in the middle of the closest ones */
        baseX = x1 + (x2 - x1) / 2;
        baseY = y1 + (y2 - y1) / 2;

        //~ printf("(%f,%f), (%f,%f), (%f, %f)\n", x1,y1,x2,y2, baseX, baseY);
    }
}

void RangeSteering::rangeCallback(const am_driver::Range::ConstPtr& msg)
{
    double range = msg->range;
    std::string fromId = msg->fromId;
    std::string toId = msg->toId;
    static int gotData[2] = { 0, 0 };
    double tmp;

    double tmpOld;

    switch (rangeState)
    {

    case STATE_DETECT_BEACONS:
        /* Check interbeacon ranges only*/
        if (fromId != robotId)
        {
            //~ cout << "fromId: "<< fromId << ", toId: "<< toId << ", range: " << range << ", minRange: " << minRange
            //<< endl;

            /* Sample for a while and detect the closest beacons */
            if (numInterBeaconSamples > 20)
            {
                rangeState = STATE_MEASURE;
                cout << "Dock between: " << beacon1Id << " and " << beacon2Id << " inter range: " << minRange << endl;
            }
            else
            {
                /* Closer than last */
                if (range < minRange)
                {
                    minRange = range;
                    beacon1Id = fromId;
                    beacon2Id = toId;
                }
                ++numInterBeaconSamples;
            }
        }

        break;

    case STATE_MEASURE:

        if (fromId == robotId)
        {
            if (toId == beacon1Id)
            {
                rangeMutex.lock();
                leftRange = range;
                rangeMutex.unlock();
                gotData[0] = 1;
            }
            if (toId == beacon2Id)
            {
                rangeMutex.lock();
                rightRange = range;
                rangeMutex.unlock();
                gotData[1] = 1;
            }
        }
        else if ((fromId == beacon1Id && toId == beacon2Id) || (fromId == beacon2Id && toId == beacon1Id))
        {
            /* filter the values to increase certainty */
            // Range between home beacons.
            if (baseDist == 0)
            {
                baseDist = range;
            }
            else
            {
                baseDist = 0.9 * baseDist + 0.1 * range;
            }

            /* Range when in home station is computed from offset and base distance */
            rangeHome = sqrt(baseDist * baseDist / 4 + baselineOffset * baselineOffset);
        }

        if (gotData[0] == 1 && gotData[1] == 1)
        {
            newRange = true;
            gotData[0] = 0;
            gotData[1] = 0;
        }
        break;
    }
}

void RangeSteering::stop()
{
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    cmdPub.publish(vel);
}

void RangeSteering::cycleDock(ros::Duration dt)
{
    ros::Time currentTime = ros::Time::now();
    double w0, w;
    geometry_msgs::Twist vel;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;

    //~ /* Check if close enough to start local docking */
    //~ double dockStartRange = sqrt(dockStart*dockStart + baseDist*baseDist/4);
    //~
    //~
    //~ if (fabs(leftRange - dockStartRange) < 0.1 && fabs(rightRange - dockStartRange) < 0.1)
    //~ {
    //~ dockStarted = true;
    //~ cout << "Starting Local Docking!" << endl;
    //~ }

    switch (dockState)
    {
    /* Set the turn angle to base, heading is calculated based on the followed radius */
    /* Approximated by 90 degrees (actually dependent on radius and both ranges) */
    case STATE_START_TURN_TO:
    {
        vel.linear.x = 0.0;
        /* Heading dependent on distance */
        double beta = -followDirection * (M_PI / 2 - asin(baseDist / (2 * leftRange)));
        homeHeading = odoHeading - beta;
        dockState = STATE_TURN;
        cout << "Start turning home!, beta: " << beta << ", home: " << homeHeading << ", odoHeading: " << odoHeading
             << endl;
        break;
    }
    case STATE_TURN:
    {

        vel.linear.x = 0.0;
        vel.angular.z = followDirection * turnSpeed;
        double diff = fabs(odoHeading - homeHeading);
        if (diff >= 2 * M_PI)
        {
            diff -= 2 * M_PI;
        }
        else if (diff <= -2 * M_PI)
        {
            diff += 2 * M_PI;
        }

        cout << "----Turning diff: " << diff * 180 / M_PI << "---" << endl;

        /* Stop within +-5 deg of final angle */
        if (diff < 5 * M_PI / 180)
        {
            vel.angular.z = 0;
            dockState = STATE_DOCK_STRAIGHT;
            dockStarted = true;
            cout << "----Turning done, start straight! " << endl;
        }

        cmdPub.publish(vel);
        break;
    }
    case STATE_DOCK_STRAIGHT:
    {
        if (dockStarted)
        {

            w = 0;

            //~ printf("cosA: %f, xL: %f, yL: %f, dx: %f, dy: %f, distToHome: %f, angleToHome: %f\n",  cosAlpha, xL, yL,
            //dx,dy, distToHome, angleToHome);

            /* Check if docking complete */
            if (distToHome < 0.1)
            {
                cout << "Docking Completed!" << endl;
                doDock = false;
                dockStarted = false;
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
                cmdPub.publish(vel);
            }
            if (doDock)
            {
                vel.linear.x = 0.2; // speed*(distToHome/dockStart);

                errDock = angleToHome;
                if (errOldDock != errDock)
                {
                    dErrDock = errDock - errOldDock;
                }
                errOldDock = errDock;
                w = kpDock * errDock + kdDock * dErrDock;
                vel.angular.z = w;
                cout << "e = " << errDock << ", dErrDock = " << dErrDock << ", w = " << w
                     << ", kpDock*err = " << kpDock* errDock << ",  kdDock*dErrDock = " << kdDock* dErrDock << endl;
                cmdPub.publish(vel);
            }
            break;
        }
    }
    }
}
void RangeSteering::cycleRangeCover(ros::Duration dt)
{
    ros::Time currentTime = ros::Time::now();
    double w0, w;

    geometry_msgs::Twist vel;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;

    if (doControl)
    {
        switch (state)
        {
        case STATE_START_FOLLOW_RADIUS:
        {
            passed180 = false;
            startHeading = odoHeading;
            state = STATE_FOLLOW_RADIUS;
        }
        case STATE_FOLLOW_RADIUS:
        {

            /* Check if a 360 is completed, in that case increase radius. */
            double diff = fabs(odoHeading - startHeading);

            //~ cout << "diff:" << diff*180/M_PI << "passed180: " << passed180 << endl;
            /* Stop within +-10 deg of final angle (180) */
            if (diff < 10 * M_PI / 180 && passed180 == true)
            {
                radius += radiusInc;
                startHeading = odoHeading;
                passed180 = false;
            }
            if (diff > 170 * M_PI / 180 && diff < 190 * M_PI / 180)
            {
                passed180 = true;
            }

            vel.linear.x = speed;

            /* Angular velocity is v/r when travelling on the tangent of the circle */
            /* The radius is given as reference, so is speed => control on angular velocity*/
            w0 = speed / radius;

            /* The errors in radius; */
            err = leftRange - radius;

            /* Only integrate error if magnitude is less than 100 */
            iErr = +errOld;
            if (fabs(iErr > 100))
            {
                iErr -= errOld;
            }

            if (err != errOld)
            {
                dErr = err - errOld;
            }
            errOld = err;

            /* PID control v*/
            w = w0 + kp * err + ki * iErr + kd * dErr;
            printf("r: %f, rRef: %f\n", leftRange, radius);

            //~ printf("r: %f, rRef: %f, err: %f, w0: %f, w: %f, kp*err: %f, iErr: %f, kd*dErr: %f\n", leftRange,
            //radius, err, w0, w, kp*err,iErr,kd*dErr);

            vel.angular.z = followDirection * w;

            if (statusCollision || statusOutside)
            {
                //~ cout << "time to turn!" << endl;
                state = STATE_START_TURN;
            }

            /* If docking is desired, stop following when both ranges are equal */
            if (seekDockStart == true)
            {
                /* Once at same radius for both, start moving towards base! */
                if (fabs(leftRange - rightRange) < 0.1)
                {
                    doControl = false;
                    doDock = true;
                    dockState = STATE_START_TURN_TO;
                    /* Start the control directly */
                    dockStarted = true;
                }
            }

            break;
        }
        case STATE_START_TURN:
        {
            cout << "----Start Turn!---" << endl;
            vel.linear.x = 0;
            vel.angular.z = -followDirection * turnSpeed;
            startHeading = odoHeading;

            state = STATE_TURN;
            break;
        }
        case STATE_TURN:
        {
            vel.angular.z = -followDirection * turnSpeed;
            double diff = fabs(odoHeading - startHeading);
            //~ cout << "----Turning diff: " << diff*180/M_PI << " (goal 180deg)!---" << endl;

            /* Stop within +-10 deg of final angle (180) */
            if (diff > 170 * M_PI / 180 && diff < 190 * M_PI / 180)
            {
                vel.linear.x = 0;
                vel.angular.z = 0;
                followDirection *= -1;
                state = STATE_START_MOVE_FROM_LOOP;
                radius += radiusInc;
            }
            break;
        }
        case STATE_START_MOVE_FROM_LOOP:
        {
            /* If two move from loops are initiated within 1s, the cutting is done,
             * or the robot is stuck, hence start docking! */
            if (currentTime.toSec() - tStart.toSec() < 7.0)
            {
                state = STATE_START_FOLLOW_RADIUS;
                //~ doControl = false;
                // setDockPath();
                seekDockStart = true;
            }
            tStart = currentTime;
            vel.linear.x = speed;
            vel.angular.z = 0;
            state = STATE_MOVE_FROM_LOOP;
        }
        case STATE_MOVE_FROM_LOOP:
        {
            vel.linear.x = speed;
            vel.angular.z = 0;

            /* Drive a bit forward to avoid loop */
            if (currentTime.toSec() - tStart.toSec() > 0.4)
            {
                vel.linear.x = 0;
                vel.angular.z = 0;
                state = STATE_START_FOLLOW_RADIUS;
            }
        }
        default:
            break;
        }

        cmdPub.publish(vel);
    }
}

void RangeSteering::setDockPath()
{

    coveragePath.header.stamp = ros::Time::now();
    coveragePath.header.frame_id = "odom_combined";
    // Now convert/export this as a Path as well
    coveragePath.poses.clear();

    geometry_msgs::PoseStamped pose;
    dockStarted = false;
    doDock = true;

    /* Compute angle of base beacons */
    rangeMutex.lock();
    double baseAngle = atan2((y2 - y1), (x2 - x1));

    double firstDist;
    if (leftRange > dockStart)
    {
        firstDist = leftRange;
    }
    else
    {
        firstDist = dockStart + 0.5;
    }

    cout << "Base: (" << baseX << "," << baseY << ")" << endl;
    /* First follow current radius to the center of the charging station */
    pose.pose.position.x = baseX + sin(baseAngle) * firstDist;
    pose.pose.position.y = baseY + cos(baseAngle) * firstDist;
    pose.pose.position.z = 0;
    cout << "Docking path start: (" << pose.pose.position.x << "," << pose.pose.position.y << ")" << endl;
    coveragePath.poses.push_back(pose);
    cout << "Start: (" << pose.pose.position.x << "," << pose.pose.position.y << ")" << endl;

    /* Then path to just outside docking station (2m) */
    pose.pose.position.x = baseX + dockStart * sin(baseAngle);
    pose.pose.position.y = baseY + dockStart * cos(baseAngle);
    pose.pose.position.z = 0;
    cout << "Docking path goal: (" << pose.pose.position.x << "," << pose.pose.position.y << ")" << endl;
    coveragePath.poses.push_back(pose);
    cout << "Dock: (" << pose.pose.position.x << "," << pose.pose.position.y << ")" << endl;

    rangeMutex.unlock();

    coveragePub.publish(coveragePath);

    /* Issue command to start docking! */
    std_msgs::UInt16 mode;
    mode.data = 0x43;
    modePub.publish(mode);
}

bool RangeSteering::update(ros::Duration dt)
{

    ros::Time currentTime = ros::Time::now();

    /* Compute distance left to the goal: */
    /* Two triangles are created with the distance between docking
     * beacons as base and the range values as sides. One defined by
     * goal position and one defined by current position. Both distance
     * to goal and angle to goal is calculated and used to control the
     * vehicle to the correct position. */

    cosAlpha = (-rightRange * rightRange + baseDist * baseDist + leftRange * leftRange) / (2 * baseDist * leftRange);
    xL = leftRange * cosAlpha;
    yL = sqrt(leftRange * leftRange - xL * xL);
    dx = (baseDist / 2 - xL);
    dy = (yL - baselineOffset);
    distToHome = sqrt((dx * dx) + (dy * dy));
    angleToHome = asin((baseDist / 2 - xL) / distToHome);

    if (doControl)
    {
        cycleRangeCover(dt);
    }
    else if (doDock)
    {
        cycleDock(dt);
    }
    return true;
}
}
