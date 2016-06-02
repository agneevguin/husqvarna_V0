/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_hrp_apps/tracking_app.h"

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <boost/foreach.hpp>

namespace Husqvarna
{

// SensorStatus received
#define HVA_SS_HMB_CTRL 0x0001
#define HVA_SS_OUTSIDE 0x0002
#define HVA_SS_COLLISION 0x0004
#define HVA_SS_LIFTED 0x0008
#define HVA_SS_TOO_STEEP 0x0010
#define HVA_SS_PARKED 0x0020
#define HVA_SS_IN_CS 0x0040
#define HVA_SS_USER_STOP 0x0080
#define HVA_SS_CFG_NEEDED 0x0100
#define HVA_SS_DISC_ON 0x0200

// Internal states
#define HVA_TRK_STATE_IDLE (0)
#define HVA_TRK_STATE_RANDOM (1)
#define HVA_TRK_STATE_IN_CS (2)
#define HVA_TRK_STATE_FINISHED (3)

// Messages to Automower driver
#define HVA_MODE_REQ_MANUAL (0x90)
#define HVA_MODE_REQ_RANDOM (0x91)
#define HVA_MODE_REQ_DISC_OFF (0x92)
#define HVA_MODE_REQ_DISC_ON (0x93)
#define HVA_MODE_REQ_FINISH_ON (0x100)
#define HVA_MODE_REQ_FINISH_OFF (0x101)

// Messages to STEERING
#define HVA_MODE_ABORT (0x01)
#define HVA_MODE_ESTIMATE_BEACONS (0x40)
#define HVA_MODE_SIMPLE_RANDOM (0x41)
#define HVA_MODE_CHALLENGE_BORDER (0x42)
#define HVA_MODE_FOLLOW_PATH (0x44)
#define HVA_MODE_EXIT_CHARGE_STATION (0x45)
#define HVA_MODE_STEERING_PAUSE (0x50)
#define HVA_MODE_STEERING_RESUME (0x51)


// OPERATIONAL MODES (i.e. published in SensorStatus)
#define AM_OP_MODE_OFFLINE (0x0000)
#define AM_OP_MODE_CONNECTED_MANUAL (0x0001)
#define AM_OP_MODE_CONNECTED_RANDOM (0x0002)
#define AM_OP_MODE_CONNECTED_SIMULATOR (0x0003)

TrackingApp::TrackingApp(const ros::NodeHandle& nodeh)
{
    // Init attributes
    this->nh = nodeh;

    // Setup some ROS stuff
    statusSub = nh.subscribe("sensor_status", 1, &TrackingApp::statusCallback, this);
    cmdPub = nh.advertise<std_msgs::UInt16>("cmd_mode", 1);

    // Parameters
    ros::NodeHandle n_private("~");

    int tmpTime;
    n_private.param("wantedMowingTime", tmpTime, 60 * 60);
    ROS_INFO("Param: wantedMowingTime: [%d]", tmpTime);
    wantedMowingTime = ros::Duration(tmpTime);

    state = HVA_TRK_STATE_IDLE;

    logCounter = 0;
}

TrackingApp::~TrackingApp()
{
}

void TrackingApp::statusCallback(const am_driver::SensorStatus::ConstPtr& msg)
{
    userStopped = (msg->sensorStatus & HVA_SS_USER_STOP);
    inCS = (msg->sensorStatus & HVA_SS_IN_CS);
    sensorStatus = (int)msg->sensorStatus;
    operationalMode = (int)msg->operationalMode;
}

bool TrackingApp::setup()
{
    ROS_INFO("TrackingApp::setup()");

    return true;
}

bool TrackingApp::update(ros::Duration dt)
{
    
    bool randomMode = false;
    
    if (operationalMode != AM_OP_MODE_OFFLINE)
    {
        randomMode = true;
    }
    
    ros::Time current_time = ros::Time::now();

    ros::Duration mowingTime;

    // Sensor status
    logCounter--;
    if (logCounter < 0)
    {
        logCounter = 300;
        std::bitset<16> ss(sensorStatus);
        std::cout << "Sensor Status: " << ss << " operationalMode: " << operationalMode << std::endl;
        std::cout << "               xxxxxxDxUCPSLCOx (Disc, User, CS, Park, Steep, Lift, Collision, Outside)"
                  << std::endl;
        std::cout << "state: " << state << std::endl;
    }

    if (operationalMode != AM_OP_MODE_OFFLINE)
    {
        switch (state)
        {
        case HVA_TRK_STATE_IDLE:
            if (inCS)
            {
                ROS_INFO("TrackApp::Change to IN_CS STATE as mower is in CS.");

                if (operationalMode == AM_OP_MODE_CONNECTED_SIMULATOR)
                {
                    // We need to start the "simple random mode" for simulation
                    std_msgs::UInt16 modeCmd;
                    modeCmd.data = HVA_MODE_SIMPLE_RANDOM;
                    cmdPub.publish(modeCmd);
                    ROS_INFO("TrackApp::Send start RANDOM to STEERING.");
                }
                else if (operationalMode != AM_OP_MODE_CONNECTED_RANDOM)
                {
                    // REQUEST RANDOM mode to the mower (force it to CS)
                    std_msgs::UInt16 modeCmd;
                    modeCmd.data = HVA_MODE_REQ_RANDOM;
                    cmdPub.publish(modeCmd);
                    ROS_INFO("TrackApp::Send start RANDOM to AM_DRIVER.");
                }

                state = HVA_TRK_STATE_IN_CS;
            }
            else if (randomMode)
            {
                ROS_INFO("TrackApp::Change to RANDOM STATE as mower is mowing(RANDOM MODE).");
                
                if (operationalMode == AM_OP_MODE_CONNECTED_SIMULATOR)
                {
                    // We need to start the "simple random mode" for simulation
                    std_msgs::UInt16 modeCmd;
                    modeCmd.data = HVA_MODE_SIMPLE_RANDOM;
                    cmdPub.publish(modeCmd);
                    ROS_INFO("TrackApp::Send start RANDOM to STEERING.");
                }
                else if (operationalMode != AM_OP_MODE_CONNECTED_RANDOM)
                {
                    // REQUEST RANDOM mode to the mower (force it to CS)
                    std_msgs::UInt16 modeCmd;
                    modeCmd.data = HVA_MODE_REQ_RANDOM;
                    cmdPub.publish(modeCmd);
                    ROS_INFO("TrackApp::Send start RANDOM to AM_DRIVER.");
                }

                state = HVA_TRK_STATE_RANDOM;
                startedAt = ros::Time::now();
            }
            break;

        case HVA_TRK_STATE_RANDOM:

            mowingTime = ros::Time::now() - startedAt;

            if (inCS)
            {
                ROS_INFO("TrackApp::Change to IN_CS STATE as mower is in CS.");
                state = HVA_TRK_STATE_IN_CS;
            }
            else if (!randomMode)
            {
                ROS_INFO("TrackApp::Change to IDLE STATE as mower is not mowing(RANDOM MODE).");
                state = HVA_TRK_STATE_IDLE;
            }
            else if (mowingTime > wantedMowingTime)
            {
                ROS_INFO("TrackApp::Change to FINISHED STATE as mower is done mowing(TIME).");

                if (operationalMode == AM_OP_MODE_CONNECTED_SIMULATOR)
                {
                    std_msgs::UInt16 modeCmd;
                    modeCmd.data = HVA_MODE_SIMPLE_RANDOM;
                    cmdPub.publish(modeCmd);
                    ROS_INFO("TrackApp::Sending HVA_MODE_SIMPLE_RANDOM (will toggle OFF) to STEERING.");

                    state = HVA_TRK_STATE_IN_CS;
                }
                else
                {
                    // REQUEST FINISHED to the mower (force it to CS)
                    std_msgs::UInt16 modeCmd;
                    modeCmd.data = HVA_MODE_REQ_FINISH_ON;
                    cmdPub.publish(modeCmd);
                    ROS_INFO("TrackApp::Sending FINISH to AM_DRIVER.");
                    
                    state = HVA_TRK_STATE_FINISHED;
                }

            }
            break;

        case HVA_TRK_STATE_IN_CS:
            if (!inCS)
            {
                ROS_INFO("TrackApp::Change to IDLE STATE as mower has left the CS.");
                state = HVA_TRK_STATE_IDLE;
                startedAt = ros::Time::now();
            }
            break;

        case HVA_TRK_STATE_FINISHED:
            if (inCS)
            {
                ROS_INFO("TrackApp::Change to IN_CS mode as mower is in CS.");
                state = HVA_TRK_STATE_IN_CS;

                // Clear the FINISHED flag
                std_msgs::UInt16 modeCmd;
                modeCmd.data = HVA_MODE_REQ_FINISH_OFF;
                cmdPub.publish(modeCmd);
            }
            break;
        }
    }

    return true;
}
}
