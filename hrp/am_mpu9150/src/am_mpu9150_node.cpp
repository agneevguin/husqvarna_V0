/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 * File is borrowed/inspired by:
 * This file is part of linux-mpu9150
 * Copyright (c) 2013 Pansenti, LLC
 */

#include <sstream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

// Include C-source into CPP
#ifdef __cplusplus
    extern "C" {
#endif
        #include "mpu9150.h"
        #include "linux_glue.h"
        #include "local_defaults.h"
        #include "quaternion.h"
		
		#include "eMPL/inv_mpu_dmp_motion_driver.h"

#ifdef __cplusplus
    }
#endif

// FSR 2000
static double gyroScaleX = M_PI / (16.40 * 180.0);
static double gyroScaleY = M_PI / (16.40 * 180.0);
static double gyroScaleZ = M_PI / (16.40 * 180.0);

/*
// FSR 1000
static double gyroScaleX = M_PI / (32.8 * 180.0);
static double gyroScaleY = M_PI / (32.8 * 180.0);
static double gyroScaleZ = M_PI / (32.8 * 180.0);
*/

/*
// FSR 500
static double gyroScaleX = M_PI / (62.5 * 180.0);
static double gyroScaleY = M_PI / (62.5 * 180.0);
static double gyroScaleZ = M_PI / (62.5 * 180.0);
*/

/*
// FSR 250
static double gyroScaleX = M_PI / (131.0 * 180.0);
static double gyroScaleY = M_PI / (131.0 * 180.0);
static double gyroScaleZ = M_PI / (131.0 * 180.0);
*/

// Prototypes
int load_calibration(int mag, std::string cal_file);


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "am_mpu9150_node");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1);
	ros::Publisher imu_euler_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu_euler", 1);
	
	ros::Rate loop_rate(50);
	
	sensor_msgs::Imu imu;
	imu.header.frame_id = "imu";
	
	geometry_msgs::Vector3Stamped euler;
	euler.header.frame_id = "imu_euler";

	// Init the sensor the values are hardcoded at the local_defaults.h file
	int opt, len;
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	int verbose = 0;
	
	std::string accelFile;
	std::string magFile;

	// Parameters from ROS
	ros::NodeHandle n_private("~");
	int def_i2c_bus = 2;
	n_private.param("i2cbus", i2c_bus, def_i2c_bus);
            if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS)
	{
		ROS_ERROR("am_mpu9150: Imu Bus problem");
		return -1;
	}
	
	int def_sample_rate = 50;
	n_private.param("samplerate", sample_rate, def_sample_rate);
	if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE)
	{
		ROS_ERROR("am_mpu9150: Sample rate problem");
		return -1;
	}
	loop_rate = sample_rate;

	int def_yaw_mix_factor = 0;
	n_private.param("yawmixfactor", yaw_mix_factor, def_yaw_mix_factor);
	if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
	{
		ROS_ERROR("am_mpu9150: yawmixfactor problem");
		return -1;
	}
	
	std::string defAccelFile = "./accelcal.txt";
	n_private.param("accelfile", accelFile, defAccelFile);

	std::string defMagFile = "./magcal.txt";
	n_private.param("magfile", magFile, defMagFile);
	
	
	unsigned long loop_delay;
	mpudata_t mpu;

    // Initialize the MPU-9150
	mpu9150_set_debug(verbose);
	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
	{
		ROS_ERROR("am_mpu9150::Failed init!");
		exit(1);
	}
	
	//load_calibration(0, accelFile);
	//load_calibration(1, magFile);
	
	memset(&mpu, 0, sizeof(mpudata_t));

	vector3d_t e;
	quaternion_t q;

	while (ros::ok())
	{
		std::stringstream ss;
		if (mpu9150_read(&mpu) == 0) 
		{
			// ROTATE The angles correctly...
			quaternionToEuler(mpu.fusedQuat, e);
			e[VEC3_Z] = -e[VEC3_Z];
			eulerToQuaternion(e, q);
			
			// FUSED
			imu.header.stamp = ros::Time::now();
			imu.orientation.x = q[QUAT_X];
			imu.orientation.y = q[QUAT_Y];
			imu.orientation.z = q[QUAT_Z];
			imu.orientation.w = q[QUAT_W];

			// GYRO
			double gx = mpu.rawGyro[VEC3_X];
			double gy = mpu.rawGyro[VEC3_Y];
			double gz = mpu.rawGyro[VEC3_Z];

			gx = gx * gyroScaleX;
			gy = gy * gyroScaleY;
			gz = gz * gyroScaleZ;

			imu.angular_velocity.x = gx;
			imu.angular_velocity.y = gy;
			imu.angular_velocity.z = gz;

			// ACCEL
			imu.linear_acceleration.x = mpu.calibratedAccel[VEC3_X];
			imu.linear_acceleration.y = mpu.calibratedAccel[VEC3_Y];
			imu.linear_acceleration.z = mpu.calibratedAccel[VEC3_Z];
			
			// EULER
			euler.header.stamp = imu.header.stamp;
			euler.vector.x = mpu.fusedEuler[VEC3_Y];
			euler.vector.y = mpu.fusedEuler[VEC3_X];
			euler.vector.z = -mpu.fusedEuler[VEC3_Z];
			
			// Publish the messages
			imu_pub.publish(imu);
			imu_euler_pub.publish(euler);
			
			//ROS_INFO("y=%f, p=%f, r=%f", euler.vector.z, euler.vector.y, euler.vector.x);

			
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

int load_calibration(int mag, std::string cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	f = fopen(cal_file.c_str(), "r");
	
	if (!f) 
	{
		ROS_ERROR("am_mpu9150::load_calibration:: Failed to open file %s", cal_file.c_str());
		perror("File problem:");
		return -1;
	}
	
	memset(buff, 0, sizeof(buff));
	
	for (i = 0; i < 6; i++) 
	{
		if (!fgets(buff, 20, f)) 
		{
			ROS_ERROR("am_mpu9150::load_calibration::Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) 
		{
			ROS_ERROR("am_mpu9150::load_calibration::Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
	{
		ROS_ERROR("am_mpu9150::load_calibration:: Failed to load calibration!");
		return -1;
	}

	// Store calibration
	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);
	
	if (mag) 
	{
		mpu9150_set_mag_cal(&cal);
	}
	else 
	{
		mpu9150_set_accel_cal(&cal);
	}

	return 0;
}
