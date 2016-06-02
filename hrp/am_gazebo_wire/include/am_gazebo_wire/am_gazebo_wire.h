#ifndef GAZEBO_ROS_WIRE_H
#define GAZEBO_ROS_WIRE_H

#include <ros/ros.h>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <am_driver/Loop.h>
#include <am_driver/LoopData.h>

namespace gazebo
{
	
class GazeboRosWire : public ModelPlugin
{
public:
	/// \brief Constructor
	GazeboRosWire();

	/// \brief Destructor
	virtual ~GazeboRosWire();

	/// \brief Load the controller
	void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

	/// \brief Update the controller
protected: 
	virtual void Update();

private:
	typedef struct
		{
			double mapWidth;
			double mapHeight;
			unsigned int imgWidth;
			unsigned int imgHeight;
			std::vector<int16_t> magField;
			std::string name;
		} loopData;

	void InitPlugin();
	bool rosStarted;
	
	int GetLoopValue(math::Pose pose,const loopData& loop);

	gazebo::GazeboRosPtr gazebo_ros1_;
	gazebo::GazeboRosPtr gazebo_ros2_;
	gazebo::GazeboRosPtr gazebo_ros3_;

	// Simulation objects
	physics::WorldPtr theWorld;
	physics::ModelPtr theWire;
	physics::ModelPtr theMower1;
	physics::ModelPtr theMower2;
	physics::ModelPtr theMower3;
	physics::LinkPtr  theGardenLink;
	
	// Used for loop/wire calculations from and Image
	math::Pose gardenPose;
	std::string wireBitmapName;

	//FrKa
	std::vector<loopData> loopVec;


	int16_t GetMagValue(math::Pose pose);
	void ExtractBinData(const std::string& filename,loopData& loop);
	void ExtractImageData(const std::string& filename,loopData& loop);

	// The ROS loop messages
	ros::Publisher loopPub1;
	ros::Publisher loopPub2;
	ros::Publisher loopPub3;
	am_driver::Loop loop;
	
	// Internal links to use for simulation
	physics::LinkPtr robotLink1;
	physics::LinkPtr robotLink2;
	physics::LinkPtr robotLink3;
	physics::LinkPtr frontCenterLink1;
	physics::LinkPtr frontCenterLink2;
	physics::LinkPtr frontCenterLink3;
	physics::LinkPtr frontRightLink1;
	physics::LinkPtr frontRightLink2;
	physics::LinkPtr frontRightLink3;
	physics::LinkPtr rearRightLink1;
	physics::LinkPtr rearRightLink2;
	physics::LinkPtr rearRightLink3;
	physics::LinkPtr rearLeftLink1;
	physics::LinkPtr rearLeftLink2;
	physics::LinkPtr rearLeftLink3;
	
	event::ConnectionPtr updateConnection;
};
	
}

#endif
