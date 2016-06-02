#ifndef GAZEBO_ROS_UWBRANGE_HH
#define GAZEBO_ROS_UWBRANGE_HH

#include <ros/ros.h>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/math/Vector3.hh"
#include <am_gazebo_uwbrange/update_timer.h>

namespace gazebo
{

class Beacon
{
public:
    std::string id;
    double x;
    double y;
    double z;
    double noise;
    double scale;
    double aDelayNs;
    double maxRange;
};

class GazeboRosUwbRange : public ModelPlugin
{
    /// \brief Constructor
public:
    GazeboRosUwbRange();

    /// \brief Destructor
public:
    virtual ~GazeboRosUwbRange();

    /// \brief Load the controller
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
protected:
    virtual void Update();

private:
    gazebo::GazeboRosPtr gazebo_ros_;
    gazebo::GazeboRosPtr gazebo_ros_automower;

    physics::ModelPtr parent;
    physics::WorldPtr world;
    physics::ModelPtr theMower;
    
    gazebo::physics::PhysicsEnginePtr engine;
    gazebo::physics::RayShapePtr ray;
    math::Vector3 start;
    math::Vector3 end;
    
    void InitPlugin();
    void UpdateMultiRangeLegacy();
    void UpdateMultiRange();
    // Parameters
    int numBeacons;
    int index;
    int updateRate;
    double robotADelayNs;
       

    sdf::ElementPtr mapSdf;
    physics::ModelPtr myParent;
    std::vector<Beacon*> beacons;

    ros::Publisher multiRangePub;
    ros::Publisher rangePub;
    ros::Subscriber modeSub;
    bool rosStarted;
    physics::LinkPtr link;

    event::ConnectionPtr updateConnection;
    gazebo::UpdateTimer updateTimer;
};
}

#endif
