#include <am_gazebo_uwbrange/am_gazebo_uwbrange.h>
#include <ros/ros.h>

#include <boost/lexical_cast.hpp>

#include <am_driver/Range.h>
#include <am_driver/MultiRange.h>


#define SPEED_OF_LIGHT_NS 0.299792458
namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosUwbRange);


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosUwbRange::GazeboRosUwbRange()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosUwbRange::~GazeboRosUwbRange()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosUwbRange::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo "
                         "system plugin 'libam_gazebo_uwbrange.so' in the gazebo_ros package)");
        return;
    }

    ROS_INFO("UWBRange: am_gazebo_uwbrange loaded...");
    mapSdf = _sdf;
    myParent = _parent;
    parent = _parent;
    world = _parent->GetWorld();
    if (!world)
    {
        ROS_WARN("UWB plugin warning: No world!\n");
        return;
    }
    rosStarted = false;

    engine = world->GetPhysicsEngine();
    engine->InitForThread();        
    ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
              engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
    // Default, will change after ros init
    updateRate = 20;
    // connect Update function
    updateTimer.setUpdateRate(updateRate * 1.0);
    updateTimer.Load(world, _sdf);
    updateConnection = updateTimer.Connect(boost::bind(&GazeboRosUwbRange::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Setup all things related to ROS
void GazeboRosUwbRange::InitPlugin()
{

    theMower = world->GetModel("automower");
    if (!theMower)
    {
        ROS_WARN("UWB plugin warning: No automower model!\n");
        return;
    }

    sdf::ElementPtr mowerSdf = theMower->GetSDF();

    // Init gazebo_ros
    gazebo_ros_ = GazeboRosPtr(new GazeboRos(myParent, mapSdf, "UWBRange"));

    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    /* Get update rate parameter */
    gazebo_ros_->getParameter<int>(updateRate, "updateRate", 20);
    ROS_INFO("UWBRange: updateRate=%d", updateRate);
    updateTimer.setUpdateRate(updateRate * 1.0);

    gazebo_ros_->getParameter<double>(robotADelayNs, "robotADelayNs", 0.0);
    ROS_INFO("UWBRange: robotADelayNs=%f", robotADelayNs);

    // Get the mower link base...
    std::string link_name = "base_link";
    link = theMower->GetLink(link_name);

    if (!link)
    {
        ROS_FATAL("UWBRange plugin error: link %s does not exist\n", link_name.c_str());
        return;
    }

    // Publisher
    rangePub = gazebo_ros_->node()->advertise<am_driver::Range>("/uwb_gz", 100);
    
    // LOAD BEACONS
    gazebo_ros_->getParameter<int>(numBeacons, "numBeacons", 0);
    ROS_INFO("UWBRange: Num beacons=%d", numBeacons);
    if (numBeacons > 0)
    {
        // Load the beacons...
        for (int i = 1; i <= numBeacons; i++)
        {
            Beacon* aBeacon = new Beacon();

            std::string paramBase = "beacon" + boost::lexical_cast<std::string>(i);

            // Load ID
            std::string param = paramBase + "id";
            std::string defId = "DECA-" + boost::lexical_cast<std::string>(i);
            gazebo_ros_->getParameter<std::string>(aBeacon->id, param.c_str(), defId);

            // Load X, Y, Z
            param = paramBase + "x";
            gazebo_ros_->getParameter<double>(aBeacon->x, param.c_str(), 0.0);
            param = paramBase + "y";
            gazebo_ros_->getParameter<double>(aBeacon->y, param.c_str(), 0.0);
            param = paramBase + "z";
            gazebo_ros_->getParameter<double>(aBeacon->z, param.c_str(), 0.0);

            // NOISE
            param = paramBase + "noise";
            gazebo_ros_->getParameter<double>(aBeacon->noise, param.c_str(), 0.1);

            // SCALE
            param = paramBase + "scale";
            gazebo_ros_->getParameter<double>(aBeacon->scale, param.c_str(), 1.0);

            // Antenna Delay
            param = paramBase + "aDelayNs";
            gazebo_ros_->getParameter<double>(aBeacon->aDelayNs, param.c_str(), 0.0);

            // Max Range
            param = paramBase + "maxRange";
            gazebo_ros_->getParameter<double>(aBeacon->maxRange, param.c_str(), 100.0);

            ROS_INFO("Loaded Beacon: %s - (%f, %f, %f) noise=%f, scale=%f, aDelay=%f, maxRange=%f",
                     aBeacon->id.c_str(),
                     aBeacon->x,
                     aBeacon->y,
                     aBeacon->z,
                     aBeacon->noise,
                     aBeacon->scale,
                     aBeacon->aDelayNs,
                     aBeacon->maxRange);

            beacons.push_back(aBeacon);
        }
    }

    // listen to the update event (broadcast every simulation iteration)
    // updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosUwbRange::Update, this ) );

    index = 0;
    rosStarted = true;
}

void GazeboRosUwbRange::UpdateMultiRange()
{
    /* Include Robot Pose in the array*/
    math::Pose pose = link->GetWorldPose();

    double xpos = pose.pos.x;
    double ypos = pose.pos.y;
    double zpos = pose.pos.z;

    Beacon* b1;
    Beacon* b2;

    double dist;
    std::string entityName;
        
    int rangeIndex = 0;

    am_driver::Range rangeMsg;
    rangeMsg.header.stamp = ros::Time::now();
    rangeMsg.header.frame_id = "uwb_base";

    /* Send all interbeacon ranges*/
    for (int i = 0; i < numBeacons - 1; i++)
    {
        b1 = beacons[i];

        for (int j = i + 1; j < numBeacons; j++)
        {

            b2 = beacons[j];

            // Calculate distance
            double dx = b1->x - b2->x;
            double dy = b1->y - b2->y;
            double dz = b1->z - b2->z;

            double range = sqrt(dx * dx + dy * dy + dz * dz);

            start.x = b1->x;
            start.y = b1->y;
            start.z = b1->z + 1.0;
            
            end.x = b2->x;
            end.y = b2->y;
            end.z = b2->z + 1.0;
            
            ray->SetPoints(start, end);
            ray->GetIntersection(dist, entityName);

            //~ std::cout << b1->id << "->" <<  b2->id << ": " <<  range << std::endl;
            /* Only publish if within range and line of sight */
            if (range <= b2->maxRange && range <= b1->maxRange && entityName.empty())
            {
                // Add noise
                double err = ((rand() % 1000) / 1000.0) * b1->noise - b1->noise / 2.0;
                range = range + err;

                // Add antenna delay, if counted as air time distance increases! Should be handled at a lower level,
                // only included for test of calibration
                double delayErr = ((b1->aDelayNs + b2->aDelayNs) / 2) * SPEED_OF_LIGHT_NS;
                range += delayErr;

                rangeMsg.fromId = b1->id;
                rangeMsg.toId = b2->id;
                rangeMsg.range = range;
                // Add scale
                range = range * b1->scale;
                rangeMsg.range = range;

                rangePub.publish(rangeMsg);
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosUwbRange::Update()
{
    if (!rosStarted)
    {
        InitPlugin();
    }
    else
    {
        if (numBeacons <= 0)
        {
            return;
        }
        math::Pose pose = link->GetWorldPose();

        double xpos = pose.pos.x;
        double ypos = pose.pos.y;
        double zpos = pose.pos.z;
        double dist;
        std::string entityName;
        // for (int i=0; i<numBeacons; i++)
        {
            Beacon* b = beacons[index];

            // Calculate distance to beacon
            double dx = b->x - xpos;
            double dy = b->y - ypos;
            double dz = b->z - zpos;

            double range = sqrt(dx * dx + dy * dy + dz * dz);

            // Check Line of sight
            start.x = b->x;
            start.y = b->y;
            start.z = b->z + 1.0;
            
            end.x = xpos;
            end.y = ypos;
            end.z = zpos + 1.0;
            ray->SetPoints(start, end);
            ray->GetIntersection(dist, entityName);
            
            if (entityName.empty())
            {
                /* Only publish if within range */
                if (range <= b->maxRange)
                {
                    // Add noise
                    double err = ((rand() % 1000) / 1000.0) * b->noise - b->noise / 2.0;
                    range = range + err;

                    // Add scale
                    range = range * b->scale;

                    // Add antenna delay, if counted as air time distance increases! Should be handled at a lower level,
                    // only included for test of calibration
                    double delayErr = ((robotADelayNs + b->aDelayNs) / 2) * SPEED_OF_LIGHT_NS;
                    range += delayErr;

                    am_driver::Range rangeMsg;

                    rangeMsg.header.frame_id = "uwb_base";
                    rangeMsg.header.stamp = ros::Time::now();
                    rangeMsg.fromId = "DECA0100-100";
                    rangeMsg.toId = b->id;
                    rangeMsg.range = range;
                    rangePub.publish(rangeMsg);
                }
            }
            else
            {
                //ROS_WARN("Beacon out of sight %fm", dist);
            }
            index++;
            if (index >= numBeacons)
            {
                index = 0;
                UpdateMultiRange();
            }
        }
    }
}
}
