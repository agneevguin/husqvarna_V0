#include <am_gazebo_wire/am_gazebo_wire.h>
#include <ros/ros.h>

#include <am_driver/Loop.h>
#include <am_driver/SensorStatus.h>
#include <boost/filesystem.hpp>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWire);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosWire::GazeboRosWire()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosWire::~GazeboRosWire()
{

}

void GazeboRosWire::ExtractImageData(const std::string& filename,loopData& loop )
{

	common::Image* theImage;
	theImage = new common::Image(filename);
	loop.imgWidth = theImage->GetWidth();
	loop.imgHeight = theImage->GetHeight();
	ROS_INFO("Extracting Image data from: filename=%s", theImage->GetFilename().c_str());

	unsigned char *bitmap;
	// Set the bitmap to NULL (will have GetRGBData to allocate it...)
	bitmap = NULL;
	unsigned int size = 0;
	theImage->GetRGBData(&bitmap, size);


	unsigned int scanWidth = size / theImage->GetHeight();
	unsigned int bytesPerPixel = scanWidth / theImage->GetWidth();

	loop.magField.clear();

	loop.magField.reserve(size/bytesPerPixel);

	for (int i = 0; i < size; i += bytesPerPixel)
	{
		loop.magField.push_back((int16_t)((bitmap[i + 2] << 8) | (bitmap[i + 1])));
	}
	delete[] bitmap;
	delete theImage;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosWire::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

	// Make sure the ROS node for Gazebo has already been initalized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libam_gazebo_wire.so' in the gazebo_ros package)");
		return;
	}

	ROS_INFO("WIRE: am_gazebo_wire loaded...");

	theWire = _parent;
	theWorld = _parent->GetWorld();

	rosStarted = false;

	std::string gardenLinkName = "garden";

	theGardenLink = theWire->GetLink(gardenLinkName);
	if (!theGardenLink)
	{
		ROS_WARN("WIRE plugin warning: No garden link found...");
		return;
	}

	gardenPose = theGardenLink->GetWorldPose();
	std::cout << "Garden pos: " << gardenPose.pos.x << ", " << gardenPose.pos.y << ", " << gardenPose.pos.z << std::endl;


	sdf::ElementPtr plugin = _parent->GetSDF()->GetElement("plugin");
	if (!plugin)
	{
		ROS_WARN("WIRE plugin warning: No plugin element found...");
	}


	//Read all loop nodes and extract loopdata struct
	if(plugin->HasElement("loop"))
	{

		sdf::ElementPtr myLoop = plugin->GetElement("loop");
		while(myLoop!=NULL)
		{
			loopData tmp;
			tmp.name = myLoop->GetElement("name")->GetValue()->GetAsString();



			myLoop->GetElement("mapHeight")->GetValue()->Get<double>(tmp.mapHeight);
			myLoop->GetElement("mapWidth")->GetValue()->Get<double>(tmp.mapWidth);
			std::string wbf = myLoop->GetElement("wireMapName")->GetValue()->GetAsString();
			if(boost::filesystem::path(wbf).extension()== boost::filesystem::path(".bin"))
			{
				ExtractBinData(wbf,tmp);
			}
			else
			{
				ExtractImageData(wbf,tmp);
			}

			ROS_INFO("Loop:%s WireMapName=%s h=%f w=%f",
					tmp.name.c_str(), wbf.c_str(),tmp.mapHeight,tmp.mapWidth);
			loopVec.push_back(tmp);
			myLoop = myLoop->GetNextElement("loop");
		}

	}
	else
	{
		ROS_WARN("WIRE: No loops defined!" );
	}
	//	ROS_INFO("Loopvec0 mapheight:%f mapwidth:%f imgX:%d imgY:%d size%d calc:%d",
	//			loopVec[0].mapHeight,loopVec[0].mapWidth,loopVec[0].imgWidth,loopVec[0].imgHeight,
	//			(int)loopVec[0].magField.size(), loopVec[0].imgWidth*loopVec[0].imgHeight);


	//
	//	wireBitmapName = plugin->GetElement("wireMapName")->GetValue()->GetAsString();
	//
	//	ROS_INFO("WIRE: WireBitMapName=%s", wireBitmapName.c_str());
	//
	//	// Get the map width/height
	//	plugin->GetElement("mapHeight")->GetValue()->Get<double>(loopVec[0].mapHeight);
	//	plugin->GetElement("mapWidth")->GetValue()->Get<double>(loopVec[0].mapWidth);
	//
	//	ROS_INFO("WIRE: map size: (%f, %f)", loopVec[0].mapWidth, loopVec[0].mapHeight);
	//
	//
	//
	//
	//
	//	ExtractImageData(wireBitmapName,loopVec[0].magField);
	//
	//	loopVec[0].magField.clear();
	//	ExtractBinData("/home/p25/tmp/awdA0.bin",loopVec[0].magField);
	//
	//	loopVec[0].imgWidth = (unsigned int)sqrt(loopVec[0].magField.size());
	//	loopVec[0].imgHeight = loopVec[0].imgWidth;

	// connect Update function
	//updateTimer.setUpdateRate(35.0);
	//updateTimer.Load(world, _sdf);
	//updateConnection = updateTimer.Connect(boost::bind(&GazeboRosUwbRange::Update, this));	

	// listen to the update event (broadcast every simulation iteration)
	ROS_INFO("Done ParsingMaps");
	updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosWire::Update, this ) );

}


////////////////////////////////////////////////////////////////////////////////
// Setup all things related to ROS
void GazeboRosWire::InitPlugin()
{
	//
	// Get the mower link base...
	//

	theMower1 = theWorld->GetModel("robot1");
	theMower2 = theWorld->GetModel("robot2");
	theMower3 = theWorld->GetModel("robot3");
	if (!theMower1 || !theMower2 || !theMower3)
	{
		//ROS_WARN("WIRE plugin warning: No automower model!\n");
		return;
	}

	sdf::ElementPtr mowerSdf1 = theMower1->GetSDF();
	sdf::ElementPtr mowerSdf2 = theMower2->GetSDF();
	sdf::ElementPtr mowerSdf3 = theMower3->GetSDF();

	// Init gazebo_ros
	gazebo_ros1_ = GazeboRosPtr ( new GazeboRos ( theMower1, mowerSdf1, "WIRE_Automower" ) );
	gazebo_ros2_ = GazeboRosPtr ( new GazeboRos ( theMower2, mowerSdf1, "WIRE_Automower" ) );
	gazebo_ros3_ = GazeboRosPtr ( new GazeboRos ( theMower3, mowerSdf1, "WIRE_Automower" ) );

	// Make sure the ROS node for Gazebo has already been initialized
	gazebo_ros1_->isInitialized();
	gazebo_ros2_->isInitialized();
	gazebo_ros3_->isInitialized();

	// Publishers
	loopPub1 = gazebo_ros1_->node()->advertise<am_driver::Loop>("loop", 1);
	loopPub2 = gazebo_ros2_->node()->advertise<am_driver::Loop>("loop", 1);
	loopPub3 = gazebo_ros3_->node()->advertise<am_driver::Loop>("loop", 1);
	//statusPub = gazebo_ros_->node()->advertise<am_driver::SensorStatus>("/sensor_status", 10);

	//
	// Get the mower link base...
	//

	std::string link_name = "base_link";
	robotLink1 = theMower1->GetLink(link_name);
	robotLink2 = theMower2->GetLink(link_name);
	robotLink3 = theMower3->GetLink(link_name);

	if (!robotLink1 || !robotLink2 || !robotLink3)
	{
		ROS_FATAL("WIRE plugin error: link %s does not exist\n", link_name.c_str());
		return;
	}

	link_name = "loop_front_center";
	frontCenterLink1 = theMower1->GetLink(link_name);
	frontCenterLink2 = theMower2->GetLink(link_name);
	frontCenterLink3 = theMower3->GetLink(link_name);

	if (!frontCenterLink1 || !frontCenterLink2 || !frontCenterLink3)
	{
		ROS_FATAL("WIRE plugin error: link %s does not exist\n", link_name.c_str());
	}

	link_name = "loop_front_right";
	frontRightLink1 = theMower1->GetLink(link_name);
	frontRightLink2 = theMower2->GetLink(link_name);
	frontRightLink3 = theMower3->GetLink(link_name);

	if (!frontRightLink1 || !frontRightLink2 || !frontRightLink3)
	{
		ROS_FATAL("WIRE plugin error: link %s does not exist\n", link_name.c_str());
	}

	link_name = "loop_rear_right";
	rearRightLink1 = theMower1->GetLink(link_name);
	rearRightLink2 = theMower2->GetLink(link_name);
	rearRightLink3 = theMower3->GetLink(link_name);

	if (!rearRightLink1 || !rearRightLink2 || !rearRightLink3)
	{
		ROS_FATAL("WIRE plugin error: link %s does not exist\n", link_name.c_str());
	}

	link_name = "loop_rear_left";
	rearLeftLink1 = theMower1->GetLink(link_name);
	rearLeftLink2 = theMower2->GetLink(link_name);
	rearLeftLink3 = theMower3->GetLink(link_name);

	if (!rearLeftLink1 || !rearLeftLink2 || !rearLeftLink3)
	{
		ROS_FATAL("WIRE plugin error: link %s does not exist\n", link_name.c_str());
	}
	/*
	physics::Link_V links = theMower->GetLinks();
	for (unsigned int i = 0; i < links.size(); i++)
	{
		std::cout << "link name: " << links.at(i)->GetName() << " pose: " << links.at(i)->GetWorldPose().pos << std::endl;
	}
	 */
	rosStarted = true;
}


int GazeboRosWire::GetLoopValue(math::Pose pose,const loopData& loop)
{
	// The "relative pose" of the sensor is...
	double xpos = pose.pos.x - gardenPose.pos.x;
	double ypos = pose.pos.y - gardenPose.pos.y;

	//std::cout << "AM (relative) pos: " << xpos << ", " << ypos << ", " << zpos << std::endl;
	//X and y i meter
	// Convert coords from robot to bitmap...
	unsigned int imgXpos = (loop.imgWidth/2 + loop.imgWidth * xpos/(loop.mapWidth));
	unsigned int imgYpos = (loop.imgHeight/2 + loop.imgHeight * ypos/(loop.mapHeight));

	//	std::cout << "AM (map) pos: " << imgXpos << ", " << imgYpos << std::endl;

	// Invert y-axis
	imgYpos = loop.imgHeight - imgYpos;

	unsigned int pos=imgYpos*loop.imgWidth+imgXpos;

	if(pos<loop.magField.size())
	{
		return loop.magField[pos];
	}
	return -10000;
}



////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosWire::Update()
{
	// Done here so that we can detect when the automower object is available...
	if (!rosStarted)
	{
		InitPlugin();
	}
	else
	{

		math::Pose frontC;
		math::Pose frontR;
		math::Pose rearR;
		math::Pose rearL;
		for (int iterator = 0; iterator < 3; iterator++) {
			switch (iterator) {
				case 0: 
					frontC = frontCenterLink1->GetWorldPose();
					frontR = frontRightLink1->GetWorldPose();
					rearR = rearRightLink1->GetWorldPose();
					rearL  = rearLeftLink1->GetWorldPose();
					break;
				case 1:
					frontC = frontCenterLink2->GetWorldPose();
					frontR = frontRightLink2->GetWorldPose();
					rearR = rearRightLink2->GetWorldPose();
					rearL  = rearLeftLink2->GetWorldPose();
					break;
				case 2:
					frontC = frontCenterLink3->GetWorldPose();
					frontR = frontRightLink3->GetWorldPose();
					rearR = rearRightLink3->GetWorldPose();
					rearL  = rearLeftLink3->GetWorldPose();
					break;
			}

			for(int i=0;i<loopVec.size();i++)
			{
				if(loopVec[i].name=="A0")
				{

					loop.frontCenter = GetLoopValue(frontC,loopVec[i]);
					loop.A0.frontCenter=loop.frontCenter;

					loop.frontRight = GetLoopValue(frontR,loopVec[i]);
					loop.A0.frontRight=loop.frontRight;

					loop.rearRight = GetLoopValue(rearR,loopVec[i]);
					loop.A0.rearRight=loop.rearRight;

					loop.rearLeft = GetLoopValue(rearL,loopVec[i]);
					loop.A0.rearLeft=loop.rearLeft;
				}
				if(loopVec[i].name=="N")
				{
					loop.N.frontCenter = GetLoopValue(frontC,loopVec[i]);
					loop.N.frontRight = GetLoopValue(frontR,loopVec[i]);
					loop.N.rearRight = GetLoopValue(rearR,loopVec[i]);
					loop.N.rearLeft = GetLoopValue(rearL,loopVec[i]);

				}
				if(loopVec[i].name=="F")
				{
					loop.F.frontCenter = GetLoopValue(frontC,loopVec[i]);
					loop.F.frontRight = GetLoopValue(frontR,loopVec[i]);
					loop.F.rearRight = GetLoopValue(rearR,loopVec[i]);
					loop.F.rearLeft = GetLoopValue(rearL,loopVec[i]);

				}
				if(loopVec[i].name=="G1")
				{
					loop.G1.frontCenter = GetLoopValue(frontC,loopVec[i]);
					loop.G1.frontRight = GetLoopValue(frontR,loopVec[i]);
					loop.G1.rearRight = GetLoopValue(rearR,loopVec[i]);
					loop.G1.rearLeft = GetLoopValue(rearL,loopVec[i]);

				}
				if(loopVec[i].name=="G2")
				{
					loop.G2.frontCenter = GetLoopValue(frontC,loopVec[i]);
					loop.G2.frontRight = GetLoopValue(frontR,loopVec[i]);
					loop.G2.rearRight = GetLoopValue(rearR,loopVec[i]);
					loop.G2.rearLeft = GetLoopValue(rearL,loopVec[i]);

				}
				if(loopVec[i].name=="G3")
				{
					loop.G3.frontCenter = GetLoopValue(frontC,loopVec[i]);
					loop.G3.frontRight = GetLoopValue(frontR,loopVec[i]);
					loop.G3.rearRight = GetLoopValue(rearR,loopVec[i]);
					loop.G3.rearLeft = GetLoopValue(rearL,loopVec[i]);

				}
			}

			// Treat as "in" or "out" of the loop
			loop.header.stamp = ros::Time::now();
			loop.header.frame_id = "loop_base";

			// Publish the loop
			switch (iterator) {
				case 0: loopPub1.publish(loop); break;
				case 1: loopPub2.publish(loop); break;
				case 2: loopPub3.publish(loop); break;
			}

			/*		MOVED TO am_gazebo_sensors !!!"
			// Do the test if we are in/out
			am_driver::SensorStatus status;
			status.sensorStatus = 0x0;

			if (loop.frontCenter < 0)
			{
				status.sensorStatus |= 0x02;
			}
			if (loop.rearRight < 0)
			{
				status.sensorStatus |= 0x02;
			}
			if (loop.rearLeft < 0)
			{
				status.sensorStatus |= 0x02;
			}
			statusPub.publish(status);
			 */
		}
	}
}


void GazeboRosWire::ExtractBinData(const std::string& filename,loopData& loop)
{


	ROS_INFO("FindfileUri binary data from: filename=%s", common::SystemPaths::Instance()->FindFileURI(filename).c_str());


	std::ifstream ifs(common::SystemPaths::Instance()->FindFileURI(filename).c_str(), std::ios::binary|std::ios::ate );
	std::ifstream::pos_type pos =ifs.tellg();

	loop.magField.clear();
	//Allocate room for the data
	loop.magField.resize(pos/sizeof(int16_t));

	ifs.seekg(0,std::ios::beg);
	ifs.read((char*)(&(loop.magField[0])),pos);


	loop.imgWidth = (unsigned int)sqrt(loop.magField.size());
	loop.imgHeight = loop.imgWidth;


}

}
