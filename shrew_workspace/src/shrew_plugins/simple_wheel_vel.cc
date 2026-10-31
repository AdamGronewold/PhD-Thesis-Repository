#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

/////////////////////////////////////////////////
//EXECUTABLE USED TO PUBLISH VELOCITIES TO WHEELS

int main(int _argc, char **_argv)
{
	// Load gazebo as a client
	gazebo::client::setup(_argc, _argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Advertise the vel_cmd topic
	gazebo::transport::PublisherPtr velPub =
    node->Advertise<gazebo::msgs::Quaternion>("~/shrew_robot/wheels/cmd_vel");

	// Wait for a subscriber to connect to this publisher
	velPub->WaitForConnection();
	
	// Create a Quaternion message to send our velocity
	gazebo::msgs::Quaternion msg;

	// Set the velocity
	gazebo::msgs::Set(&msg, ignition::math::Quaternion(std::atof(_argv[1]), std::atof(_argv[2]), std::atof(_argv[3]), std::atof(_argv[4])));

	// Send the message
	velPub->Publish(msg);		
	
	// Make sure to shut everything down.
	gazebo::client::shutdown();
  
}
