#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>

std::string robot_name;
ros::Time last_time;
void poseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){ 

	//make sure we are sending this at a new time
	if (ros::Time::now()!=last_time) {
		static tf2_ros::TransformBroadcaster broadcaster;
		geometry_msgs::TransformStamped transformStamped [msg->name.size()];
		
		for (int i = 0; i < msg->name.size(); i++) {
			transformStamped[i].header.stamp = ros::Time::now();
			transformStamped[i].header.frame_id = "world";
			transformStamped[i].child_frame_id = msg->name[i];
			transformStamped[i].transform.translation.x = msg->pose[i].position.x;
			transformStamped[i].transform.translation.y = msg->pose[i].position.y;
			transformStamped[i].transform.translation.z = msg->pose[i].position.z;
			transformStamped[i].transform.rotation.x = msg->pose[i].orientation.x;
			transformStamped[i].transform.rotation.y = msg->pose[i].orientation.y;
			transformStamped[i].transform.rotation.z = msg->pose[i].orientation.z;
			transformStamped[i].transform.rotation.w = msg->pose[i].orientation.w;
		}
		broadcaster.sendTransform(transformStamped[0]); 
		broadcaster.sendTransform(transformStamped[1]);
		broadcaster.sendTransform(transformStamped[2]);
		last_time=ros::Time::now();
	}
	else {
		;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "full_sim_world_tf2_broadcaster");
	ros::NodeHandle private_node("~");
	if (! private_node.hasParam("shrew_robot")) {
		if (argc != 2) {
			ROS_ERROR("Need robot name as argument"); 
			return -1;
		};
		robot_name = argv[1];
	}
	else {
		private_node.getParam("shrew_robot", robot_name);
	}
    last_time=ros::Time(0);
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("/gazebo/model_states", 10, &poseCallback);
	
	ros::spin();
	return 0;
};
