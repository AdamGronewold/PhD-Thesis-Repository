/*
 * \file  shrew_fixed_joint_plugin.cc
 *
 * \brief Plugin used to modify a shrew_robot_fixed_joint in simulation and integrate the simulation with ROS.
 * 
 * \author  Adam Gronewold (Adam.M.Gronewold.TH@Dartmouth.edu)
 *
 * $ Id: 05/10/2021 11:23:40 AM Gronewold $
 *
 * Sources and personal notes:
 * http: //gazebosim.org/tutorials?tut=set_velocity
 * http: //gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin
 * http: //osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/classgazebo_1_1physics_1_1Model.html
 * https: //github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp
 * http: //docs.ros.org/en/jade/api/gazebo_plugins/html/classgazebo_1_1GazeboRos.html#a562895d4f6326d665de7914c4a819732
 * Subscribers, Publishers, ...
 * http: //gazebosim.org/tutorials?tut=topics_subscribed&cat=transport
 * 
 * Dynamic Elements
 * https:// answers.gazebosim.org//question/7082/using-sdf-tags-in-urdf-gazebo-extension-directly/
 *
 * To Determine when a model is in a region http://gazebosim.org/tutorials?tut=occupiedevent&cat=plugins
 * http ://gazebosim.org/tutorials?tut=joint_events&cat=
 * list gazebo topic running gz topic -l
 *
 * ROS Timing
 * http://wiki.ros.org/roscpp/Overview/Time
 */

////////////////////////////////////////////////////////////////////////////////////////////

//GAZEBO HEADERS
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/ode/ODEJoint.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/common/Plugin.hh>

//ROS HEADERS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

//OTHER HEADERS
#include <functional>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <sstream>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float32.h>
#include <sdf/sdf.hh>

//CUSTOM HEADERS
//#include <msgs/shrew_twist.pb.h>

////////////////////////////////////////////////////////////////////////////////////////////
namespace gazebo
{

	//typedef const boost::shared_ptr<const gazebo::msgs::Quaternion> ConstQuaternionPtr;

	//A GAZEBO-ROS TYPE PLUGIN CLASS INHERETED FROM MODELPLUGIN CLASS FOR INTERACTING WITH A SHREW ROBOT IN SIMULATION
	class ShrewFixedPlugin : public ModelPlugin 
	{	
		////////////////////////////////////////////////////////////////////////////////////
		enum wheelsEnum{FL=0, FR=1, BL=2, BR=3,}; //USED FOR OPERATIONS ON VECTORS OF JOINTS
		enum pidEnum{Kp=0, Ki=1, Kd=2,}; //USED FOR OPERATIONS ON VECTOR OF PID PARAMETERS
		
		////////////////////////////////////////////////////////////////////////////////////
		//CONSTRUCTOR
		public: ShrewFixedPlugin() : ModelPlugin()
		{	
    		std::cout << launchString << std::endl; //SOME FUN
		};
		
		////////////////////////////////////////////////////////////////////////////////////	
		//THE LOAD FUNCTION IS CALLED BY GAZEBO WHEN THE PLUGIN IS INSERTED INTO SIMULATION
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) //_parent is the reference to the model. _sdf is the reference to plugin included in the model file.
		{  			
			
			//SAFETY CHECKS AND PRINTS	
				GZ_ASSERT(_parent, "The parent pointer is null.\n");
				GZ_ASSERT(_sdf, "The sdf pointer is null.\n");
			
			//DECLARATIONS AND PRIVATE POINTERS
				this->model = _parent;
				this->sdf = _sdf;
				this->wheels_.resize(4);

			//FILL WHEEL PTR USED WHEN SETTING VELOCITIES
				this->fillWheelPtr();
			
			//FIX JOINT PROPERTIES NOT SET WHEN PARSING URDF
				this->setDynamProps();
										
			//MAKE THE PID OBJECT, USED WHEN SETTING WHEEL SPEEDS
				this->makePID();
				
			//INITIALIZE ROS, IF IT HAS NOT ALREADY BEEN DONE.
				if (!ros::isInitialized()) {
  					int argc = 0;
  					char **argv = NULL;
  					ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
				}

			//NODES
				//INITIALIZE GAZEBO NODE WITH NAME OF WORLD
				this->gazeboNode = transport::NodePtr(new transport::Node());
				this->gazeboNode->Init(this->model->GetWorld()->Name());
				
				//RESET ROS NODE
				this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
				
			//CREATING TOPICS
				//GAZEBO TOPICS
				std::string wheel_cmd_vel_topic = "~/" + this->model->GetName() +"/wheels/cmd_vel"; // SETUP TOPIC NAME

			//SUBSCRIBERS	
				//GAZEBO SUBSCRIBERS	
				this->wheelVelCmdGazeboSub = this->gazeboNode->Subscribe(wheel_cmd_vel_topic, &ShrewFixedPlugin::OnWheelVelocityMsg, this); // Subscribe to topic where wheel velocity commands are sent	
				
				//ROS SUBSCRIBERS
				ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
					"/" + this->model->GetName() + "/wheels/cmd_vel", 1, 
					boost::bind(&ShrewFixedPlugin::OnWheelVelocityMsgFromROS, this, _1), 
					ros::VoidPtr(), ros::getGlobalCallbackQueue());
				this->wheelCmdROSSub = this->rosNode->subscribe(so);
				
				ros::SubscribeOptions so2 = ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
					"/" + this->model->GetName() + "/wheels/slip", 1, 
					boost::bind(&ShrewFixedPlugin::OnWheelSlipMsgFromROS, this, _1), 
					ros::VoidPtr(), ros::getGlobalCallbackQueue());
				this->wheelSlipROSSub = this->rosNode->subscribe(so2);
				
				ros::SubscribeOptions so3 = ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
					"/" + this->model->GetName() + "/wheels/contact_locations", 1, 
					boost::bind(&ShrewFixedPlugin::OnWheelContactMsgFromROS, this, _1), 
					ros::VoidPtr(), ros::getGlobalCallbackQueue());
				this->wheelContactROSSub = this->rosNode->subscribe(so3);
				
			//PUBLISHERS
				//GAZEBO PUBLISHERS
				this->gazeboPhysicsPub = this->gazeboNode->Advertise<gazebo::msgs::Physics>("~/physics"); //PUBLISHER ON PHYSICS TOPIC; USED FOR ADJUSTING TIMING OF THE GAZEBO SIMULATION
				this->wheelVelCmdGazeboPub = gazeboNode->Advertise<gazebo::msgs::Quaternion>(wheel_cmd_vel_topic);
					
				//ROS PUBLISHERS
				this->jointStateROSPublisher = this->rosNode->advertise<sensor_msgs::JointState>("/joint_states", 1000);
						

			//LATCH OUR UPDATE FUNCTION TO GAZEBO TIME
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
						std::bind(&ShrewFixedPlugin::OnUpdate, this)); //This event is broadcast every simulation iteration.

      		//SPIN UP THE QUEUE HELPER THREAD.
				this->rosQueueThread = std::thread(std::bind(&ShrewFixedPlugin::QueueThread, this));
				
			//SET UP TIMING
				this->setGazeboTiming();
				this->waitForClockROS();

      	};
		
		////////////////////////////////////////////////////////////////////////////////////	
        //GAZEBO UPDATE CALLBACK - ACTIONS TO TAKE ON GAZEBO UPDATES; CALLED EVERY ITERATION
		private: void OnUpdate() //joint_state are published here
		{
			//PUBLISH JOINT STATES TO ROS
				this->jointStateROSPublisher.publish(this->createJointStatesMsgsForROS());	
			//FIX INTERNAL JOINT
				this->SetInternalJoint();
		};	
		
		////////////////////////////////////////////////////////////////////////////////////////////
		// ROS HELPER FUNCTION TO PROCESS MESSAGES
		private: void QueueThread()
		{
			//INVOKE ALL CALLBACKS CURRENTLY IN THE QUEUE. IF CALLBACK NOT READY TO BE CALLED, PUSHES
			//IT BACK ONTO QUEUE. TIMEOUT SPECIFIES WAIT TIME FOR CALLBACK TO BE AVAILABLE BEFORE RETURNING.
  				double timeout = this->model->GetWorld()->Physics()->gazebo::physics::PhysicsEngine::GetUpdatePeriod();
 				while (this->rosNode->ok())
  				{
   					ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(timeout));
   					//this->rosQueue.callAvailable(ros::WallDuration(timeout));
  				}
		};
							
		////////////////////////////////////////////////////////////////////////////////////	
        //SUBSCRIBER CALLBACK - ACTIONS TO TAKE WHEN VELOCITY MESSAGE IS SENT DIRECTLY TO WHEELS
		private: void OnWheelVelocityMsgFromROS(const geometry_msgs::Quaternion::ConstPtr& _vel)  //For velocity updating dynamically from received message
		{	
			//pass the message on to the Gazebo subscriber
				gazebo::msgs::Quaternion msg;
				gazebo::msgs::Set(&msg, ignition::math::Quaternion(
					_vel->x, _vel->y, _vel->z, _vel->w));	
				this->wheelVelCmdGazeboPub->Publish(msg);

		}; 
		
		private: void OnWheelContactMsgFromROS(const geometry_msgs::Quaternion::ConstPtr& _contacts)  //For velocity updating dynamically from received message
		{	
			//pass the message on to the Gazebo subscriber
			contact_locations_.resize(4);
			this->contact_locations_[0]=_contacts->x;
			this->contact_locations_[1]=_contacts->y;
			this->contact_locations_[2]=_contacts->z;
			this->contact_locations_[3]=_contacts->w;

		}; 
					
		////////////////////////////////////////////////////////////////////////////////////	
        //SUBSCRIBER CALLBACK - ACTIONS TO TAKE WHEN VELOCITY MESSAGE IS SENT DIRECTLY TO WHEELS
		private: void OnWheelVelocityMsg(ConstQuaternionPtr &_vel)  //For velocity updating dynamically from received message
		{	
			this->SetWheelSpeeds(_vel);
			
		}; 
		
		////////////////////////////////////////////////////////////////////////////////////	
        //SET WHEEL VELOCITIES IN SIM
		public: void SetWheelSpeeds(ConstQuaternionPtr &_vel) //public member function which can be used as a function on the command line or by message passing on a named topic
		{
			//SET VELOCITIES
        		this->model->GetJointController()->SetVelocityTarget(wheels_[FL]->GetScopedName(), _vel->w()); // m/s. or rad/s. Set the target velocity
				this->model->GetJointController()->SetVelocityTarget(wheels_[FR]->GetScopedName(), _vel->x()); // m/s. or rad/s. Set the target velocity
				this->model->GetJointController()->SetVelocityTarget(wheels_[BL]->GetScopedName(), _vel->y()); // m/s. or rad/s. Set the target velocity
				this->model->GetJointController()->SetVelocityTarget(wheels_[BR]->GetScopedName(), _vel->z()); // m/s. or rad/s. Set the target velocity
			
			//STORING VELOCITY DIRECTIONS TO USE WHEN APPLYING NET FORCE
				this->vel_cmds_.resize(4);
				this->vel_cmds_[FL]=_vel->w();
				this->vel_cmds_[FR]=_vel->x();
				this->vel_cmds_[BL]=_vel->y();
				this->vel_cmds_[BR]=_vel->z();
			//DEBUG STUFF
				//std::cout << "New velocity command sent to wheels." << std::endl;
				//std::cout << "FL vel: " << _vel->w() << " rad/s" << std::endl;	
				//std::cout << "FR vel: " << _vel->x() << " rad/s" << std::endl;
				//std::cout << "BL vel: " << _vel->y() << " rad/s" << std::endl;
				//std::cout << "BR vel: " << _vel->z() << " rad/s" << std::endl;
				//std::cout << _vel->DebugString() << std::endl; 
		};
		
		public: void OnWheelSlipMsgFromROS(const geometry_msgs::Quaternion::ConstPtr& _slip)
		{
			//std::cout << _slip->x << ", " << _slip->y << ", " << _slip->z << ", " << _slip->w << std::endl;
			std::vector<double> slips;
			slips.resize(4);
			slips[0]=_slip->x;
			slips[1]=_slip->y;
			slips[2]=_slip->z;
			slips[3]=_slip->w;
			this->wheel_angles_.resize(4);
			this->net_forces_.resize(4);
			this->res_torques_.resize(4);
			wheel_angles_[FL]=this->model->GetJoint("joint_wheel_FL")->Position(0);
			wheel_angles_[FR]=this->model->GetJoint("joint_wheel_FR")->Position(0);
			wheel_angles_[BL]=this->model->GetJoint("joint_wheel_BL")->Position(0);
			wheel_angles_[BR]=this->model->GetJoint("joint_wheel_BR")->Position(0);
			
			//wheel_angles_[FL]=std::fmod(abs(this->model->GetJoint("joint_wheel_FL")->Position(0)), (2*M_PI));
			//wheel_angles_[FR]=std::fmod(abs(this->model->GetJoint("joint_wheel_FR")->Position(0)), (2*M_PI));
			//wheel_angles_[BL]=std::fmod(abs(this->model->GetJoint("joint_wheel_BL")->Position(0)), (2*M_PI));
			//wheel_angles_[BR]=std::fmod(abs(this->model->GetJoint("joint_wheel_BR")->Position(0)), (2*M_PI));
			//std::cout << wheel_angles_[0] << ", " << wheel_angles_[1] << ", " << wheel_angles_[2] << ", " << wheel_angles_[3] << std::endl;
			
			for (int i=0; i<5; i++) {
				if (this->contact_locations_[i]==1) {
					net_forces_[i]=1.8*(40-exp(-4*(slips[i]-0.95))); //high friction model
				}
				
				if (this->contact_locations_[i]==0) {
					net_forces_[i]=1.8*(40-exp(-1*(slips[i]-3.8)));//low friction model
				}
				
				if (net_forces_[i]<0) {
					net_forces_[i]=0;
				}
				if (vel_cmds_[i]<0) {
					net_forces_[i]=-net_forces_[i];
				}
				//if (std::fmod(i, 2)==0) {
				//	net_forces_[i]=-net_forces_[i];
				//}
				res_torques_[i]=-net_forces_[i]*0.1396;
			}
			
			//std::cout << wheel_angles_[FL] << std::endl;
			//std::cout << wheel_angles_[FR] << std::endl;
			//std::cout << wheel_angles_[BL]<< std::endl;
			//std::cout << wheel_angles_[BR] << std::endl;
			
			//std::cout << cos(wheel_angles_[FL])*net_forces_[0] << " " << sin(wheel_angles_[FL])*net_forces_[0] << std::endl;
			//std::cout << cos(wheel_angles_[FR])*net_forces_[1] << " " << sin(wheel_angles_[FR])*net_forces_[1] << std::endl;
			//std::cout << cos(wheel_angles_[BL])*net_forces_[2] << " " << sin(wheel_angles_[BL])*net_forces_[2] << std::endl;
			//std::cout << cos(wheel_angles_[BR])*net_forces_[3] << " " << sin(wheel_angles_[BR])*net_forces_[3] << std::endl;
			
			
			//std::cout << "Net Force FL: " << net_forces_[0] << std::endl;
			//std::cout << "Net Force FR: " << net_forces_[1] << std::endl;
			//std::cout << "Net Force BL: " << net_forces_[2] << std::endl;
			//std::cout << "Net Force BR: " << net_forces_[3] << std::endl;
			
			//std::cout << "Resistive Torque FL: " << res_torques_[0] << std::endl;
			//std::cout << "Resistive Torque FR: " << res_torques_[1] << std::endl;
			//std::cout << "Resistive Torque BL: " << res_torques_[2] << std::endl;
			//std::cout << "Resistive Torque BR: " << res_torques_[3] << std::endl;
			
			//SET NET FORCE ON EACH WHEEL	
			//this->model->GetLink("wheel_FL")->SetForce(ignition::math::Vector3d(cos(wheel_angles_[FL])*net_forces_[0],0,sin(wheel_angles_[FL])*net_forces_[0]));
			//this->model->GetLink("wheel_FR")->SetForce(ignition::math::Vector3d(cos(wheel_angles_[FR])*net_forces_[1],0,sin(wheel_angles_[FR])*net_forces_[1]));
			//this->model->GetLink("wheel_BL")->SetForce(ignition::math::Vector3d(cos(wheel_angles_[BL])*net_forces_[2],0,sin(wheel_angles_[BL])*net_forces_[2]));
			//this->model->GetLink("wheel_BR")->SetForce(ignition::math::Vector3d(cos(wheel_angles_[BR])*net_forces_[3],0,sin(wheel_angles_[BR])*net_forces_[3]));
			
			//SET RESISTIVE TORQUE ON EACH WHEEL
			//this->model->GetLink("wheel_FL")->SetTorque(ignition::math::Vector3d(0, res_torques_[0], 0));
			//this->model->GetLink("wheel_FR")->SetTorque(ignition::math::Vector3d(0, res_torques_[1], 0));
			//this->model->GetLink("wheel_BL")->SetTorque(ignition::math::Vector3d(0, res_torques_[2], 0));
			//this->model->GetLink("wheel_BR")->SetTorque(ignition::math::Vector3d(0, res_torques_[3], 0));
		};
		
		////////////////////////////////////////////////////////////////////////////////////	
        //SET INTERNAL JOINT POSITION
		private: void SetInternalJoint()
		{
			//SET VELOCITIES
        		this->model->GetJointController()->SetPositionTarget(this->model->GetJoint("internal_joint")->GetScopedName(), 0);

		};		
		
		////////////////////////////////////////////////////////////////////////////////////	
        //SET GAZEBO TIMING
		private: void setGazeboTiming()
		{
			//CREATE PHYSICS MESSAGE
				msgs::Physics physicsMsg;
      			physicsMsg.set_type(msgs::Physics::ODE);
		
			//CHECK IF THE PARAMETERS ARE NAMED IN THE URDF
				if (this->sdf->HasElement("updateRate")) { //UPDATES ATTEMPTED PER SECOND; HZ
					physicsMsg.set_real_time_update_rate(this->sdf->Get<double>("updateRate"));
				} 
				
				if (this->sdf->HasElement("stepTime")) { //MAX TIME DURATION BETWEEN PHYSICS UPDATES
					physicsMsg.set_max_step_size(this->sdf->Get<double>("stepTime"));
				}
				
				if (this->sdf->HasElement("iterations")) {//SPECIFIES ITERATIONS USED BY LCP SOLVERS
					physicsMsg.set_iters(this->sdf->Get<double>("iterations"));
				}
				
			//PUBLISH PHYSICS MESSAGE
				this->gazeboPhysicsPub->Publish(physicsMsg); 
		};
		
		////////////////////////////////////////////////////////////////////////////////////
		//WAIT FOR ROS NODE TO RECEIVE /clock MESSAGE FROM GAZEBO. ROS USES GAZEBO SIM TIME AS SET BY use_sim_time IN LAUNCH FILES
		private: void waitForClockROS()
		{
			ros::Time last_ros_time_; 
			bool wait = true;
			while (wait) { 
				last_ros_time_ = ros::Time::now();
				if (last_ros_time_.toSec() > 0) {
					wait = false;
				}
			}
		};
		
		////////////////////////////////////////////////////////////////////////////////////	
        //CREATE THE OBJECT USED FOR PID
		private: void makePID()
		{
			//CHECK IF GAINS ARE SPECIFIED IN MODEL
				if (this->sdf->HasElement("wheel_kp") && this->sdf->HasElement("wheel_ki") && this->sdf->HasElement("wheel_kd")) {
					//INITIALIZE PID OBJECT	
					this->gains_.resize(3);
					this->gains_[Kp] = this->sdf->Get<double>("wheel_kp"); //http: //osrf-distributions.s3.amazonaws.com/gazebo/api/1.9.1/classgazebo_1_1common_1_1PID.html#a83ec1335e4e25d1c07ab6f9cf6075dab
	 				this->gains_[Ki] = this->sdf->Get<double>("wheel_ki");
					this->gains_[Kd] = this->sdf->Get<double>("wheel_kd");
					this->pid = common::PID(this->gains_[Kp], this->gains_[Ki], this->gains_[Kd]); //setup a controller
					std::cerr << "Wheel velocity PID:" << std::endl;
					std::cerr << "Kp: " << this->gains_[Kp] << ", Ki: " << this->gains_[Ki] << ", Kd: " << this->gains_[Kd] << std::endl;
			
					this->ij_pid = common::PID(10, 0, 0);
					std::cerr << "Internal joint position PID:" << std::endl;
					std::cerr << "Kp: 10,  Ki: 0, Kd: 0"<< std::endl;
			
					//ADD CONTROLLABLE JOINTS TO MODEL	
					this->model->GetJointController()->AddJoint(this->wheels_[FL]); //add the joint to the model controller
					this->model->GetJointController()->SetVelocityPID(this->wheels_[FL]->GetScopedName(), this->pid); //apply the controller to the joint
						
					this->model->GetJointController()->AddJoint(this->wheels_[FR]); //add the joint to the model controller
					this->model->GetJointController()->SetVelocityPID(this->wheels_[FR]->GetScopedName(), this->pid); //apply the controller to the joint
						
					this->model->GetJointController()->AddJoint(this->wheels_[BL]); //add the joint to the model controller
					this->model->GetJointController()->SetVelocityPID(this->wheels_[BL]->GetScopedName(), this->pid); //apply the controller to the joint
						
					this->model->GetJointController()->AddJoint(this->wheels_[BR]); //add the joint to the model controller
					this->model->GetJointController()->SetVelocityPID(this->wheels_[BR]->GetScopedName(), this->pid); //apply the controller to the joint
				
					this->model->GetJointController()->AddJoint(this->model->GetJoint("internal_joint")); //add the joint to the model controller
					this->model->GetJointController()->SetPositionPID(this->model->GetJoint("internal_joint")->GetScopedName(), this->ij_pid); //apply the controller to the joint
				
				}
		};
		
		////////////////////////////////////////////////////////////////////////////////////	
        //FILL PTR TO WHEEL JOINTS
		private: void fillWheelPtr()
		{
			//STORE JOINTS
				if (this->model->GetJointCount() == 0) { 
					std::cerr << "ERROR: No joints found in our model. This plugin will not work." << std::endl;
					return;
				}
				this->wheels_[FL] = this->model->GetJoint("joint_wheel_FL");
				this->wheels_[FR] = this->model->GetJoint("joint_wheel_FR");
				this->wheels_[BL] = this->model->GetJoint("joint_wheel_BL");
				this->wheels_[BR] = this->model->GetJoint("joint_wheel_BR");
		};		
		
		////////////////////////////////////////////////////////////////////////////////////
        //FIX DYNAMIC PROPERTIES NOT SET WHEN PARSED BY GAZEBO
        //In the future it would likely be a good idea to implement the filling of these parameters using a YAML file
		private: void setDynamProps()
		{
	
			//STORE WHEEL DYNAMIC ELEMENTS - FOR GAZEBO API
				if (this->sdf->HasElement("wheel_damping") && this->sdf->HasElement("wheel_friction")
					&& this->sdf->HasElement("wheel_effort_lim") && this->sdf->HasElement("wheel_vel_lim")) {
						double wheel_damp=this->sdf->Get<double>("wheel_damping");
						double wheel_fric=this->sdf->Get<double>("wheel_friction");
						double wheel_effLim=this->sdf->Get<double>("wheel_effort_lim");
						double wheel_velLim=this->sdf->Get<double>("wheel_vel_lim");
						this->wheels_[FL]->SetDamping(0, wheel_damp); //viscous friction ✓ 
						this->wheels_[FL]->SetParam("friction", 0, wheel_fric); //static friction ✓
						this->wheels_[FL]->SetEffortLimit(0, wheel_effLim); //torque limit ✓ 22.5
						this->wheels_[FL]->SetVelocityLimit(0, wheel_velLim); //velocity limit ✓
						this->wheels_[FR]->SetDamping(0, wheel_damp); //viscous friction ✓ 
						this->wheels_[FR]->SetParam("friction", 0, wheel_fric); //static friction ✓
						this->wheels_[FR]->SetEffortLimit(0, wheel_effLim); //torque limit ✓ 22.5
						this->wheels_[FR]->SetVelocityLimit(0, wheel_velLim); //velocity limit ✓
						this->wheels_[BL]->SetDamping(0, wheel_damp); //viscous friction ✓ 
						this->wheels_[BL]->SetParam("friction", 0, wheel_fric); //static friction ✓
						this->wheels_[BL]->SetEffortLimit(0, wheel_effLim); //torque limit ✓ 22.5
						this->wheels_[BL]->SetVelocityLimit(0, wheel_velLim); //velocity limit ✓
						this->wheels_[BR]->SetDamping(0, wheel_damp); //viscous friction ✓ 
						this->wheels_[BR]->SetParam("friction", 0, wheel_fric); //static friction ✓
						this->wheels_[BR]->SetEffortLimit(0, wheel_effLim); //torque limit ✓ 22.5
						this->wheels_[BR]->SetVelocityLimit(0, wheel_velLim); //velocity limit ✓
						this->wheels_[FL]->SetProvideFeedback(1); //feedback the forces and torques ✓
						this->wheels_[FR]->SetProvideFeedback(1); //feedback the forces and torques ✓
						this->wheels_[BL]->SetProvideFeedback(1); //feedback the forces and torques ✓
						this->wheels_[BR]->SetProvideFeedback(1); //feedback the forces and torques ✓
					
						//DEBUG STUFF
						//std::cout << "Motor Damping is: " << this->wheels_[FL]->GetDamping(0) << std::endl; //✓
						//std::cout << "Motor Friction is: " << this->wheels_[FL]->GetParam("friction", 0) << std::endl; //✓
						//std::cout << "Motor Effort Limit is: " << this->wheels_[FL]->GetEffortLimit(0) << std::endl; //✓
						//std::cout << "Wheel Velocity Limit is: " << this->wheels_[FL]->GetVelocityLimit(0) << std::endl; //✓
				} else {
					GZ_ASSERT(this->sdf, "Dynamic properties of model wheels not set properly in URDF. Include these in the <gazebo><plugin> tags at the base of the robot model.\n");
				}
				
			//STORE INTERNAL JOINT DYNAMIC PROPERTIES	
				if (this->sdf->HasElement("internal_joint_damping") && this->sdf->HasElement("internal_joint_friction")
					&& this->sdf->HasElement("internal_joint_effort_lim") && this->sdf->HasElement("internal_joint_vel_lim")) {
						double ij_damp=this->sdf->Get<double>("internal_joint_damping");
						double ij_fric=this->sdf->Get<double>("internal_joint_friction");
						double ij_effLim=this->sdf->Get<double>("internal_joint_effort_lim");
						double ij_velLim=this->sdf->Get<double>("internal_joint_vel_lim");
						this->model->GetJoint("internal_joint")->SetDamping(0, ij_damp); //viscous friction ✓ 
						this->model->GetJoint("internal_joint")->SetParam("friction", 0, ij_fric); //static friction ✓
						this->model->GetJoint("internal_joint")->SetEffortLimit(0, ij_effLim); //torque limit ✓ 22.5
						this->model->GetJoint("internal_joint")->SetVelocityLimit(0, ij_velLim); //velocity limit ✓
						this->model->GetJoint("internal_joint")->SetProvideFeedback(1); //feedback the forces and torques ✓
						
						//DEBUG STUFF
						//std::cout << "Internal Joint Damping: " << this->model->GetJoint("internal_joint")->GetDamping(0) << std::endl; //✓
						//std::cout << "Internal Joint Friction: " << this->model->GetJoint("internal_joint")->GetParam("friction", 0) << std::endl; //✓
						//std::cout << "Internal Joint Effort Limit: " << this->model->GetJoint("internal_joint")->GetEffortLimit(0) << std::endl; //✓
						//std::cout << "Internal Joint Velocity Limit: " << this->model->GetJoint("internal_joint")->GetVelocityLimit(0) << std::endl; //✓
				
				}
				
		};
		
		////////////////////////////////////////////////////////////////////////////////////
        //FILL A MESSAGE WITH JOINT STATE INFORMATION TO BE SENT TO ROS
       	//In the future it would probably be a good idea to make this its own executable
		private: sensor_msgs::JointState createJointStatesMsgsForROS()
		{	
			//std::cout << "Position:" << this->wheels_[FL]->Position() << std::endl;
			//std::cout << "Velocity:" << this->wheels_[FL]->GetVelocity(0) << std::endl;
			//std::cout << "Force:" << this->wheels_[FL]->GetForce(0) << std::endl;
			//std::cout << "ForceTorque body1Force" << this->wheels_[FL]->GetForceTorque(0).body1Force << std::endl;
			//std::cout << "ForceTorque body2Force" << this->wheels_[FL]->GetForceTorque(0).body2Force << std::endl;
			//std::cout << "ForceTorque body1Torque" << this->wheels_[FL]->GetForceTorque(0).body1Torque << std::endl;
			//std::cout << "ForceTorque body2Torque" << this->wheels_[FL]->GetForceTorque(0).body2Torque << std::endl;
			//std::cout << "LinkForce:" << this->wheels_[FL]->LinkForce(0) << std::endl;
			//std::cout << "LinkTorque:" << this->wheels_[FL]->LinkTorque(0) << std::endl;
			//std::cout << "WorldPose:" << this->wheels_[FL]->WorldPose() << std::endl;
		
		
			sensor_msgs::JointState jointState; //create an object of type "jointState"
			
			//jointState.header.seq is filled automatically with the iteration of the published message.
			jointState.header.stamp = ros::Time::now();
			jointState.header.frame_id = "shrew_robot_fixed_joint";
			
			jointState.name.push_back("internal_joint");
			jointState.name.push_back("joint_wheel_FL");
			jointState.name.push_back("joint_wheel_FR");
			jointState.name.push_back("joint_wheel_BL");    
			jointState.name.push_back("joint_wheel_BR");    
			
			jointState.position.push_back(this->model->GetJoint("internal_joint")->Position());
			jointState.position.push_back(this->wheels_[FL]->Position());
			jointState.position.push_back(this->wheels_[FR]->Position());
			jointState.position.push_back(this->wheels_[BL]->Position());
			jointState.position.push_back(this->wheels_[BR]->Position());
			
			jointState.velocity.push_back(this->model->GetJoint("internal_joint")->GetVelocity(0));
			jointState.velocity.push_back(this->wheels_[FL]->GetVelocity(0));
			jointState.velocity.push_back(this->wheels_[FR]->GetVelocity(0));
			jointState.velocity.push_back(this->wheels_[BL]->GetVelocity(0));
			jointState.velocity.push_back(this->wheels_[BR]->GetVelocity(0));
			
			jointState.effort.push_back(this->model->GetJoint("internal_joint")->GetForce(0));
			jointState.effort.push_back(this->wheels_[FL]->GetForce(0));
			jointState.effort.push_back(this->wheels_[FR]->GetForce(0));
			jointState.effort.push_back(this->wheels_[BL]->GetForce(0));
			jointState.effort.push_back(this->wheels_[BR]->GetForce(0));
			
			return jointState;
		};
		
		////////////////////////////////////////////////////////////////////////////////////
      	//MEMBER POINTERS
      	private:	physics::ModelPtr model; // POINTER TO THE MODEL
		private:	sdf::ElementPtr sdf; // POINTER TO THE SDF
		private:	std::vector<boost::shared_ptr<physics::Joint>> wheels_; // VECTOR OF POINTERS TO THE WHEEL JOINTS
		private:	std::vector<double> gains_; // VECTOR OF OUR GAINS
		private:	std::vector<double> vel_cmds_;
		private:	std::vector<double> wheel_angles_;
		private:	std::vector<double> net_forces_;
		private:	std::vector<double> res_torques_;
		private:	std::vector<double> contact_locations_;
		
		private:	common::PID	pid; //PID FOR WHEELS
		private:	common::PID	ij_pid; //PID FOR INTERNAL JOINT
      	
      	private:	event::ConnectionPtr updateConnection; // POINTER TO THE UPDATE EVENT CONNECTION

		private: 	transport::NodePtr gazeboNode; //A NODE USED FOR GAZEBO TRANSPORT
		private: 	transport::SubscriberPtr wheelVelCmdGazeboSub; //A SUBSCRIBER TO THE NAMED VELOCITY COMMAND TOPIC.
		private:	transport::PublisherPtr gazeboPhysicsPub; //A PTR TO A PUBLISHER ON THE GAZEBO PHYSICS TOPIC
		private:	transport::PublisherPtr wheelVelCmdGazeboPub;

		private: 	std::unique_ptr<ros::NodeHandle> rosNode; //NODE USED FOR ROS TRANSPORT
		private: 	ros::Subscriber wheelCmdROSSub; //A ROS SUBSCRIBER
		private:	ros::Subscriber wheelSlipROSSub; //A ROS SUBSCRIBER TO THE WHEEL SLIP TOPIC
		private:	ros::Subscriber wheelContactROSSub;
		private:	ros::Publisher jointStateROSPublisher; //A ROS PUBLISHER
		private: 	ros::CallbackQueue rosQueue; //A ROS CALLBACKQUEUE THAT HELPS PROCESS MESSAGES
		private: 	std::thread rosQueueThread; //A THREAD THAT KEEPS RUNNING THE ROSQUEUE

		private:	std::string launchString = R"(____________________________________)" "\n" "\n"
												R"(              ______--------___)" "\n"
												R"(             /|             / |)" "\n"
												R"(  o___________|_\__________/__|)" "\n"
												R"( ]|___     |  |=   ||  =|___  |")" "\n"
												R"( //   \\    |  |____||_///   \\|")" "\n"
												R"(|  X  |\--------------/|  X  |\")" "\n"
												R"( \___/   SHREW ROBOT    \___/)" "\n"
												R"( )" "\n"
												R"(        TIME TO DRIVE WITH)" "\n"
												R"(     THE SHREW ROBOT PLUGIN!)" "\n" "\n"
												R"(____________________________________)" "\n";
			
	};	
	// REGISTER THIS PLUGIN WITH THE SIMULATOR
	GZ_REGISTER_MODEL_PLUGIN(ShrewFixedPlugin);
};
