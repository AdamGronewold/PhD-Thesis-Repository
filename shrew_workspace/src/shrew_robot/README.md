#SHREW_ROBOT
#by Adam M. Gronewold

**PROJECT DESCRIPTION**
This project is for a flexible robot, a Shrew, being developed for applications in space, to compare
rigid body and flexible body rover dynamics, and for collaborative robotic efforts.

**MOTIVATION OF PROJECT**
This project began as under a small NASA BIG Idea Challenge grant to produce meaningful technologies 
for characterization of the lunar south pole. Currently, the focus of the project
is to assess the ability of a flexible robot to sense changes in terrain strictly through proprioceptive
sensing of the state of vehicle joints.


**OPERATING INSTRUCTIONS**
Commands:
	"roslaunch shrew_robot shrew_robot_rviz.launch" launches the robot in Rviz.
	"roslaunch shrew_robot shrew_robot_gazebo.launch" launches the robot in Gazebo.
	"roslaunch shrew_control shrew_control.launch" launches the controllers to make joints active.
	"roslaunch terrain_patches terrain_patches_rviz.launch" launches the terrain in Rviz.
	"roslaunch terrain_patches terrain_patches_gazebo.launch" launches the terrain in Gazebo.
	"roslaunch shrew_robot shrew_robot_wterrain_gazebo.launch" launches the robot in the terrain.
	

**CHECK FOR PROPER LOADING**
Those concerned about the model loading properly are suggested to try the following:

1) 	Load the model in Rviz to check the tf. This can be done with the command:
		"roslaunch shrew_robot shrew_robot_rviz.launch". 
	The robot should load in the center of
	the Rviz ground plane. If all parts of the robot load in the same location it is likely
	because Rviz is using something other than base_link as the fixed frame. The joint state
	GUI will also be loaded by default. This may be used to assess the freedom of joints in the
	robot model (although real joint states are not being loaded to the model, if for instance
	it is open in Gazebo as well.)
	
2)	Load the robot into Gazebo with one of the Gazebo launch files:
		"roslaunch shrew_robot shrew_robot_gazebo.launch" (without terrain)
		"roslaunch shrew_robot shrew_robot_wterrain_gazebo.launch (in the terrain_patches terrain)
	It should load without errors in both cases. If one then loads the controllers for the robot
	using:
		"roslaunch shrew_control shrew_control.launch"
	the robot_state_publisher and the shrew_robot/controller_spawner nodes should be loaded. Check
	with "rosnode list". Many topics should also now be open relating to the joints of the vehicle.
	Check with "rostopic list".
	
3)	Publish test commands to vehicle joints. This can be performed simply using:
		"rostopic pub -1 /shrew_robot/(name of the controller for the joint)/command std_msgs/Float64 "data: (value)"
	Only joints with a <!transmission> tag in the urdf can be manipulated by publishing commands.
	
4) 	Ensure the velocity controllers are downloaded for use by the wheels. Run:
		"sudo apt install ros-noetic-velocity-controllers"
	or list the available types with:
		"rosservice call controller_manager/list_controller_types"


**REQUIREMENTS TO CHANGES**
Changes to the robot model should be performed in the shrew_robot.urdf.xacro file, as all other
versions of the robot are generated either automatically by gazebo (sdf type files) or manually by 
copying the xacro file into a urdf file.

**FILE DESCRIPTIONS**
shrew_robot >
	launch >
		shrew_robot.rviz - rviz world file
		shrew_robot_gazebo.launch - launch in Gazebo
		shrew_robot_rviz.launch - launch in Rviz
		shrew_robot_wterrain_gazebo.launch - launch robot in terrain
	meshes >
		shrew_axle.STL
		shrew_pivot.STL
		shrew_slot.STL
		shrew_wheel.STL
		shrew_winch.STL
	urdf >
		shrew_robot.urdf.xacro - Primary robot model
		shrew_robot.urdf - urdf model generated from xacro
	CMakeLists.txt
	package.xml
	README.md - full package readme
	
shrew_control >
	config >
		shrew_control.yaml
	launch >
		shrew_control.launch - launch robot controllers
	CMakeLists.txt
	package.xml
	
terrain_patches >
	launch >
		terrain_patches.rviz - Rviz world file for terrain
		terrain_patches_gazebo.launch - launch terrain in Gazebo
		terrain_patches_rviz.launch - launch terrain in Rviz
	urdf >
		materials.xacro - rviz material file
		terrain_patches.urdf.xacro - primary terrain model
		terrain_patches_plugin.gazebo - gazebo colors file
	worlds >
		terrain_patches.world - world containing terrain model
		terrain_patches2.world
	CMakeLists.txt
	model.config
	package.xml
	README.md - terrain readme
	
**CONTACT FOR QUESTIONS**
Adam M. Gronewold
Thayer School of Engineering at Dartmouth College
PhD student - Ray Lab

Adam.M.Gronewold.TH@Dartmouth.edu



