#TERRAIN_PATCHES
#by Adam M. Gronewold

**PROJECT DESCRIPTION**
The terrain patches project is for the construction of a flat terrain with variable surface parameters. Blocks belonging to the model are defined in the urdf.xacro file. The intention of the project is to have blocks of like color and material to have like surface properties. As it is constructed in the urdf format, that means by design Gazebo is loading the model as a robot. An alternative approach to construct a terrain using the DEM approach, however this will result in a terrain with only variable elevation and not variable friction, collision, bounce, etc. elements.

**MOTIVATION OF PROJECT**
The motivation for this project is to construct a testbed for terrain sensing type robots to be simulated in Gazebo and to assess the ability to extract meaningful information about the terrain from the robot's interaction with blocks a varying terrain properties.

**OVERVIEW OF FUNCTIONALITY**
The functionality is minimal. It is solely used to construct a base for a robot to drive around on. The model is not intended to do more. Surface properties of blocks in the model may be changed by modifying the terrain_patches.urdf.xacro file.

**OPERATING INSTRUCTIONS**
To launch the terrain in Rviz use the command "roslaunch terrain_patches terrain_patches_rviz.launch". Similarly, to launch the model in Gazebo use "roslaunch terrain_patches terrain_patches_gazebo.launch". Other arguments can be added to the end of this command such as extra_gazebo_args:="--verbose".

**CHECK FOR PROPER LOADING**
The best method to check if the model is loading properly is to launch the model in Gazebo to check to make sure the TF tree is loading properly.

**REQUIREMENTS TO CHANGES**
It is best to change the model of the terrain using the terrain_patches.urdf.xacro file and then copy that document to make a terrain_patches.urdf file in the terrain_patches/urdf folder, because the urdf.xacro file is where the model is constructed. Gazebo will automatically convert the urdf file to an .sdf file where needed. If an .sdf file is needed one can take the same approach to copy the urdf.xacro file into a model.sdf file in a terrain_patches/sdf folder.

**CONTACT FOR QUESTIONS**
Adam M. Gronewold
Thayer School of Engineering at Dartmouth College
PhD student - Ray Lab

Adam.M.Gronewold.TH@Dartmouth.edu



