# PhD-Thesis-Repository
By: Adam Gronewold; Adam.M.Gronewold.TH@Dartmouth.edu; Winter 2025
----------------------------------------------------------------------------
This repository contains code used to test UGV sensing and mobility technologies in an agricultural environment, in contribution to a PhD thesis at Dartmouth College.

# CODE FOR TACTILE SENSING AND VISUALLY IMPAIRED NAVIGATION
  Matlab code to test and validate tactile sensor technologies are included. “One_Feeler_Sensor_Sim.m” is the Matlab file used to test the dynamics of a tactile sensor in a stationary frame. It can act as a test bed to improve contact detection by iterating on the finite state machine described in Chapter 5.3, and to validate feeler sensors with different parameters. It could also be used to engineer contact classifiers that are vehicle-speed agnostic, making the tactile sensing system applicable to a wider range of real-world applications. 
  “FSM_Classifier_for_Real_Dynamics.m” is the finite state machine as tuned for a real-world vehicle travelling at 0.2 m/s. 
  “Blind_Nav_With_Feeler_Sensors.m” is the comprehensive simulation, described in Chapter 6.4 of the thesis, of a 2D vehicle, affixed with two tactile sensors and autonomously navigating a cornfield. Parameterization of both the physical vehicle and the environment, and of the vehicle’s estimated understanding of the environment as impacted by measurement noise, errors, extrapolation of position, etc., are available to adjust freely within the simulation. This simulation environment can be used to study other navigation approaches for a cornfield, test performance at different crop development stages by adding simulated leaves, and test sensor performance in different field structures with varied gap widths and planting patterns. 
  Tactile sensing and blind navigation within broadacre crops is explained briefly in a presentation included on the Internet Archive: 

# CODE FOR SHREW ROBOT SIMULATION
  An additional package is included of a ROS simulation environment for SHREW robots. The directory named "shrew_workspace" is a catkin style ROS Noetic workspace to be paired with Gazebo-11. The workspace includes a URDF description of the robot, a terrain surface with varying "soil" parameters, and all the necessary launch files to get started with testing and further refinement of a SHREW robot. More information on this aspect of the project can be found in the README of the main "shrew_robot" directory. Also included is a rigid body version of the vehicle. 
A brief live-demonstration of the simulation environment and how to get started using it is available through the Internet Archive: https://archive.org/details/shrewgazebo-demo

