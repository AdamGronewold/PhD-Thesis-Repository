#!/usr/bin/env python

#------------------------------------------------------------------------------------------
#by Adam Gronewold
#version1
#this file is used to move the shrew robot in a gazebo simulation
#------------------------------------------------------------------------------------------

import time #used for timing prior to openning ROS Node
import sys #used to shutdown
import numpy as np #import for math operations
import decimal
import rospy #main module for ROS APIs
import roslaunch

from geometry_msgs.msg import Point #message type for reference angles
from geometry_msgs.msg import Quaternion #message type for cmd
from sensor_msgs.msg import JointState #message type for base_scan
from geometry_msgs.msg import PoseStamped #message type for 2D Nav Goal
from nav_msgs.msg import Odometry #message type for odom
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import LinkStates

import tf #library for transformations
import tf2_ros

import os #used to create subdirectory to log information
import re #used to extract num from string
import xlwt #used for creation of workbook
from xlwt import Workbook
from xlwt import easyxf

#constants
FREQUENCY=10 #Hz
DESIRED_LINEAR_VELOCITY=0.25 #m/s
YAW_KP=6
ANGULAR_RATE=np.pi/4 #rad/s
WHEEL_RADIUS=0.1396 #m
YAW_LIM=0.4 #radians +/-

class ShrewMove:

#-------------------------------------------initialize---------------------------------------		
	def __init__(self):
		
		#Create log directory and sheet to log data to
		self.logger_init()
				
		#Set up spreadsheet to write data to
		self.spreadsheet_init()
		
		#initialize node
		rospy.init_node("shrew_fixed_joint_mover") 
		
		
		
		#setting up publishers/subscribers
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

		self.last_mode_change_time=rospy.Time.now()
		self.last_joint_msg = JointState
		self.last_link_msg = LinkStates
		self.last_contacts_FL_msg = ContactsState
		self.last_contacts_FR_msg = ContactsState
		self.last_contacts_BL_msg = ContactsState
		self.last_contacts_BR_msg = ContactsState
		
		self.yaw_ref_pub=rospy.Publisher("/shrew_robot_fixed_joint/yaw_joints/ref_angles", Point, queue_size=1)
		self.wheel_vel_pub=rospy.Publisher("/shrew_robot_fixed_joint/wheels/cmd_vel", Quaternion, queue_size=1)
		self.slip_pub=rospy.Publisher("/shrew_robot_fixed_joint/wheels/slip_vel", Quaternion, queue_size=1)
		self.contact_locations_pub=rospy.Publisher("/shrew_robot_fixed_joint/wheels/contact_locations", Quaternion, queue_size=1)
		self.contact_forces_FL_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_forces/FL", Quaternion, queue_size=1)
		self.contact_forces_FR_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_forces/FR", Quaternion, queue_size=1)
		self.contact_forces_BL_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_forces/BL", Quaternion, queue_size=1)
		self.contact_forces_BR_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_forces/BR", Quaternion, queue_size=1)
		self.contact_torques_FL_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_torques/FL", Quaternion, queue_size=1)
		self.contact_torques_FR_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_torques/FR", Quaternion, queue_size=1)
		self.contact_torques_BL_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_torques/BL", Quaternion, queue_size=1)
		self.contact_torques_BR_pub=rospy.Publisher("/shrew_robot_fixed_joint/contact_torques/BR", Quaternion, queue_size=1)		
		
		self.joint_sub=rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
		self.link_state_sub=rospy.Subscriber("gazebo/link_states", LinkStates, self.link_state_callback)
		self.nav_goal_sub=rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.nav_goal_callback)
		
		self.contacts_FL_sub=rospy.Subscriber("/contacts_state_wheel_FL", ContactsState, self.contacts_state_FL_callback)
		self.contacts_FR_sub=rospy.Subscriber("/contacts_state_wheel_FR", ContactsState, self.contacts_state_FR_callback)
		self.contacts_BL_sub=rospy.Subscriber("/contacts_state_wheel_BL", ContactsState, self.contacts_state_BL_callback)
		self.contacts_BR_sub=rospy.Subscriber("/contacts_state_wheel_BR", ContactsState, self.contacts_state_BR_callback)
		
		#register shutdown callback function
		rospy.on_shutdown(self.rospy_close_callback)
			
		self.rqt_launch()
		
		#allowing time for registration of ros master if it is not open
		rospy.sleep(1)
	
		print('Publish commands as PoseStamped on /move_base_simple/goal using RViz. A default speed is used currently.')

	def rqt_launch(self):
		parent_directory = "/home/ray-lab/catkin_ws/src/full_sim_fixed_joint/launch/"
		full_launch_path = os.path.join(parent_directory, "shrew_fixed_joint_sim_rqt.launch")
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		self.launch = roslaunch.parent.ROSLaunchParent(uuid, [full_launch_path])
		self.launch.start()
		rospy.sleep(1)
		print('')
		#rospy.loginfo("started")
		
	def logger_init(self):
		parent_directory = "/home/ray-lab/catkin_ws/src/full_sim_fixed_joint/logs/" #name the parent directory under which log directory is created
		subfolders = [ f.name for f in os.scandir(parent_directory) if f.is_dir()] #list subdirectories under parent
		directory_nums=[]
		for x in subfolders:
			directory_nums.append([int(num) for num in re.findall(r"\d+", x)]) #extract the int signifier at the end of all subdirectories
		
		new_log_directory = "full_sim_fixed_joint_log_"+str(np.max(directory_nums)+1) #name for new logger subdirectory
		self.new_log_path = os.path.join(parent_directory, new_log_directory)
		os.mkdir(self.new_log_path) #make log directory	
		print("New log directory created for this simulation at: '%s'" % self.new_log_path)
		time.sleep(1)
		print('')

	def spreadsheet_init(self):
	
		#create workbook
		self.wb = Workbook()
		#add sheet to workbook
		self.sheet=self.wb.add_sheet("Data Sheet 1")
		first_col = self.sheet.col(0)
		first_col.width = 512 * 20 
		
		first_row = self.sheet.row(0)
		first_row.height = 32*20
		
		second_row = self.sheet.row(1)
		second_row.height = 16 * 20
		third_row = self.sheet.row(2)
		third_row.height= 64 * 20
		
		wrap_align = xlwt.Alignment()
		wrap_align.wrap = True	
		wrap_align.horz = xlwt.Alignment.HORZ_LEFT
		wrap_align.vert = xlwt.Alignment.VERT_TOP
		self.wrap = xlwt.XFStyle()
		self.wrap.alignment = wrap_align
		
		title_font = xlwt.Font() #Create the Font
		title_font.name = 'Arial'
		title_font.bold = True
		title_font.height = 320 #16*20, for 16 point
		title = xlwt.XFStyle() #Create the Style
		title.font = title_font #Apply the Font to the Style
		title.alignment = wrap_align
		
		head1_font = xlwt.Font() #Create the Font
		head1_font.name = 'Arial'
		head1_font.bold = True
		head1_font.height = 240 #12*20, for 12 point
		head1 = xlwt.XFStyle() #Create the Style
		head1.font = head1_font #Apply the Font to the Style
		head1.alignment = wrap_align
		
		head2_font = xlwt.Font() #Create the Font
		head2_font.name = 'Arial'
		title_font.bold = True
		head2_font.height = 220 #11*20, for 11 point
		head2 = xlwt.XFStyle() #Create the Style
		head2.font = head2_font #Apply the Font to the Style	
		head2.alignment = wrap_align
		
		#add headings to workbook
		self.sheet.write(0,0, "Full Sim Data", style=title)
		self.sheet.write(1,0, "String outputs", style=head1)
			
		self.sheet.write(1,2, "Sim Time", style=head1)
		self.sheet.write_merge(1,1,4,9, "Positioning", style=head1)
		self.sheet.write(2,4, "Global SHREW Position (x,y)", style=head2)
		self.sheet.write(2,5, "Global SHREW Orientation (rad)", style=head2)
		self.sheet.write(2,6, "Global Goal Location (x,y)", style=head2)
		self.sheet.write(2,7, "Angle to Goal in SHREW frame (rad)", style=head2)
		self.sheet.write(2,8, "Ref. Heading in SHREW frame (rad)", style=head2)
		self.sheet.write(2,9, "Driving Direction", style=head2)

		self.sheet.write_merge(1,1,11,14, "Commanded Velocities to Wheels", style=head1)
		self.sheet.write(2,11, "Front Left Wheel", style=head2)
		self.sheet.write(2,12, "Front Right Wheel", style=head2)
		self.sheet.write(2,13, "Back Left Wheel", style=head2)
		self.sheet.write(2,14, "Back Right Wheel", style=head2)
			
		#states			
		self.sheet.write_merge(1,1,16,19, "Front Left Wheel States", style=head1)
		self.sheet.write(2,16, "Position (rad)", style=head2)
		self.sheet.write(2,17, "Speed (rad/s)", style=head2)
		self.sheet.write(2,18, "Joint Effort (Nm)", style=head2)
		self.sheet.write(2,19, "Reaction Force from Ground (N)", style=head2)
		
		self.sheet.write_merge(1,1,21,24, "Front Right Wheel States", style=head1)
		self.sheet.write(2,21, "Position (rad)", style=head2)
		self.sheet.write(2,22, "Speed (rad/s)", style=head2)
		self.sheet.write(2,23, "Joint Effort (Nm)", style=head2)				
		self.sheet.write(2,24, "Reaction Force from Ground (N)", style=head2)

		self.sheet.write_merge(1,1,26,29, "Back Left Wheel States", style=head1)
		self.sheet.write(2,26, "Position (rad)", style=head2)		
		self.sheet.write(2,27, "Speed (rad/s)", style=head2)
		self.sheet.write(2,28, "Joint Effort (Nm)", style=head2)					
		self.sheet.write(2,29, "Reaction Force from Ground (N)", style=head2)

		self.sheet.write_merge(1,1,31,34, "Back Right Wheel States", style=head1)
		self.sheet.write(2,31, "Position (rad)", style=head2)
		self.sheet.write(2,32, "Speed (rad/s)", style=head2)
		self.sheet.write(2,33, "Joint Effort (Nm)", style=head2)	
		self.sheet.write(2,34, "Reaction Force from Ground (N)", style=head2)
		
		#error
		self.sheet.write_merge(1,1,36,39, "Velocity Error on Wheels", style=head1)
		self.sheet.write(2,36, "Front Left Wheel", style=head2)
		self.sheet.write(2,37, "Front Right Wheel", style=head2)
		self.sheet.write(2,38, "Back Left Wheel", style=head2)
		self.sheet.write(2,39, "Back Right Wheel", style=head2)
		
		#slip
		self.sheet.write_merge(1,1,41,43, "Front Left Slip", style=head1)
		self.sheet.write(2,41, "Percvd. Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,42, "Actual Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,43, "Slip", style=head2)
		
		self.sheet.write_merge(1,1,45,47, "Front Right Slip", style=head1)
		self.sheet.write(2,45, "Percvd. Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,46, "Actual Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,47, "Slip", style=head2)
		
		self.sheet.write_merge(1,1,49,51, "Back Left Slip", style=head1)
		self.sheet.write(2,49, "Percvd. Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,50, "Actual Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,51, "Slip", style=head2)
		
		self.sheet.write_merge(1,1,53,55, "Back Right Slip", style=head1)
		self.sheet.write(2,53, "Percvd. Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,54, "Actual Lin. Rate (m/s)", style=head2)
		self.sheet.write(2,55, "Slip", style=head2)
		
		#reaction forces
		self.sheet.write_merge(1,1,57,60, "Front Left Reaction Force", style=head1)
		self.sheet.write(2,57, "x (N)", style=head2)
		self.sheet.write(2,58, "y (N)", style=head2)
		self.sheet.write(2,59, "z (N)", style=head2)
		self.sheet.write(2,60, "total (N)", style=head2)
		
		self.sheet.write_merge(1,1,62,65, "Front Right Reaction Force", style=head1)
		self.sheet.write(2,62, "x (N)", style=head2)
		self.sheet.write(2,63, "y (N)", style=head2)
		self.sheet.write(2,64, "z (N)", style=head2)
		self.sheet.write(2,65, "total (N)", style=head2)
		
		self.sheet.write_merge(1,1,67,70, "Back Left Reaction Force", style=head1)
		self.sheet.write(2,67, "x (N)", style=head2)
		self.sheet.write(2,68, "y (N)", style=head2)
		self.sheet.write(2,69, "z (N)", style=head2)
		self.sheet.write(2,70, "total (N)", style=head2)
		
		self.sheet.write_merge(1,1,72,75, "Back Right Reaction Force", style=head1)
		self.sheet.write(2,72, "x (N)", style=head2)
		self.sheet.write(2,73, "y (N)", style=head2)
		self.sheet.write(2,74, "z (N)", style=head2)
		self.sheet.write(2,75, "total (N)", style=head2)
		
		#reaction torques
		self.sheet.write_merge(1,1,77,80, "Front Left Reaction Torque", style=head1)
		self.sheet.write(2,77, "x (Nm)", style=head2)
		self.sheet.write(2,78, "y (Nm)", style=head2)
		self.sheet.write(2,79, "z (Nm)", style=head2)
		self.sheet.write(2,80, "total (Nm)", style=head2)
		
		self.sheet.write_merge(1,1,82,85, "Front Right Reaction Torque", style=head1)
		self.sheet.write(2,82, "x (Nm)", style=head2)
		self.sheet.write(2,83, "y (Nm)", style=head2)
		self.sheet.write(2,84, "z (Nm)", style=head2)
		self.sheet.write(2,85, "total (Nm)", style=head2)
		
		self.sheet.write_merge(1,1,87,90, "Back Left Reaction Torque", style=head1)
		self.sheet.write(2,87, "x (Nm)", style=head2)
		self.sheet.write(2,88, "y (Nm)", style=head2)
		self.sheet.write(2,89, "z (Nm)", style=head2)
		self.sheet.write(2,90, "total (Nm)", style=head2)
		
		self.sheet.write_merge(1,1,92,95, "Back Right Reaction Torque", style=head1)
		self.sheet.write(2,92, "x (Nm)", style=head2)
		self.sheet.write(2,93, "y (Nm)", style=head2)
		self.sheet.write(2,94, "z (Nm)", style=head2)
		self.sheet.write(2,95, "total (Nm)", style=head2)
		
		self.write_row=3 
		
#-------------------------------------------callbacks---------------------------------------
		
		
	def contacts_state_FL_callback(self, contact_state_msg):
		if not (self.last_contacts_FL_msg==contact_state_msg) and (contact_state_msg.states!=[]):
			self.last_contacts_FL_msg=contact_state_msg
			self.contact_location_FL=contact_state_msg.states[0].collision2_name
			self.reaction_forces_FL=contact_state_msg.states[0].total_wrench.force
			self.reaction_torques_FL=contact_state_msg.states[0].total_wrench.torque
			self.total_reaction_force_FL=(self.reaction_forces_FL.x**2+self.reaction_forces_FL.y**2+self.reaction_forces_FL.z**2)**(1/2)
			self.total_reaction_torque_FL=(self.reaction_torques_FL.x**2+self.reaction_torques_FL.y**2+self.reaction_torques_FL.z**2)**(1/2)
		
	def contacts_state_FR_callback(self, contact_state_msg):
		if not (self.last_contacts_FR_msg==contact_state_msg) and (contact_state_msg.states!=[]):
			self.last_contacts_FR_msg=contact_state_msg
			self.contact_location_FR=contact_state_msg.states[0].collision2_name
			self.reaction_forces_FR=contact_state_msg.states[0].total_wrench.force
			self.reaction_torques_FR=contact_state_msg.states[0].total_wrench.torque
			self.total_reaction_force_FR=(self.reaction_forces_FR.x**2+self.reaction_forces_FR.y**2+self.reaction_forces_FR.z**2)**(1/2)
			self.total_reaction_torque_FR=(self.reaction_torques_FR.x**2+self.reaction_torques_FR.y**2+self.reaction_torques_FR.z**2)**(1/2)
		
	def contacts_state_BL_callback(self, contact_state_msg):
		if not (self.last_contacts_BL_msg==contact_state_msg) and (contact_state_msg.states!=[]):
			self.last_contacts_BL_msg=contact_state_msg
			self.contact_location_BL=contact_state_msg.states[0].collision2_name
			self.reaction_forces_BL=contact_state_msg.states[0].total_wrench.force
			self.reaction_torques_BL=contact_state_msg.states[0].total_wrench.torque
			self.total_reaction_force_BL=(self.reaction_forces_BL.x**2+self.reaction_forces_BL.y**2+self.reaction_forces_BL.z**2)**(1/2)
			self.total_reaction_torque_BL=(self.reaction_torques_BL.x**2+self.reaction_torques_BL.y**2+self.reaction_torques_BL.z**2)**(1/2)
		
	def contacts_state_BR_callback(self, contact_state_msg):
		if not (self.last_contacts_BR_msg==contact_state_msg) and (contact_state_msg.states!=[]):
			self.last_contacts_BR_msg=contact_state_msg
			self.contact_location_BR=contact_state_msg.states[0].collision2_name
			self.reaction_forces_BR=contact_state_msg.states[0].total_wrench.force
			self.reaction_torques_BR=contact_state_msg.states[0].total_wrench.torque
			self.total_reaction_force_BR=(self.reaction_forces_BR.x**2+self.reaction_forces_BR.y**2+self.reaction_forces_BR.z**2)**(1/2)
			self.total_reaction_torque_BR=(self.reaction_torques_BR.x**2+self.reaction_torques_BR.y**2+self.reaction_torques_BR.z**2)**(1/2)

	def link_state_callback(self, link_state_msg):
		if not self.last_link_msg==link_state_msg:
			self.last_link_msg=link_state_msg
			#print(link_state_msg.name[17])
			##print(link_state_msg.name[18])
			#print(link_state_msg.name[24])
			#print(link_state_msg.name[25])
			
			self.perceived_fl_linear_rate=abs(self.current_fl_wheel_speed*WHEEL_RADIUS)
			self.perceived_fr_linear_rate=abs(self.current_fr_wheel_speed*WHEEL_RADIUS)
			self.perceived_bl_linear_rate=abs(self.current_bl_wheel_speed*WHEEL_RADIUS)
			self.perceived_br_linear_rate=abs(self.current_br_wheel_speed*WHEEL_RADIUS)
			
			#print("Per. FL: "+str(round(self.perceived_fl_linear_rate,3))+" Per. FR: "+str(round(self.perceived_fr_linear_rate,3))+" Per. BL: "+str(round(self.perceived_bl_linear_rate,3))+" Per. BR: "+str(round(self.perceived_br_linear_rate,3)))
			
			self.actual_fl_linear_rate=(link_state_msg.twist[15].linear.x**2+link_state_msg.twist[15].linear.y**2)**(1/2)
			self.actual_fr_linear_rate=(link_state_msg.twist[16].linear.x**2+link_state_msg.twist[16].linear.y**2)**(1/2)
			self.actual_bl_linear_rate=(link_state_msg.twist[20].linear.x**2+link_state_msg.twist[20].linear.y**2)**(1/2)
			self.actual_br_linear_rate=(link_state_msg.twist[21].linear.x**2+link_state_msg.twist[21].linear.y**2)**(1/2)
	
			#print("FL: "+str(round(self.actual_fl_linear_rate,3))+" FR: "+str(round(self.actual_fr_linear_rate,3))+" BL: "+str(round(self.actual_bl_linear_rate,3))+" BR: "+str(round(self.actual_br_linear_rate,3)))	
	
			self.slip_fl=(self.perceived_fl_linear_rate-self.actual_fl_linear_rate)/DESIRED_LINEAR_VELOCITY
			self.slip_fr=(self.perceived_fr_linear_rate-self.actual_fr_linear_rate)/DESIRED_LINEAR_VELOCITY
			self.slip_bl=(self.perceived_bl_linear_rate-self.actual_bl_linear_rate)/DESIRED_LINEAR_VELOCITY
			self.slip_br=(self.perceived_br_linear_rate-self.actual_br_linear_rate)/DESIRED_LINEAR_VELOCITY
			
			self.slip_fl=max(0, self.slip_fl)
			self.slip_fl=min(self.slip_fl, 1)
			
			self.slip_fr=max(0, self.slip_fr)
			self.slip_fr=min(self.slip_fr, 1)
			
			self.slip_bl=max(0, self.slip_bl)
			self.slip_bl=min(self.slip_bl, 1)
			
			self.slip_br=max(0, self.slip_br)
			self.slip_br=min(self.slip_br, 1)
	
	def joint_state_callback(self, joint_state_msg):
		if not self.last_joint_msg==joint_state_msg: #checking to see if there is an update
			self.last_joint_msg=joint_state_msg #reset to keep from receiving too many messages
			
			self.current_joint_msg_stamp=str(joint_state_msg.header.stamp.secs)+str(joint_state_msg.header.stamp.nsecs)
			self.current_fl_wheel_position=joint_state_msg.position[1]
			self.current_fr_wheel_position=joint_state_msg.position[2]
			self.current_bl_wheel_position=joint_state_msg.position[3]
			self.current_br_wheel_position=joint_state_msg.position[4]
			
			self.current_fl_wheel_speed=joint_state_msg.velocity[1]
			self.current_fr_wheel_speed=joint_state_msg.velocity[2]
			self.current_bl_wheel_speed=joint_state_msg.velocity[3]
			self.current_br_wheel_speed=joint_state_msg.velocity[4]
			
			self.current_fl_wheel_effort=joint_state_msg.effort[1]
			self.current_fr_wheel_effort=joint_state_msg.effort[2]
			self.current_bl_wheel_effort=joint_state_msg.effort[3]
			self.current_br_wheel_effort=joint_state_msg.effort[4]

	def nav_goal_callback(self, nav_goal_msg): 
		self.wheel_vel_pub.publish(Quaternion(0,0,0,0))
		self.nav_goal=nav_goal_msg
		print("New navigation goal received. Beginning navigation...\n")
		self.sheet.write(self.write_row,0,"New navigation goal received. Beginning navigation...", style=self.wrap)
		self.sheet.write(self.write_row,2, str(rospy.get_rostime()))
		self.sheet.write(self.write_row,6, str([self.nav_goal.pose.position.x, self.nav_goal.pose.position.y]))
		self.sheet.write(self.write_row,11, 0)
		self.sheet.write(self.write_row,12, 0)
		self.sheet.write(self.write_row,13, 0)
		self.sheet.write(self.write_row,14, 0)
		self.write_row+=1
		rospy.sleep(1)
			
	
#-----------------------------------------functions to move---------------------------------
	
	def get_current_global_pose(self):
		try:
			trans = self.tfBuffer.lookup_transform('world', 'shrew_robot_fixed_joint', rospy.Time.now(), rospy.Duration(1.0))
			self.current_global_pose = trans.transform
		except tf2_ros.LookupException:
			print('Lookup Exception')

		except tf2_ros.ConnectivityException:
			print('Connectivity Exception')

		except tf2_ros.ExtrapolationException:
			print('Extrapolation Exception')
		
	def find_remaining_navigation(self):
		if (self.nav_goal.pose.position.x-0.25<=self.current_global_pose.translation.x<=self.nav_goal.pose.position.x+0.25) and (self.nav_goal.pose.position.y-0.25<=self.current_global_pose.translation.y<=self.nav_goal.pose.position.y+0.25):
			wheel_command = Quaternion(0,0,0,0)
			self.wheel_vel_pub.publish(wheel_command)
			print('At Goal. Waiting for new goal...\n')
			self.sheet.write(self.write_row,0,"At Goal. Waiting for new goal...", style=self.wrap)
			self.sheet.write(self.write_row,2, str(rospy.get_rostime()))
			self.sheet.write(self.write_row,11, 0)
			self.sheet.write(self.write_row,12, 0)
			self.sheet.write(self.write_row,13, 0)
			self.sheet.write(self.write_row,14, 0)
			self.write_row+=1
			rospy.wait_for_message('/move_base_simple/goal', PoseStamped)
		else:
			self.remaining_x = self.nav_goal.pose.position.x-self.current_global_pose.translation.x
			self.remaining_y = self.nav_goal.pose.position.y-self.current_global_pose.translation.y
	
	def set_velocity_parameters(self):
		quaternion=(self.current_global_pose.rotation.x,
			self.current_global_pose.rotation.y,
			self.current_global_pose.rotation.z,
			self.current_global_pose.rotation.w)
		euler=tf.transformations.euler_from_quaternion(quaternion)
		#roll=euler[0]
		#pitch=euler[1]
		self.yaw=euler[2]+np.pi #current robot heading as euler angle in world frame (0, 2pi)
		current_angle_to_goal=np.arctan2((self.remaining_y), (self.remaining_x))+np.pi #direction to goal in robot frame (0, 2pi)
		desired_heading_angle=(self.yaw-current_angle_to_goal)%(2*np.pi)
		self.current_angle_to_goal=current_angle_to_goal
		self.ref_heading=desired_heading_angle
		
		if (0<=desired_heading_angle<=np.pi/2) or (3*np.pi/2<=desired_heading_angle<=2*np.pi):	
			self.desired_velocity=DESIRED_LINEAR_VELOCITY
			#adjusting for overflowing 0,2pi
			if desired_heading_angle<=np.pi/2:
				#print("Case 1")
				self.front_yaw_ref=-desired_heading_angle
				self.rear_yaw_ref=self.front_yaw_ref
					
			elif desired_heading_angle>=3*np.pi/2:
				#print("Case 2")
				self.front_yaw_ref=-(desired_heading_angle-2*np.pi)
				self.rear_yaw_ref=self.front_yaw_ref
		
		else:
			self.desired_velocity=-DESIRED_LINEAR_VELOCITY
			#print("Case 3")
			self.front_yaw_ref=(np.pi-desired_heading_angle)
			self.rear_yaw_ref=self.front_yaw_ref

		
		self.front_yaw_ref=max(min(YAW_LIM, self.front_yaw_ref), -YAW_LIM)
		self.rear_yaw_ref=max(min(YAW_LIM, self.rear_yaw_ref), -YAW_LIM)
		self.fl_ref_speed=self.desired_velocity/WHEEL_RADIUS
		self.fr_ref_speed=self.desired_velocity/WHEEL_RADIUS
		self.bl_ref_speed=self.desired_velocity/WHEEL_RADIUS
		self.br_ref_speed=self.desired_velocity/WHEEL_RADIUS
	
	def error_assess(self):
		self.front_yaw_error = self.front_yaw_ref
		self.rear_yaw_error = self.rear_yaw_ref
			
	def publish_commands_and_info(self):
		
		self.current_fl_speed_to_pub = self.fl_ref_speed - YAW_KP*self.front_yaw_error
		self.current_fr_speed_to_pub = self.fr_ref_speed + YAW_KP*self.front_yaw_error
		self.current_bl_speed_to_pub = self.bl_ref_speed - YAW_KP*self.rear_yaw_error
		self.current_br_speed_to_pub = self.br_ref_speed + YAW_KP*self.rear_yaw_error
		
		wheel_commands = Quaternion(self.current_fl_speed_to_pub, self.current_fr_speed_to_pub, self.current_bl_speed_to_pub, self.current_br_speed_to_pub)
		self.wheel_vel_pub.publish(wheel_commands)
		
		self.yaw_ref_pub.publish(Point(self.front_yaw_ref, self.rear_yaw_ref, 0))
		self.slip_pub.publish(Quaternion(self.slip_fl, self.slip_fr, self.slip_bl, self.slip_br))

		#0 signifies to plugin that the surface is low traction
		#1 signifies to the plugin that the surface is high traction
		#This is used to set the net forces applied to the wheels
		if (self.contact_location_FL=="terrain_patches::block_7::block_7_collision") or (self.contact_location_FL=="terrain_patches::block_6::block_6_collision"):
			self.contact_location_FL=0
		else:
			self.contact_location_FL=1
		
		if (self.contact_location_FR=="terrain_patches::block_7::block_7_collision") or (self.contact_location_FR=="terrain_patches::block_6::block_6_collision"):
			self.contact_location_FR=0
		else:
			self.contact_location_FR=1
			
		if (self.contact_location_BL=="terrain_patches::block_7::block_7_collision") or (self.contact_location_BL=="terrain_patches::block_6::block_6_collision"):
			self.contact_location_BL=0
		else:
			self.contact_location_BL=1
			
		if (self.contact_location_BR=="terrain_patches::block_7::block_7_collision") or (self.contact_location_BR=="terrain_patches::block_6::block_6_collision"):
			self.contact_location_BR=0
		else:
			self.contact_location_BR=1	
			
		self.contact_locations_pub.publish(Quaternion(self.contact_location_FL, self.contact_location_FR, self.contact_location_BL, self.contact_location_BR))

		self.contact_forces_FL_pub.publish(Quaternion(self.reaction_forces_FL.x, self.reaction_forces_FL.y, self.reaction_forces_FL.z, self.total_reaction_force_FL))
		self.contact_forces_FR_pub.publish(Quaternion(self.reaction_forces_FR.x, self.reaction_forces_FR.y, self.reaction_forces_FR.z, self.total_reaction_force_FR))
		self.contact_forces_BL_pub.publish(Quaternion(self.reaction_forces_BL.x, self.reaction_forces_BL.y, self.reaction_forces_BL.z, self.total_reaction_force_BL))
		self.contact_forces_BR_pub.publish(Quaternion(self.reaction_forces_BR.x, self.reaction_forces_BR.y, self.reaction_forces_BR.z, self.total_reaction_force_BR))
		self.contact_torques_FL_pub.publish(Quaternion(self.reaction_torques_FL.x, self.reaction_torques_FL.y, self.reaction_torques_FL.z, self.total_reaction_torque_FL))
		self.contact_torques_FR_pub.publish(Quaternion(self.reaction_torques_FR.x,self.reaction_torques_FR.y, self.reaction_torques_FR.z, self.total_reaction_torque_FR))
		self.contact_torques_BL_pub.publish(Quaternion(self.reaction_torques_BL.x, self.reaction_torques_BL.y, self.reaction_torques_BL.z, self.total_reaction_torque_BL))
		self.contact_torques_BR_pub.publish(Quaternion(self.reaction_torques_BR.x, self.reaction_torques_BR.y, self.reaction_torques_BR.z, self.total_reaction_torque_BR))

#----------------------------------------------log data-------------------------------------
	
	def log_joint_data(self):
		self.sheet.write(self.write_row,2, self.current_joint_msg_stamp)
		
		#wheel positions
		self.sheet.write(self.write_row,16,self.current_fl_wheel_position)
		self.sheet.write(self.write_row,21,self.current_fr_wheel_position)
		self.sheet.write(self.write_row,26,self.current_bl_wheel_position)
		self.sheet.write(self.write_row,31,self.current_br_wheel_position)
		
		#wheel speeds
		self.sheet.write(self.write_row,17,self.current_fl_wheel_speed)
		self.sheet.write(self.write_row,22,self.current_fr_wheel_speed)
		self.sheet.write(self.write_row,27,self.current_bl_wheel_speed)
		self.sheet.write(self.write_row,32,self.current_br_wheel_speed)
		
		#wheel effort
		self.sheet.write(self.write_row,18,self.current_fl_wheel_effort)
		self.sheet.write(self.write_row,23,self.current_fr_wheel_effort)
		self.sheet.write(self.write_row,28,self.current_bl_wheel_effort)
		self.sheet.write(self.write_row,33,self.current_br_wheel_effort)
		
	def log_positions(self):
		self.sheet.write(self.write_row,4,str([self.current_global_pose.translation.x, self.current_global_pose.translation.y]))
		self.sheet.write(self.write_row,5,self.yaw)
		self.sheet.write(self.write_row,6,str([self.nav_goal.pose.position.x, self.nav_goal.pose.position.y]))
		self.sheet.write(self.write_row,7, self.current_angle_to_goal)
		self.sheet.write(self.write_row,8, self.ref_heading)
		if self.desired_velocity > 0:
			self.sheet.write(self.write_row,9,"Forward")
		else:
			self.sheet.write(self.write_row,9,"Reverse")
			
	def log_errors_refs(self):
		
		self.sheet.write(self.write_row,36, self.current_fl_wheel_speed-self.current_fl_speed_to_pub)
		self.sheet.write(self.write_row,37, self.current_fr_wheel_speed-self.current_fr_speed_to_pub)
		self.sheet.write(self.write_row,38, self.current_bl_wheel_speed-self.current_bl_speed_to_pub)
		self.sheet.write(self.write_row,39, self.current_br_wheel_speed-self.current_br_speed_to_pub)
		
	def log_slip(self): #all in world frame
		#front left
		self.sheet.write(self.write_row,41, self.perceived_fl_linear_rate)
		self.sheet.write(self.write_row,42, self.actual_fl_linear_rate)
		self.sheet.write(self.write_row,43, self.slip_fl_linear_rate)
		
		#front right
		self.sheet.write(self.write_row,45, self.perceived_fr_linear_rate)
		self.sheet.write(self.write_row,46, self.actual_fr_linear_rate)
		self.sheet.write(self.write_row,47, self.slip_fr_linear_rate)
		
		#back left
		self.sheet.write(self.write_row,49, self.perceived_bl_linear_rate)
		self.sheet.write(self.write_row,50, self.actual_bl_linear_rate)
		self.sheet.write(self.write_row,51, self.slip_bl_linear_rate)
		
		#back right
		self.sheet.write(self.write_row,53, self.perceived_br_linear_rate)
		self.sheet.write(self.write_row,54, self.actual_br_linear_rate)
		self.sheet.write(self.write_row,55, self.slip_br_linear_rate)
	
	def log_commands(self):
		self.sheet.write(self.write_row, 11, self.current_fl_speed_to_pub)
		self.sheet.write(self.write_row, 12, self.current_fr_speed_to_pub)
		self.sheet.write(self.write_row, 13, self.current_bl_speed_to_pub)
		self.sheet.write(self.write_row, 14, self.current_br_speed_to_pub)

	def log_contact_forces(self):
		#reaction forces
		self.sheet.write(self.write_row,57, self.reaction_forces_FL.x)
		self.sheet.write(self.write_row,58, self.reaction_forces_FL.y)
		self.sheet.write(self.write_row,59, self.reaction_forces_FL.z)
		self.sheet.write(self.write_row,60, self.total_reaction_force_FL)
		
		self.sheet.write(self.write_row,62, self.reaction_forces_FR.x)
		self.sheet.write(self.write_row,63, self.reaction_forces_FR.y)
		self.sheet.write(self.write_row,64, self.reaction_forces_FR.z)
		self.sheet.write(self.write_row,65, self.total_reaction_force_FR)

		self.sheet.write(self.write_row,67, self.reaction_forces_BL.x)
		self.sheet.write(self.write_row,68, self.reaction_forces_BL.y)
		self.sheet.write(self.write_row,69, self.reaction_forces_BL.z)
		self.sheet.write(self.write_row,70, self.total_reaction_force_BL)
		
		self.sheet.write(self.write_row,72, self.reaction_forces_BR.x)
		self.sheet.write(self.write_row,73, self.reaction_forces_BR.y)
		self.sheet.write(self.write_row,74, self.reaction_forces_BR.z)
		self.sheet.write(self.write_row,75, self.total_reaction_force_BR)
		
		#reaction torques
		self.sheet.write(self.write_row,77, self.reaction_torques_FL.x)
		self.sheet.write(self.write_row,78, self.reaction_torques_FL.y)
		self.sheet.write(self.write_row,79, self.reaction_torques_FL.z)
		self.sheet.write(self.write_row,80, self.total_reaction_torque_FL)
		
		self.sheet.write(self.write_row,82, self.reaction_torques_FR.x)
		self.sheet.write(self.write_row,83, self.reaction_torques_FR.y)
		self.sheet.write(self.write_row,84, self.reaction_torques_FR.z)
		self.sheet.write(self.write_row,85, self.total_reaction_torque_FR)
		
		self.sheet.write(self.write_row,87, self.reaction_torques_BL.x)
		self.sheet.write(self.write_row,88, self.reaction_torques_BL.y)
		self.sheet.write(self.write_row,89, self.reaction_torques_BL.z)
		self.sheet.write(self.write_row,90, self.total_reaction_torque_BL)
		
		self.sheet.write(self.write_row,92, self.reaction_torques_BR.x)
		self.sheet.write(self.write_row,93, self.reaction_torques_BR.y)
		self.sheet.write(self.write_row,94, self.reaction_torques_BR.z)
		self.sheet.write(self.write_row,95, self.total_reaction_torque_BR)

	def log_data(self):
		self.log_joint_data() #log the joint state data
		self.log_commands()
		self.log_positions()
		self.log_errors_refs()
		self.log_slip()
		self.log_contact_forces()
		self.write_row+=1
		
	def finalize_logs(self):
		wb_save_name=('full_sim_fixed_joint_data.xls')
		self.wb.save(os.path.join(self.new_log_path, wb_save_name))

#-------------------------------------------end execution-----------------------------------

	def shutdown(self):
		print('Shutting down without saving...\n')
		time.sleep(1)
		sys.exit(1)

	def rospy_close_callback(self):
		wheel_command = Quaternion(0,0,0,0)
		self.wheel_vel_pub.publish(wheel_command)
		time.sleep(1)
		print('\nTerminating ROS Node: '+rospy.get_name())
		time.sleep(1)
		print('\nFinalizing logs and closing files.')
		self.finalize_logs()
		print('\nClosing rqt_gui windows.')
		self.launch.shutdown()
		
		
#------------------------------------------------main---------------------------------------

	def main(self):
		rate=rospy.Rate(FREQUENCY)
		rospy.wait_for_message('/move_base_simple/goal', PoseStamped)
		while not rospy.is_shutdown():
			self.get_current_global_pose()
			self.find_remaining_navigation()
			self.set_velocity_parameters()
			self.error_assess()
			self.publish_commands_and_info()
			self.log_data()
			rate.sleep()


#--------------------------------------------------------------------------------------------


	 
if __name__=="__main__":
	t=ShrewMove() #instance of ShrewMove and open the nodes
	t.main() #executing instance of node
