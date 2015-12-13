#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_followme')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from std_msgs.msg import String
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		
		# initialize follow me application 
		self.fm_state 		= 0
		self.fm_acq_cnt 	= 0
		self.fm_active 		= 0
		self.fm_dbg_cnt 	= 0
		self.fm_exec_tmr_0 	= 15 # ~4hz
		self.fm_exec_tmr 	= self.fm_exec_tmr_0
		self.fm_dist_max    = 100 # cm

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)
		
		# Publisher for follow me application
		# Note that we do not need to explicitly initialize this topic, it will be initialized automatically on first usage.
		self.pubFollowMe = rospy.Publisher('/ardrone/follow_me', String)
		

	def ReceiveNavdata(self,navdata):
		#
		self.status = navdata.state
		
		# run Follow Me at subinterval of navdata receipt
		self.fm_exec_tmr -= 1
		if(self.fm_active == 1 and self.fm_exec_tmr <= 0):
			# follow me application parsing
			self.tags_cnt  = navdata.tags_count
			self.tags_xc   = navdata.tags_xc
			self.tags_yc   = navdata.tags_yc
			self.tags_dist = navdata.tags_distance
			#
			self.FollowMe()
			
			# reload follow me execution timer
			self.fm_exec_tmr = self.fm_exec_tmr_0			
		
	def FollowMe(self):

		self.fm_active = 1
		self.fm_dbg_cnt += 1
		
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:

			if(self.fm_state == 0):
				# transition to Target Acquisition state 
				self.fm_state = 1
			
			if(self.fm_state == 1):
				if(self.tags_cnt > 0):
					# sucessful acquisition, transition to Target Lock state
					self.fm_state   = 2
				else:
					self.fm_acq_cnt += 1
					
				# add code to use the acq counter
			
			if(self.fm_state == 2):	
				if(self.tags_cnt < 0):
					# lost target, go back to Target Acquisition state
					self.fm_state   = 1
					self.fm_acq_cnt = 0
				else:
					# compute vel_cmd to position target in center of view
					move_left = 0	#
					move_fwd  = 0
					yaw_left  = 0
					move_up   = 0
					
					# left to right
					if self.tags_xc < 300:
						move_left = -1
					elif self.tags_xc > 700:
						move_left = 1
					
					# up and down
					if self.tags_yc < 300:
						move_up   = -1
					elif self.tags_yc > 700:
						move_up   =  1				
						
					# forward and back
					if self.tags_dist > self.fm_dist_max:
						move_fwd = 1
					
					#controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
					self.SetCommand(move_left, move_fwd, yaw_left, move_up)
		else:
			# drone parked
			self.fm_state = 0
					
		# publish Follow Me data  
 		fm_msg = String()
 		fm_msg.data = 'Received tag count: {0:d}'.format(self.tags_cnt)
 		self.pubFollowMe.publish(fm_msg)
 		
 		fm_msg.data = 'Follow Me state: {0:d}'.format(self.fm_state)
 		self.pubFollowMe.publish(fm_msg)

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)

