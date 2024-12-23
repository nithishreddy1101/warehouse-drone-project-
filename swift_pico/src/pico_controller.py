#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			/throttle_pid
						/pitch_pid
						/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2.0, 2.0, 17.5]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		self.Kp = [1000*0.02, 1000*0.02, 901*0.02]
		self.Ki = [0.0001*3, 0.0001*3, 0.00]
		self.Kd = [1280*0.6, 1280*0.6, 1262*0.6]


		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]
		self.previous_error=[0.0,0.0,0.0]
		self.pid_error=PIDError()
		self.iterm=[0.0,0.0,0.0]
		self.error=[0.0,0.0,0.0]
		self.diff =[0.0,0.0,0.0]





		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.060 # in seconds

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)

		#------------------------Add other ROS Subscribers here-----------------------------------------------------
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

		self.arm()  # ARMING THE DRONE
		self.create_timer(self.sample_time,self.pid)
		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)


	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	def whycon_callback(self, msg):
		self.drone_position[0] = msg.poses[0].position.x 
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.02 
		self.Ki[2] = alt.ki * 0.0001
		self.Kd[2] = alt.kd * 0.6
	
	def pitch_set_pid(self,alt):
		self.Kp[1] = alt.kp * 0.02
		self.Ki[1] = alt.ki * 0.0001
		self.Kd[1] = alt.kd * 0.6

	def roll_set_pid(self,alt):
		self.Kp[0] = alt.kp * 0.02
		self.Ki[0] = alt.ki * 0.0001
		self.Kd[0] = alt.kd * 0.6

	def pid(self):

		self.error[0] = self.drone_position[0]-self.setpoint[0]
		self.error[1] = self.drone_position[1]-self.setpoint[1]
		self.error[2] = self.drone_position[2]-self.setpoint[2]
		self.iterm[0]=self.iterm[0]+self.error[0]
		self.iterm[1]=self.iterm[1]+self.error[1]
		self.iterm[2]=self.iterm[2]+self.error[2]
		self.diff[0]=self.error[0]-self.previous_error[0]
		self.diff[1]=self.error[1]-self.previous_error[1]
		self.diff[2]=self.error[2]-self.previous_error[2]

		self.get_logger().info(f" drone_pos ::{self.drone_position}")
	
		self.out_roll     = int(self.Kp[0]*self.error[0]+self.Kd[0]*self.diff[0]+self.Ki[0]*self.iterm[0])
		self.out_pitch    = int(self.Kp[1]*self.error[1]+self.Kd[1]*self.diff[1]+self.Ki[1]*self.iterm[1])
		self.out_throttle = int(self.Kp[2]*self.error[2]+self.Kd[2]*self.diff[2]+self.Ki[2]*self.iterm[2])

		# self.get_logger().info(f" iterm ::{self.iterm[2]} i_error {self.Ki[2]*self.iterm[2]}")

		self.cmd.rc_throttle = 1500+self.out_throttle
		self.cmd.rc_roll = 1500-self.out_roll
		self.cmd.rc_pitch = 1500+self.out_pitch

		if self.cmd.rc_roll > self.max_values[0]:
			self.cmd.rc_roll = self.max_values[0]
		if self.cmd.rc_pitch > self.max_values[1]:
			self.cmd.rc_pitch = self.max_values[1]
		if self.cmd.rc_throttle > self.max_values[2]:
			self.cmd.rc_throttle = self.max_values[2]

		if self.cmd.rc_roll <self.min_values[0]:
			self.cmd.rc_roll = self.min_values[0]
		if self.cmd.rc_pitch < self.min_values[1]:
			self.cmd.rc_pitch = self.min_values[1]
		if self.cmd.rc_throttle < self.min_values[2]:
			self.cmd.rc_throttle = self.min_values[2]
		
		self.previous_error[0]=self.error[0]
		self.previous_error[1]=self.error[1]
		self.previous_error[2]=self.error[2]
		
		self.command_pub.publish(self.cmd)

		self.pid_error.roll_error=self.error[0]
		self.pid_error.pitch_error=self.error[1]
		self.pid_error.throttle_error= self.error[2]

		self.pid_error_pub.publish(self.pid_error)



def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
