#!/usr/bin/env python3

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from waypoint_navigation.action import NavToWaypoint
#import the action

#pico control specific libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0


        self.drone_position = [0.0, 0.0, 31.0, 0.0]
        self.setpoint = [2, 2, 27, 0] 
        self.dtime = 0

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        self.Kp = [0, 0, 0, 0]
        self.Ki = [0, 0, 0, 0]
        self.Kd = [0, 0, 0, 0]

        self.Kp = [4.5, 4.5, 31, 5]
        self.Ki = [0, 0, 0, 0]
        self.Kd = [100.0, 100.0, 450, 10]

        self.max_values = [2000,2000,2000]
        self.min_values = [1000,1000,1000]
        

        self.previous_error=[0.0,0.0,0.0,0.0]
        self.pid_error=PIDError()
        self.iterm=[0.0,0.0,0.0,0.0]
        self.error=[0.0,0.0,0.0,0.0]
        self.diff =[0.0,0.0,0.0,0.0]

        self.sample_time = 0.060

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)


        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        self.action_server_=ActionServer(
            self,
            NavToWaypoint,
            "waypoint_navigation",
            execute_callback=self.execute_callback,
            callback_group=self.action_callback_group
        )
        #create an action server for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.
        #include the action_callback_group in the action server. Refer to executors in ROS 2 concepts

        
        self.arm()

        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

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
        self.command_pub.publish(self.cmd)


    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z


        self.dtime = msg.header.stamp.sec

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 1.0 
        self.Ki[2] = alt.ki * 0.001
        self.Kd[2] = alt.kd * 1.0


    def pitch_set_pid(self,alt):
        self.Kp[1] = alt.kp * 0.01
        self.Ki[1] = alt.ki * 0.0001
        self.Kd[1] = alt.kd * 0.1

    def roll_set_pid(self,alt):
        self.Kp[0] = alt.kp * 0.01
        self.Ki[0] = alt.ki * 0.0001
        self.Kd[0] = alt.kd * 0.1


    #Define callback function like altitide_set_pid to tune pitch, roll


    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)
        self.drone_position[3] = self.yaw_deg	

    def pid(self):

        for i in range(4):
            self.error[i] = self.drone_position[i]-self.setpoint[i]
            self.iterm[i]=self.iterm[i]+self.error[i]
            self.diff[i]=self.error[i]-self.previous_error[i]
            self.previous_error[i]=self.error[i]
	
        self.out_roll     = int(self.Kp[0]*self.error[0]+self.Kd[0]*self.diff[0]+self.Ki[0]*self.iterm[0])
        self.out_pitch    = int(self.Kp[1]*self.error[1]+self.Kd[1]*self.diff[1]+self.Ki[1]*self.iterm[1])
        self.out_throttle = int(self.Kp[2]*self.error[2]+self.Kd[2]*self.diff[2]+self.Ki[2]*self.iterm[2])
        self.out_yaw      = int(self.Kp[3]*self.error[3]+self.Kd[3]*self.diff[3]+self.Ki[3]*self.iterm[3])
    
        self.cmd.rc_throttle = 1500+self.out_throttle
        self.cmd.rc_roll = 1500-self.out_roll
        self.cmd.rc_pitch = 1500+self.out_pitch
        self.cmd.rc_yaw = 1500+self.out_yaw

        if self.cmd.rc_roll > self.max_values[0]:
            self.cmd.rc_roll = self.max_values[0]
        elif self.cmd.rc_roll< self.min_values[0]:
            self.cmd.rc_roll = self.min_values[0]
        if self.cmd.rc_pitch > self.max_values[1]:
            self.cmd.rc_pitch = self.max_values[1]
        elif self.cmd.rc_roll < self.min_values[1]:
            self.cmd.rc_roll = self.min_values[1]
        if self.cmd.rc_throttle > self.max_values[2]:
            self.cmd.rc_throttle = self.max_values[2]
        elif self.cmd.rc_roll < self.min_values[2]:
            self.cmd.rc_roll = self.min_values[2]



        self.command_pub.publish(self.cmd)

        self.pid_error.roll_error=self.error[0]
        self.pid_error.pitch_error=self.error[1]
        self.pid_error.throttle_error= self.error[2]
        self.pid_error.yaw_error= self.error[3]


        self.command_pub.publish(self.cmd)
        self.pid_error_pub.publish(self.pid_error)

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        self.setpoint[0] = goal_handle.request.waypoint.position.x
        self.setpoint[1] = goal_handle.request.waypoint.position.y
        self.setpoint[2] = goal_handle.request.waypoint.position.z
        self.get_logger().info(f'New Waypoint Set: {self.setpoint}')
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime
        feedback_msg=NavToWaypoint.Feedback()
        #create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        
        #--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
        # This will help you to analyse the drone behaviour and help you to tune the PID better.

        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            feedback_msg.current_waypoint.header.stamp.sec = self.max_time_inside_sphere

            goal_handle.publish_feedback(feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 1) #the value '0.4' is the error range in the whycon coordinates that will be used for grading. 
            #You can use greater values initially and then move towards the value '0.4'. This will help you to check whether your waypoint navigation is working properly. 

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        pass
            
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        self.point_in_sphere_start_time = self.dtime
                        self.get_logger().info('Drone in sphere for 1st time')

            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.get_logger().info('Drone in sphere')
                             
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.get_logger().info('Drone out of sphere')
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.point_in_sphere_start_time = None

            if self.time_inside_sphere > self.max_time_inside_sphere:
                 self.max_time_inside_sphere = self.time_inside_sphere

            if self.max_time_inside_sphere >= 3:
                 break
                        

        goal_handle.succeed()
        result=NavToWaypoint.Result()
        #create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        result.hov_time = self.dtime - self.duration
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         waypoint_server.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()
