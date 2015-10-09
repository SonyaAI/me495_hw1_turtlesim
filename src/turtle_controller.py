#!/usr/bin/env python

import rospy
from math import pi, cos, sin, sqrt, atan, atan2
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist, Vector3

#Time t -> [0,T]
T = 10 # Default value for the period, during which turtle follows one '8' trajectory.
f = 50 # frequency at which velocities are published to cmd_vel
init_location = [5.5, 5.5, 0] # Default location/orientation [x_0, y_0, theta_0]

"""
The following function initializes period T, turtle location and orientation (x,y,theta).
"""
def init_turtle():
	#Get private parameter T (period) from parameter server. Private parameter is within namespace. 
	T = rospy.get_param('~period', 10) # in launch file private param inside node tag

	#Get location and orientation at t=0
	rospy.wait_for_service('turtle1/teleport_absolute')
	location = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
	init_location = location(5.5, 5.5, 0) #middle of window, facing east

"""
The following function calculates translational and rotational velocity at time.
"""
def calc_velocities(time):
	t = time
	x = 3 * sin(4*pi*t/T)
	y = 3 * sin(2*pi*t/T)

	v_x = 12*pi/T*cos(4*pi*t/T) # dx/dt
	v_y = 6*pi/T*cos(2*pi*t/T) # dy/dt
	v_t = sqrt(v_x*v_x + v_y*v_y) #Translational velocity -> translational speed (scalar)

	theta = atan2(v_x,v_y) #use atan2(v_x,v_y) instead to figure out which quadrant to use
	a_x = - 48*pi*pi/(T*T) * sin(4*pi*t/T)
	a_y = -12*pi*pi/(T*T) * sin(2*pi*t/T)
	v_r =  (v_x*a_y - v_y*a_x)/(v_t*v_t)# Angular velocity is d(theta)/dt 
	velocities = [v_t, v_r]
	return velocities

"""
The following function publishes translational and rotaional velocities to the cmd_vel topic.
Turtlesim_node is listening to cmd_vel topic and then moves the turtle according to the new values.
"""
def pub_velocities():
	init_turtle()

	#ROS Publisher tutorial 
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
	rospy.init_node('turtle_controller_node', anonymous=True) #For the future: need for subscribing 

	#Following loop from ROS Time tutorial
	rate = rospy.Rate(f) #Rate of a loop. f in Hz. 
	while not rospy.is_shutdown():
		time = rospy.get_time()

		v_t = Vector3(calc_velocities(time)[0], 0, 0)
		v_r = Vector3(0, 0, calc_velocities(time)[1])
		velocities = Twist(v_t, v_r)
		pub.publish(velocities) 
		try:
			rate.sleep() #Throws exceptions if sleep is interrupted
		except rospy.ROSInterruptException:
			pass
		

if __name__ == '__main__':
    pub_velocities()