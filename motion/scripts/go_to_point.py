#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations

import math

# robot state variables
current_position = Point()
yaw=0

# robot state
state=0

#goal
desired_position = Point()
desired_position.x = 5
desired_position.y = 5
desired_position.z = 0

#precision
yaw_precision = (3*math.pi)/180
dist_precision = 0.3


def change_state(x):
	global state
	state= x


def fix_yaw(des_pos):

	global current_position,yaw,yaw_precision,pub
	des_yaw = math.atan2(des_pos.y-current_position.y,des_pos.x-current_position.x)
	err_yaw = yaw-des_yaw

	twist_msg = Twist()

	if yaw_precision<err_yaw or yaw_precision>err_yaw:
		if err_yaw>0:
			twist_msg.angular.z = 0.5
		elif err_yaw<0:
			twist_msg.angular.z = -0.5
		

	if  -yaw_precision<=err_yaw<=yaw_precision :
		change_state(1)

	pub.publish(twist_msg)

def head_straight(des_pos):
	global pub,state,yaw,yaw_precision,dist_precision

	pos  = math.sqrt(pow(current_position.x-des_pos.x,2)+pow(current_position.y-des_pos.y,2))
	des_yaw = math.atan2(des_pos.y-current_position.y,des_pos.x-current_position.x)
	err_yaw = yaw-des_yaw

	if  pos < dist_precision :
		change_state(2)
	else :
		twist_msg=Twist()
		twist_msg.linear.x=0.6
		pub.publish(twist_msg)

	if err_yaw>yaw_precision or err_yaw<-yaw_precision:
		change_state(0)



def stop():
	global pub
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg) 


def callback_odom(msg):
	global current_position
	global yaw

	
	current_position = msg.pose.pose.position 
	quaternion = ( msg.pose.pose.orientation.x,
					msg.pose.pose.orientation.y,
					msg.pose.pose.orientation.z,
					msg.pose.pose.orientation.w )

	euler = transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]


def main():
	global pub
	rospy.init_node('go_to_point')
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	sub_odom = rospy.Subscriber('/odom',Odometry,callback_odom)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		print("state = ",state)
		if state==0:
			fix_yaw(desired_position)
		elif state==1:
			head_straight(desired_position)
		elif state==2:
			stop() 

		rate.sleep()	



if __name__ == '__main__':
	main()

