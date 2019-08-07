#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
from m2wr_description.srv import *

import math

# robot state variables
current_yaw=0
desired_yaw = 0

# robot state
state=0

#precision
yaw_precision = (1*math.pi)/180
dist_precision = 0.3

min_dist_wall=[0,0]
laser_val = LaserScan()

def change_state(x):
	global state
	state = x

def find_wall():
	global pub,laser_val

	message = Twist()

	print(laser_val)

	if min(laser_val.ranges)<=10:
		message.linear.x=0
		message.angular.z=0
		change_state(1)
	else:
		message.linear.x=0.5

	pub.publish(message)

def turn():
	global current_yaw,yaw_precision,pub,desired_yaw
	message = Twist()

	print("current_yaw : ",current_yaw)
	print("desired_yaw : ",desired_yaw)

	yaw_err = desired_yaw - current_yaw
	if  -yaw_precision<yaw_err<yaw_precision:
		change_state(3)
	else:
		if yaw_err>0:
			message.angular.z=-0.5
		elif yaw_err<0:
			message.angular.z=0.5

	pub.publish(message)


def find_yaw():
	global min_dist_wall,current_yaw,state,desired_yaw


	desired_yaw = current_yaw + (min_dist_wall[0]*math.pi)/180 - (math.pi/2)
	print(desired_yaw)
	change_state(2)

def stop():
	message = Twist()
	message.linear.x=0
	message.angular.z=0
	pub.publish(message)

def callback_odom(msg):
	global current_yaw

	quaternion = ( msg.pose.pose.orientation.x,
					msg.pose.pose.orientation.y,
					msg.pose.pose.orientation.z,
					msg.pose.pose.orientation.w )

	euler = transformations.euler_from_quaternion(quaternion)
	current_yaw = euler[2]


def callback_laser(msg):

	global state,laser_val,min_dist_wall,dist_precision
	laser_val = msg

	# if min_dist_wall[1] > min(laser_val.ranges[0:179]) + dist_precision : 
	# 	change_state(1)
	
	min_dist_wall = [0,min(laser_val.ranges[0:179])]

	for i in range(180):
		if laser_val.ranges[i]==min_dist_wall[1]:
			min_dist_wall[0]=i
			break

# service move forward:
def move_forward(x):
    global pub
    message = Twist()
    rospy.wait_for_service('Move')
    Move = rospy.ServiceProxy('/Move',move)
    resp1 = Move(x)
    message.linear.x = resp1.linear_x


    pub.publish(message)
    print('Message published',message.linear.x)

def main():
	global pub,min_dist_wall
	rospy.init_node('follow_wall')
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	sub_odom = rospy.Subscriber('/odom',Odometry,callback_odom)
	sub_laser = rospy.Subscriber('/m2wr/laser/scan',LaserScan,callback_laser)

	rate = rospy.Rate(1)
	rate.sleep()

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		print(min_dist_wall)
		print("state = ",state)
		# state is zero when nearby wall not known
		if state==0:
			find_wall()
		# find the desired yaw
		elif state==1:
			find_yaw()
		# turn to the desired yaw
		elif state==2:
			turn()
		elif state==3:
			# stop()
			if min_dist_wall[1]>1:
				print('min greater than 1')
				move_forward(True)
		# 		change_state(4)
			else:
				move_forward(False)
			
		# 		# move_forward(False)
		# elif state==4:
		# 	if min_dist_wall[1]>1:
		# 		print('Yo')
		# 	else:
		# 		change_state(3)
				

		rate.sleep()	



if __name__ == '__main__':
	main()

