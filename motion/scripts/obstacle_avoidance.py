#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub=None

def callback_laser(msg):

	region = {
				"right":min(min(msg.ranges[0:35]),10),
				"fright":min(min(msg.ranges[36:71]),10),
				"front":min(min(msg.ranges[72:107]),10),
				"fleft":min(min(msg.ranges[108:143]),10),
				"left":min(min(msg.ranges[143:179]),10)
			 }

	path(region)

def path(region):
	msg1=Twist()
	angular_z = 0
	linear_x = 0
	state=''

	if region["front"]<1 and region["fright"]<1 and region["fleft"]<1:
		angular_z = 0.5
		linear_x = 0
		state='U-turn'
	elif region["front"]<1 and region["fright"]>1 and region["fleft"]>1:
		angular_z = -0.5
		linear_x = 0
		state='turn left'
	elif region["front"]<1 and region["fright"]>1 and region["fleft"]<1:
		angular_z = 0.5
		linear_x = 0
		state='turn right'
	elif region["front"]<1 and region["fright"]<1 and region["fleft"]>1:
		angular_z = -0.5
		linear_x = 0
		state='turn left'
	elif region["front"]>1:
		angular_z = 0
		linear_x = 0.5
		state='go forward'
	else:
		state = 'unknown case'
	
	rospy.loginfo(state)
	msg1.linear.x = linear_x
	msg1.angular.z = angular_z
	pub.publish(msg1)



def main():
	global pub
	rospy.init_node('obstacle_avoidance')
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	sub = rospy.Subscriber('/m2wr/laser/scan',LaserScan,callback_laser)
	rospy.spin()

if __name__ == '__main__':
	main()