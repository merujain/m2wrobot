#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback_laser(msg):
	region = [min(msg.ranges[0:239]),min(msg.ranges[240:479]),min(msg.ranges[480:719])]
	rospy.loginfo(region)

def main():
	rospy.init_node('read_laser')
	sub = rospy.Subscriber('/m2wr/laser/scan',LaserScan,callback_laser)
	rospy.spin()

if __name__ == '__main__':
	main()