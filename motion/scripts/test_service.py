#! /usr/bin/env python

import rospy
from m2wr_description.srv import *
from geometry_msgs.msg import Twist

pub = None

def callback_move(req):

	print('Service Called')
	if req.val==True:
		print(1)
		linear_x = 0.5
		
	elif req.val==False:
		print(2)
		linear_x = 0
		
	else :
		print("unknown")

	return moveResponse(linear_x)


def main():
	global pub
	rospy.init_node('Move_Server')
	s = rospy.Service('Move',move,callback_move)
	rospy.spin()

if __name__ == '__main__':
	main()