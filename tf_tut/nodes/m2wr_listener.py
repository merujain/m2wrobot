#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_tut')
import rospy
import math
import tf
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    rospy.init_node('m2wr_tf_listener')

    listener = tf.TransformListener()


    # turtle_vel = rospy.Publisher('/robot1/cmd_vel',Twist,queue_size=1)

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('world','laser_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        quaternion = (rot[0],rot[1],rot[2],rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        print(trans)
        print((euler[2]*180)/math.pi)

        # angular = 4 * math.atan2(trans[1], trans[0])
        
        # linear = 0.5* (math.sqrt(trans[0]**2 + trans[1]**2))
        # cmd = Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)
        # print(linear,angular)
        # print(1)

        rate.sleep()