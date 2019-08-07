#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_tut')
import rospy
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist

def callback_odom(msg,turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),turtlename,
                     "map")

if __name__ == '__main__':
    rospy.init_node('m2wr_tf_broadcaster')
    turtlename = rospy.get_param('~m2wr')
    sub_odom = rospy.Subscriber(turtlename,Odometry,callback_odom,turtlename)
    rospy.spin()

