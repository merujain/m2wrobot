#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_tut')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.15, 0.0, 0.05),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "laser_1",
                         "link_chassis_1")
        rate.sleep()