#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Point32


if __name__ == '__main__':

    rospy.init_node('pointcloud')
    cloud_pub = rospy.Publisher(
        'pointcloud/laser',
        PointCloud,
        queue_size=50
    )

    number_of_pixels = 100
    count=0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        debug_pointcloud = PointCloud()
        debug_pointcloud.header = std_msgs.msg.Header()
        debug_pointcloud.header.stamp = rospy.Time.now()
        debug_pointcloud.header.frame_id = "camera_optical"

        
        # create an empty list of correct size
        debug_pointcloud.points = [None] * number_of_pixels
        
        # debug_pointcloud.channels.resize(1)
        # debug_pointcloud.channels[0].name = "intensities";
        # debug_pointcloud.channels[0].values.resize(num_points);
        
        # fill up pointcloud with points where x value changes but y and z are all 0
        for p in xrange(0, number_of_pixels):
            debug_pointcloud.points[p] = Point32(1+count, 2+count, 3+count)
            # debug_pointcloud.channels[0].values[p] = 100 + count;

        # now publish the debug pointcloud
        cloud_pub.publish(debug_pointcloud)
        count=count+1
        rate.sleep()

    rospy.spin()