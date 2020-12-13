#!/usr/bin/env python
import rospy
import numpy as np
import tf


from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, Twist, Point32
from sensor_msgs.msg import PointCloud
from gazebo_msgs.msg import ModelStates

class WaypointTransform():

    def __init__(self):
        self.sub = rospy.Subscriber("waypoints", PoseArray, self.waypoint_callback, queue_size=1) 
        self.pub = rospy.Publisher("waypoints_transformed_cloud", PointCloud, queue_size=1) 

        self.listener = tf.TransformListener()
        self.frame = "/map"

    #transforms pose array into point cloud and transforms into map coordinates
    def waypoint_callback(self, msg):
        cloud = PointCloud()
        cloud.header = msg.header

        for i in range(len(msg.poses)):
            cloud.points.append(Point32(msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z))

        try:
            cloud = self.listener.transformPointCloud(self.frame, cloud)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        self.pub.publish(cloud)

if __name__=="__main__":
    rospy.init_node('transform_waypoint',anonymous=True)
    WaypointTransform();
    rospy.spin()


