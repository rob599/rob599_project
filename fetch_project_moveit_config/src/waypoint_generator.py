#!/usr/bin/env python3

# Import what we need
import rospy
import math
import random
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from grid_based_sweep_coverage_path_planner import planning_animation,planning

class Waypoint_generator:
    def __init__(self):
        # Initialize Subscribers
        self.M_inv_array_sub = rospy.Subscriber('data_array',numpy_msg(Floats), self.data_callback, queue_size=1)

        # Initialize Publishers
        self.waypoints_pub        = rospy.Publisher('waypoints'       , PoseArray        , queue_size=1)
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker           , queue_size=1)
        self.waypoints_missed     = rospy.Publisher('waypoints_missed', numpy_msg(Floats), queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        self.M_inv = None

        self.X = None
        self.Y = None

        # self.coodinates_2D = None
        self.resolution = .05

        self.waypoints = PoseArray()
        self.waypoints.header = self.header

        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.LINE_STRIP
        self.waypoints_marker.action = Marker.ADD
        self.waypoints_marker.id = 1
        self.waypoints_marker.scale.x = .01
        self.waypoints_marker.scale.y = .01
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 0
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 1.0



    def data_callback(self, data_msg):
        M_inv_arr = data_msg.data[:16]
        coord_2D_arr = data_msg.data[16:]
        coord_arr_len = len(coord_2D_arr)

        # Set Inverse transfer Matrix (M_inv) to equal an empty matrix.
        self.M_inv = np.empty(shape=[4,4])
        # self.coordinates_2D = np.empty(shape=[coord_arr_len,2])
        self.X = []
        self.Y = []

        for i in range(4):
            self.M_inv[i][0] = M_inv_arr[i*4 + 0]
            self.M_inv[i][1] = M_inv_arr[i*4 + 1]
            self.M_inv[i][2] = M_inv_arr[i*4 + 2]
            self.M_inv[i][3] = M_inv_arr[i*4 + 3]

        for i in range(coord_arr_len):
            if (i % 2) == 0:
                self.X.append(coord_2D_arr[i])
            else:
                self.Y.append(coord_2D_arr[i])

            if (i+1) == coord_arr_len:
                self.X.append(coord_2D_arr[0])
                self.Y.append(coord_2D_arr[1])
        # print(self.X, self.Y)
        self.waypoint_planner()



    def waypoint_planner(self):
        px,py = planning(self.X, self.Y, self.resolution)

        # dimension_increase
        poses = []
        marker_list = []
        pose_arr = np.empty(shape=[len(px),2])

        for i in range(len(px)):
            arr_2D = np.array([px[i], py[i], 0, 1])
            dim_incr = np.matmul(self.M_inv, arr_2D)
            pose_arr[i] = [px[i], py[i]]

            p = Pose()
            p.position.x = dim_incr[0]
            p.position.y = dim_incr[1]
            p.position.z = dim_incr[2]
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            poses.append(p)

            marker_list.append(Point(p.position.x, p.position.y, p.position.z))

        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Publish markers for waypoints
        self.waypoints_marker.points = marker_list
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        missed_points = np.empty(shape=[3,2])

        for i in range(3):
            e = random.randint(0,len(pose_arr)-1)
            missed_points[i] = pose_arr[e]


        # Publish the "missed" points for optimization node
        a = np.array(self.M_inv.ravel(), dtype=np.float32)
        b = np.array(missed_points.ravel(), dtype=np.float32)
        self.waypoints_missed.publish(np.array(pose_arr.ravel(), dtype=np.float32))

        # self.waypoints_missed.publish(np.array(missed_points.ravel(), dtype=np.float32))



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')
    Waypoint_generator()
    rospy.spin()
