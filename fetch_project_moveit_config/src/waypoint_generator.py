#!/usr/bin/env python3

# Import what we need
import rospy
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
        self.missed_data          = rospy.Publisher('missed_data', numpy_msg(Floats), queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Intialize the inverse matrix
        self.M_inv = None

        # Intialize the X and Y coordinates of the 2D data points
        self.X = None
        self.Y = None

        # Set the waypoint resolution (distance between points)
        self.resolution = .05

        # Initialize waypoints as a PoseArray
        self.waypoints = PoseArray()
        self.waypoints.header = self.header

        # Initialize waypoint_markers and all of the other feature values
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
        # Splice the data: Inverse matrix and the 2D coordinates
        M_inv_arr = data_msg.data[:16]
        coord_2D_arr = data_msg.data[16:]
        coord_arr_len = len(coord_2D_arr)

        # Set Inverse transfer Matrix (M_inv) to equal an empty matrix.
        self.M_inv = np.empty(shape=[4,4])

        # assign X and Y to empty lists
        self.X = []
        self.Y = []

        # Fill in the the inverse matrix with the subscribed data
        for i in range(4):
            self.M_inv[i][0] = M_inv_arr[i*4 + 0]
            self.M_inv[i][1] = M_inv_arr[i*4 + 1]
            self.M_inv[i][2] = M_inv_arr[i*4 + 2]
            self.M_inv[i][3] = M_inv_arr[i*4 + 3]

        # Append the X and Y values of the coordinates positions
        for i in range(coord_arr_len):
            if (i % 2) == 0:
                self.X.append(coord_2D_arr[i])
            else:
                self.Y.append(coord_2D_arr[i])

            # The planner needs a closed loop of points, hence why the first set
            # of coordinates are appended at the end of the lists.
            if (i+1) == coord_arr_len:
                self.X.append(coord_2D_arr[0])
                self.Y.append(coord_2D_arr[1])

        # Run waypoint_planner function
        self.waypoint_planner()



    def waypoint_planner(self):
        # Acquire the planned x an y values from the planning function
        px,py = planning(self.X, self.Y, self.resolution)

        # Create lists and array of waypoints to publish
        poses = []
        marker_list = []
        pose_arr = np.empty(shape=[len(px),2])

        # Begin dimension increase for 2D coordinates (px, py)
        for i in range(len(px)):
            arr_2D = np.array([px[i], py[i], 0, 1])
            dim_incr = np.matmul(self.M_inv, arr_2D)
            pose_arr[i] = [px[i], py[i]]

            # poses append to poses (Pose Array)
            p = Pose()
            p.position.x = dim_incr[0]
            p.position.y = dim_incr[1]
            p.position.z = dim_incr[2]
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            poses.append(p)

            # Append position values for the marker
            marker_list.append(Point(p.position.x, p.position.y, p.position.z))

        # assisn poses to the PoseArray, self,waypoints.
        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Publish markers for waypoints
        self.waypoints_marker.points = marker_list
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        # Create an array for three missed points
        missed_points = np.empty(shape=[3,2])

        # For loop to randomly select points for the missed points array
        for i in range(3):
            e = random.randint(0,len(pose_arr)-1)
            missed_points[i] = pose_arr[e]


        # Publish the inverse matrix and missed points for optimization node
        a = np.array(self.M_inv.ravel(), dtype=np.float32)
        b = np.array(missed_points.ravel(), dtype=np.float32)
        self.missed_data.publish(np.concatenate((a,b)))


if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')
    Waypoint_generator()
    rospy.spin()
