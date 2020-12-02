#!/usr/bin/env python3

# Import what we need
import rospy
import math
import actionlib
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point, PolygonStamped, PoseStamped, Point32
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from boustrophedon_msgs.msg import PlanMowingPathAction, PlanMowingPathGoal, PlanMowingPathResult

class Waypoint_generator:
    def __init__(self):
        # Set up a client for the basic nav, receives the user_defined Action
        self.coverage_client = actionlib.SimpleActionClient('plan_path', PlanMowingPathAction)
        self.coverage_client.wait_for_server()
        rospy.loginfo('Made contact with move server')

        # Initialize Subscribers
        self.M_inv_array_sub = rospy.Subscriber('data_array',numpy_msg(Floats), self.data_callback, queue_size=1)

        # Initialize Publishers
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker, queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup PolygonStamped for the 2D subplane
        self.sub_plane_polygon = PolygonStamped()
        self.sub_plane_polygon.header = self.header

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

        self.M_inv = None

        self.X = None
        self.Y = None

        self.resolution = .05

        self.offset = .3


    def data_callback(self, data_msg):
        M_inv_arr = data_msg.data[:16]
        coord_2D_arr = data_msg.data[16:]
        coord_2D_arr_len = int(len(coord_2D_arr)/2)

        # Set Inverse transfer Matrix (M_inv) to equal an empty matrix.
        self.M_inv = np.empty(shape=[4,4])
        self.X = []
        self.Y = []

        for i in range(4):
            self.M_inv[i][0] = M_inv_arr[i*4 + 0]
            self.M_inv[i][1] = M_inv_arr[i*4 + 1]
            self.M_inv[i][2] = M_inv_arr[i*4 + 2]
            self.M_inv[i][3] = M_inv_arr[i*4 + 3]

        for i in range(coord_2D_arr_len):
            self.sub_plane_polygon.polygon.points.append(Point32(coord_2D_arr[2*i + 0], coord_2D_arr[2*i + 1], 0))

        self.assign_goal()



    def assign_goal(self):
        poly = self.sub_plane_polygon
        start_pose = PoseStamped()
        start_pose.header = self.header
        start_pose.pose.position.x    = self.sub_plane_polygon.polygon.points[0].x
        start_pose.pose.position.y    = self.sub_plane_polygon.polygon.points[0].y
        start_pose.pose.position.z    = self.sub_plane_polygon.polygon.points[0].z
        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 1.0

        goal = PlanMowingPathGoal(property=poly,robot_position=start_pose)
        self.coverage_client.send_goal(goal, done_cb=self.callback_done)
        # dimension_increase

    def callback_done(self, status, result):

        poses = []
        marker_list = []
        # print("")
        # print(result.plan.points[0].point.x)
        for i in range(len(result.plan.points)):
            arr_2D = np.array([result.plan.points[i].point.x, result.plan.points[i].point.y, 0, 1])
            dim_incr = np.matmul(self.M_inv, arr_2D)

            p = Pose()
            p.position.x = dim_incr[0]
            p.position.y = dim_incr[1]
            p.position.z = dim_incr[2] + self.offset
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            poses.append(p)

            marker_list.append(Point(p.position.x, p.position.y, p.position.z))

        self.waypoints_marker.points = marker_list
        self.waypoints_marker_pub.publish(self.waypoints_marker)





if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')
    Waypoint_generator()
    rospy.spin()
