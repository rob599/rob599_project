#!/usr/bin/env python3

# Import what we need
import rospy
import numpy as np

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class Create_Interactive_Markers:
    def __init__(self):
        # Initialize Subscribers
        self.clicked_point_sub = rospy.Subscriber('clicked_point', PointStamped, self.makeInteractiveMarker)

        # Initialize Publishers
        # self.IM_list_pub = rospy.Publisher('IM_pose_list',List, queue_size=1)
        self.IM_array_pub = rospy.Publisher('IM_pose_array',numpy_msg(Floats), queue_size=1)

        # Setup header for Interactive Markers (IM)
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # create an IM server on the topic namespace interactive_marker
        self.server = InteractiveMarkerServer("interactive_marker")

        # Initialize id of IM's.
        self.id = 0

        # Intialize list of IM poses
        self.arr_msg = None

    # processFeedback gets called when an IM's position is changed
    def processFeedback(self,feedback):
        # Update the array of IM postions from processFeedback
        p = feedback.pose.position
        index = int(feedback.marker_name)
        self.arr_msg[index*3 + 0] = p.x
        self.arr_msg[index*3 + 1] = p.y
        self.arr_msg[index*3 + 2] = p.z

        # Publish updated array messages.
        self.IM_array_pub.publish(self.arr_msg)


    def makeInteractiveMarker(self, msg):
        # store x,y, and z point values from subscribed clicked points.
        if self.id == 0:
            self.arr_msg = np.array([msg.point.x, msg.point.y, msg.point.z],dtype=np.float32)
        else:
            # append new clicked points to the array, arr_msg.
            append_arr = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)
            self.arr_msg = np.append(self.arr_msg, append_arr)

        # create an IM for our server
        int_marker = InteractiveMarker()
        int_marker.header = self.header
        int_marker.scale = 0.2
        int_marker.name = str(self.id)
        int_marker.pose.position = msg.point

        # create a white sphere marker
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = int_marker.scale*0.5
        sphere_marker.scale.y = int_marker.scale*0.5
        sphere_marker.scale.z = int_marker.scale*0.5
        sphere_marker.color.r = 1
        sphere_marker.color.g = 1
        sphere_marker.color.b = 1
        sphere_marker.color.a = 1

        # create an interactive control which contains the sphere and allows the
        # marker to move in any direction when you click and drag (MOVE_3D)
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.markers.append(sphere_marker)

        # add the control to the IM
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the x direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 1
        sphere_control.orientation.y = 0
        sphere_control.orientation.z = 0
        sphere_control.name = "move_x"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the z direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 0
        sphere_control.orientation.y = 1
        sphere_control.orientation.z = 0
        sphere_control.name = "move_z"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the y direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 0
        sphere_control.orientation.y = 0
        sphere_control.orientation.z = 1
        sphere_control.name = "move_y"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # Add the IM to our collection & tell the server to call processFeedback()
        # when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

        # Update the IM id for next clicked point.
        self.id += 1

        # Publish the array
        self.IM_array_pub.publish(self.arr_msg)



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('create_interactive_markers')
    Create_Interactive_Markers()
    rospy.spin()
