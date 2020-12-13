#!/usr/bin/env python3
import rospy
import numpy as np
import matrix_helper
import sdfield_generator
import os
import sys


from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, Twist, Point32
from sensor_msgs.msg import PointCloud
from gazebo_msgs.msg import ModelStates


class CollisionWarning():

    def __init__(self):
        self.sub = rospy.Subscriber("waypoints_transformed_cloud", PointCloud, self.waypoint_callback, queue_size=1) 
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size=10) 

        self.model_pose = Pose() 
        self.get_new_pose = True

        dir_path = os.path.dirname(os.path.realpath(__file__)) + "/obj_files/box_object.obj"

        self.sdf = sdfield_generator.SDF_Generator(dir_path, 32, .05)
        distance_array, sdf_origin, sdf_spacing, sdf_resolution = self.sdf.get_sdf_array()

    #gets points from published waypoints and checks them agains the sdf
    def waypoint_callback(self, msg):
        for i in range(len(msg.points)-1):
            x = np.linspace(msg.points[i].x, msg.points[i+1].x, 10)
            y = np.linspace(msg.points[i].y, msg.points[i+1].y, 10)
            z = np.linspace(msg.points[i].z, msg.points[i+1].z, 10)
            for i in range(len(x)):
                point = [x[i], y[i], z[i]]
                origin = [self.model_pose.position.x, self.model_pose.position.y, self.model_pose.position.z]

                point_matrix = matrix_helper.get_translation_matrix(point)
                model_matrix = matrix_helper.get_translation_matrix(origin)
                transformed_point = matrix_helper.global_to_local(point_matrix, model_matrix)
               
                #rospy.loginfo(transformed_point)

                distance_from_cube = self.sdf.interpolate_sdf_from_point(transformed_point)   
                #rospy.loginfo(distance_from_cube)
                if distance_from_cube < .04 and distance_from_cube != -1:
                    rospy.logerr("YOUR PATH IS CURRENTLY IN COLLISION WITH AN OBJECT, PLEASE ADJUST THE MARKERS")
                    return
            
        rospy.loginfo("CURRENT PATH IS NOT IN COLLISION")

    #gets model world position in gazebo
    def model_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == "block" and self.get_new_pose == True:
                self.model_pose = msg.pose[i]
                self.get_new_pose = False
                


if __name__=="__main__":
    rospy.init_node('collision_warning',anonymous=True)
    CollisionWarning();
    rospy.spin()


