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
        #subscribed to gazebo model states for the cube and the transformed point cloud
        self.sub = rospy.Subscriber("waypoints_transformed_cloud", PointCloud, self.waypoint_callback, queue_size=1) 
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size=10) 

        #gets pose of model
        self.model_pose = Pose() 
        self.get_new_pose = True

        #initializes SDF of cube 
        dir_path = os.path.dirname(os.path.realpath(__file__)) + "/obj_files/box_object.obj"
        self.sdf = sdfield_generator.SDF_Generator(dir_path, 32, .05)
        distance_array, sdf_origin, sdf_spacing, sdf_resolution = self.sdf.get_sdf_array()

    #gets points from published waypoints and checks them agains the sdf
    def waypoint_callback(self, msg):
        for i in range(len(msg.points)-1):
            #extrapolates points from waypoints to increase coverage and accuracy
            x = np.linspace(msg.points[i].x, msg.points[i+1].x, 10)
            y = np.linspace(msg.points[i].y, msg.points[i+1].y, 10)
            z = np.linspace(msg.points[i].z, msg.points[i+1].z, 10)
            for i in range(len(x)):
                #creates points from model and from points created above
                point = [x[i], y[i], z[i]]
                origin = [self.model_pose.position.x, self.model_pose.position.y, self.model_pose.position.z]

                #transforms global coordinates of waypoints into local coordinates of cube to then interpolate from the sdf
                point_matrix = matrix_helper.get_translation_matrix(point)
                model_matrix = matrix_helper.get_translation_matrix(origin)
                transformed_point = matrix_helper.global_to_local(point_matrix, model_matrix)
               
                #interpolates distance from the cube using trilinear interpolation
                distance_from_cube = self.sdf.interpolate_sdf_from_point(transformed_point)   

                #if we are seen to be in collision with the object we throw an error message and return
                if distance_from_cube < .04 and distance_from_cube != -1:
                    rospy.logerr("YOUR PATH IS CURRENTLY IN COLLISION WITH AN OBJECT, PLEASE ADJUST THE MARKERS")
                    return
                    
        #if no collisions 
        rospy.loginfo("CURRENT PATH IS NOT IN COLLISION")

    #gets model world position in gazebo from model states topic
    def model_callback(self, msg):
        for i in range(len(msg.name)):
            #block is static so we only need position once, if it was dynamic we could modify this
            if msg.name[i] == "block" and self.get_new_pose == True:
                self.model_pose = msg.pose[i]
                self.get_new_pose = False
                

#starts everything
if __name__=="__main__":
    rospy.init_node('collision_warning',anonymous=True)
    CollisionWarning();
    rospy.spin()


