#!/usr/bin/env python3

# Import what we need
import six
import sys
sys.modules['sklearn.externals.six'] = six
import mlrose
import numpy as np
import rospy
import random

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class Optimizer:
    def __init__(self):
        # Initialize Subscribers
        self.waypoints_missed_sub = rospy.Subscriber('missed_data',numpy_msg(Floats), self.data_callback, queue_size=1)

        # Initialize Publishers
        self.waypoints_missed_pub = rospy.Publisher('waypoints_missed', PoseArray, queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Intialize inverse Matrix to None
        self.M_inv = None

        # Intialize the coordinates list of missed points
        self.coords_list = []

        # Intialize waypoints_missed as a PoseArray
        self.waypoints_missed = PoseArray()
        self.waypoints_missed.header = self.header


    def data_callback(self, data_msg):
        # Splice the data: Inverse matrix and the 2D coordinates of the missed points
        M_inv_arr  = data_msg.data[:16]
        missed_arr = data_msg.data[16:]
        len_missed = int(len(missed_arr)/2)

        self.coords_list = []

        # Set Inverse transfer Matrix (M_inv) to equal an empty matrix.
        self.M_inv = np.empty(shape=[4,4])

        # Fill in the the inverse matrix with the subscribed data
        for i in range(4):
            self.M_inv[i][0] = M_inv_arr[i*4 + 0]
            self.M_inv[i][1] = M_inv_arr[i*4 + 1]
            self.M_inv[i][2] = M_inv_arr[i*4 + 2]
            self.M_inv[i][3] = M_inv_arr[i*4 + 3]

        # Store the x and y positions of the missed points to self.coords_list
        for i in range(len_missed):
            self.coords_list.append((missed_arr[i*2 + 0], missed_arr[i*2 + 1]))

        # Run optimize_path function
        self.optimize_path()

    def optimize_path(self):
        # Initialize fitness function object using self.coords_list
        fitness_coords = mlrose.TravellingSales(coords = self.coords_list)

        problem_fit = mlrose.TSPOpt(length = 3,
                                    fitness_fn = fitness_coords,
                                    maximize=False)

        # Solve problem using the genetic algorithm
        best_state, best_fitness = mlrose.genetic_alg(problem_fit, random_state = 2)

        print('The best state found is: ', best_state)
        print('The fitness at the best state is: ', best_fitness)


        # Begin dimension increase for 2D coordinates (px, py)
        poses = []
        for e in best_state:
            arr_2D = np.array([self.coords_list[e][0], self.coords_list[e][1], 0, 1])
            dim_incr = np.matmul(self.M_inv, arr_2D)

            # pose appended to poses (Pose Array)
            p = Pose()
            p.position.x = dim_incr[0]
            p.position.y = dim_incr[1]
            p.position.z = dim_incr[2]
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            poses.append(p)

        # Publish PoseArray of missed poses
        self.waypoints_missed.poses = poses
        self.waypoints_missed_pub.publish(self.waypoints_missed)



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('optimizer')
    Optimizer()
    rospy.spin()
