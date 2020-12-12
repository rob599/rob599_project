#!/usr/bin/env python3

# Import what we need
import six
import sys
sys.modules['sklearn.externals.six'] = six
import mlrose
import numpy as np
import rospy
import math
import random

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from grid_based_sweep_coverage_path_planner import planning_animation,planning

class Waypoint_generator:
    def __init__(self):
        # Initialize Subscribers
        self.waypoints_missed_sub = rospy.Subscriber('waypoints_missed',numpy_msg(Floats), self.data_callback, queue_size=1)

        # Initialize Publishers
        self.waypoints_pub        = rospy.Publisher('waypoints'       , PoseArray, queue_size=1)
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker   , queue_size=1)

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


def data_callback(self, data_msg):
    M_inv_arr = data_msg.data[:16]
    missed_arr = data_msg.data[16:]
    len_missed = int(len(missed_arr)/2)
    print(len_missed)
    # Set Inverse transfer Matrix (M_inv) to equal an empty matrix.
    self.M_inv = np.empty(shape=[4,4])

    rand_list = []

    for i in range(len_missed):
        rand_list.append((missed_arr[i*2 + 0], missed_arr[i*2 + 1]))

    for i in range(4):
        self.M_inv[i][0] = M_inv_arr[i*4 + 0]
        self.M_inv[i][1] = M_inv_arr[i*4 + 1]
        self.M_inv[i][2] = M_inv_arr[i*4 + 2]
        self.M_inv[i][3] = M_inv_arr[i*4 + 3]

#     for e in rand_list:
#         if (i % 2) == 0:
#             self.X.append(coord_2D_arr[i])
#         else:
#             self.Y.append(coord_2D_arr[i])
#
#         if (i+1) == coord_arr_len:
#             self.X.append(coord_2D_arr[0])
#             self.Y.append(coord_2D_arr[1])
#     # print(self.X, self.Y)
#     self.waypoint_planner()
#
#
#
# def waypoint_planner(self):
#     #Create list of city coordinates
#     coords_list = [(1, 1), (4, 2), (5, 2), (6, 4), (4, 4), (3, 6), (1, 5), (2, 3)]
#
#     # Initialize fitness function object using coords_list
#     fitness_coords = mlrose.TravellingSales(coords = coords_list)
#
#     problem_fit = mlrose.TSPOpt(length = 8,
#                                 fitness_fn = fitness_coords,
#                                 maximize=False)
#
#     # Solve problem using the genetic algorithm
#     best_state, best_fitness = mlrose.genetic_alg(problem_fit, random_state = 2)
#
#     print('The best state found is: ', best_state)
#
#     print('The fitness at the best state is: ', best_fitness)
