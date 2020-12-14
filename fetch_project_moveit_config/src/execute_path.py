#!/usr/bin/env python

import sys
import actionlib
import subprocess
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np

from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_msgs.msg import String, Int16
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from fetch_project_moveit_config.srv import Velocities, VelocitiesResponse, CircularPath, CircularPathResponse
from tf.transformations import quaternion_from_euler

class ExecutePath(object):

  def __init__(self):
    super(ExecutePath, self).__init__()

    self.gui_input_sub        = rospy.Subscriber('gui_input', String, self.interface_callback)
    self.waypoints_sub        = rospy.Subscriber('waypoints', PoseArray, self.waypoint_callback)
    self.waypoints_missed_sub = rospy.Subscriber('waypoints_missed',PoseArray,self.missed_callback)

    # First initialize `moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a `RobotCommander`_ object. This object is the outer-level
    # interface to the robot
    self.robot = moveit_commander.RobotCommander()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to one group of joints.
    self.group = moveit_commander.MoveGroupCommander("arm_with_torso")
    self.group.set_end_effector_link("gripper_link")

    # We create a `DisplayTrajectory`_ publisher which is used later to publish
    # trajectories for RViz to visualize:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # Initialize waypoints
    self.waypoints = None

    # Initialize missed waypoints
    self.waypoints_missed = None

    # Intialize the plan
    self.plan = None

    # Getting Basic Information
    self.planning_frame = self.group.get_planning_frame()

    # Set path_to_goal to the FollowTrajectoryClient Class
    self.path_to_goal=FollowTrajectoryClient()
    #Initialise ros service to change velocity of arm
    self.service = rospy.Service('service_vel', Velocities, self.callback_srv)
    rospy.loginfo('Made  contact with  velocity server')
    self.traj_vel = 0.8
    #Initialise ros service to run optimized path on missed waypoints 
    self.service_cpath = rospy.Service('service_circle', CircularPath, self.circle_callback_srv)
    rospy.loginfo('Made  contact with circular  path server')
    #the direction to rotate, when performing  circular motion
    self.num_of_dir = 5

  def waypoint_callback(self,msg):
      self.waypoints = []
      # Append poses to a list
      for i in range(len(msg.poses)):
          self.waypoints.append(msg.poses[i])

  def missed_callback(self,msg):
       self.waypoints_missed = []
       # Append missed poses to a list
       for i in range(len(msg.poses)):
           self.waypoints_missed.append(msg.poses[i])

  def interface_callback(self,gui_input):

      if gui_input.data == "0":
          self.plan = self.plan_cartesian_path(self.waypoints)

      elif gui_input.data == "1":
          self.execute_plan(self.plan)

      elif gui_input.data == "2":
          self.path_to_goal.init_pose(self.traj_vel)

      elif gui_input.data == "3":
          self.path_to_goal.tuck_pose(self.traj_vel)

  def plan_cartesian_path(self, waypoints):
    ## Cartesian Paths
    (plan, fraction) = self.group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,              # eef_step
                                       0.00)             # jump_threshold

    plan = self.group.retime_trajectory(self.robot.get_current_state(),plan,self.traj_vel)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been computed")
    return plan

  def execute_plan(self, plan):
 #     print ("WAYPOINTS ARE:", len(self.waypoints), self.waypoints)
      self.group.execute(plan, wait=True)

  def callback_srv(self, request):
      self.traj_vel = request.velocity
      rospy.loginfo('Changed  arm Velocity  to:{0}'.format(self.traj_vel))
      return VelocitiesResponse(True)

  def circle_callback_srv(self, request):
    #Execute optimized path with circular motion
      self.start_path = request.start_circle
      if (self.start_path):
          for i in range (0, len(self.waypoints_missed)):
              self.move_to_point(i)
              self.perform_circular_motion()
          return CircularPathResponse(True)
      else:
          return CircularPathResponse(False)

  def move_to_point(self, i):
    #execute missed waypoint
      waypoints = []
      waypoints.append(self.waypoints_missed[i])
      missed_plan = self.plan_cartesian_path(waypoints)
      self.execute_plan(missed_plan)

  def perform_circular_motion(self):
      #create and execute a circular path around the point
      self.curr_pose = self.group.get_current_pose().pose
      
      for i in range(0, self.num_of_dir):        
        circular_waypoints = self.compute_circular_waypoints(i)
        circular_plan = self.plan_circular_path(circular_waypoints)
        self.execute_circular_path(circular_plan)
      rospy.loginfo('Circular motion completed')

  def compute_circular_waypoints(self, i):
    #We  want to rotate the end effector  around the current point
    #generating waypoints for the circular motion
    circular_waypoints =  []    
    print ("Current  pose:", i)
    point = self.curr_pose
    point.orientation.w =  0.9961947
    turn_angle = 0.017452
    if (i==0):
      point.orientation.y =  turn_angle

    elif (i==1):
      point.orientation.x =  turn_angle

    elif (i==2):
      point.orientation.y =  -turn_angle

    elif (i==3):
      point.orientation.x =  -turn_angle

    elif (i==4):
      circular_waypoints.append(self.curr_pose)

    else:
      circular_waypoints = []

    circular_waypoints.append(point)

    return circular_waypoints

  def plan_circular_path(self, circular_waypoints):
    ## Cartesian Paths
    (plan, fraction) = self.group.compute_cartesian_path(
                                       circular_waypoints,   # waypoints to follow
                                       0.1,              # eef_step
                                       0.00)             # jump_threshold

    circular_plan = self.group.retime_trajectory(self.robot.get_current_state(),plan,self.traj_vel)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been computed")
    return circular_plan

  def execute_circular_path(self, circular_plan):
 #     print ("WAYPOINTS ARE:", len(self.waypoints), self.waypoints)
    self.group.execute(circular_plan, wait=True)

class FollowTrajectoryClient(object):

    def __init__(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")
        self.scene = PlanningSceneInterface("base_link")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

    def tuck_pose(self, vel):

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     tolerance = 0.02,
                                                     max_velocity_scaling_factor=vel)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.scene.removeCollisionObject("keepout")
                rospy.loginfo("done")
                return

    def init_pose(self, vel):

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")

        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [.05, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     tolerance = 0.02,
                                                     max_velocity_scaling_factor=vel)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                rospy.loginfo("done")
                return


if __name__=="__main__":
    rospy.init_node('execute_path',anonymous=True)
    ExecutePath()
    rospy.spin()
