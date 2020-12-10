#!/usr/bin/env python

import sys
import actionlib
import subprocess
import rospy
import moveit_commander
import moveit_msgs.msg

from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_msgs.msg import String, Int16
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray

class MoveGroupInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupInteface, self).__init__()
    self.gui_input_sub = rospy.Subscriber('gui_input', String, self.callback)
    self.waypoints_sub = rospy.Subscriber('waypoints', PoseArray, self.plan_cartesian_path)
    ## First initialize `moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()
    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Fetch
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group = moveit_commander.MoveGroupCommander("arm_with_torso")
    group.set_end_effector_link("gripper_link")
    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    # We can also print the name of the end-effector link for this group:
    # eef_link = group.get_end_effector_link()
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    # Misc variables
    self.box_name = ''
    self.robot = robot
    # self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = "gripper_link"
    # print(eef_link)
    self.group_names = group_names
    self.plan = None
    self.path_to_goal=FollowTrajectoryClient()


  def callback(self,gui_input):

      if gui_input.data == "1":
          self.execute_plan()

      elif gui_input.data == "2":
          self.path_to_goal.init_pose()

      elif gui_input.data == "3":
          self.path_to_goal.tuck_pose()

  def plan_cartesian_path(self,msg):
    group = self.group
    robot = self.robot
    # scene = self.scene
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:

    # scene = PlanningSceneInterface("base_link")
    # scene.removeCollisionObject("table")
    # scene.addBox("table", 0.73, 1.52, 0.80, .6, 0.0, 0.80/2)
    waypoints = []
    for i in range(len(msg.poses)):
        waypoints.append(msg.poses[i])
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.00)         # jump_threshold

    self.plan = group.retime_trajectory(robot.get_current_state(),plan,.2)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print("Path has been computed")
    # return plan, fraction

  def execute_plan(self):
    group = self.group
    # scene = self.scene
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    # print "======== Press 'Enter' to start the lawn mower path. "
    # raw_input()
    group.execute(self.plan, wait=True)


class FollowTrajectoryClient(object):

    def __init__(self):
        self.client = None

    def tuck_pose(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)
        # scene.addBox("table", 0.73, 1.52, 0.80, .6, 0.0, 0.80/2)


        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     tolerance = 0.02,
                                                     max_velocity_scaling_factor=0.2)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                # scene.removeCollisionObject("table")
                rospy.loginfo("done")
                return

    def init_pose(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")
        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")

        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)
        # scene.addBox("table", 0.73, 1.52, 0.80, .6, 0.0, 0.80/2)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [.05, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
        # pose = [.05, 1.50, -0.90, 3.06, -2.10, -1.57, -1.40, -0.37,]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     tolerance = 0.02,
                                                     max_velocity_scaling_factor=0.2)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                # scene.removeCollisionObject("table")
                rospy.loginfo("done")
                return


if __name__=="__main__":
    rospy.init_node('movegroupinterface',anonymous=True)
    MoveGroupInteface()
    # path_to_goal=FollowTrajectoryClient()
    # path_to_goal.init_pose()
    rospy.spin()
