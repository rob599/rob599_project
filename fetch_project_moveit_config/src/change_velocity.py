#!/usr/bin/env python2

# An example of changing  the  velocity of the arm using a  service  call.
#
# Anjali Asar
#

# Import what we need from ROS and Python
import rospy
import sys
from fetch_project_moveit_config.srv import Velocities, VelocitiesResponse

class Mover:
    def __init__(self):
        self.service  =  rospy.Service('service_vel', Velocities, self.callback_srv) 
        rospy.loginfo('{0}:Made contact with velocity server'.format(self.__class__.__name__))

    def callback_srv(self, request):
        data = request.velocity
        print ("changing velocity to:", data)
        return VelocitiesResponse(True)

if __name__ == '__main__':
	# Initialize the node, as usual.
        rospy.init_node('change_velocity', argv=sys.argv)
        move = Mover() 
        rospy.spin()
