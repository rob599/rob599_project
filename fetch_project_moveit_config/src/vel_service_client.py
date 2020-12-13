#!/usr/bin/env python3

# A service client to change the velocity of the robot  arm.
#
# Anjali Asar


import rospy
import sys

# Import the base service message type.  Note that, as with the server, we need the .srv extension on
# the package name (asuming we named the service message directory "srv", which is conventional).
from fetch_project_moveit_config.srv import Velocities


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('service_client', argv=sys.argv)

	# This will wait for the service to become available.  It's a good idea to put this in here, since some
	# of the code below will fail if the service does not exist.
	rospy.wait_for_service('service_vel')

	# Assuming that the service is up, setting up a service proxy gives us access to it.  We'll need a name
	# and a base service message type.
	velocity = rospy.ServiceProxy('service_vel', Velocities)

	# To illustrate the use of a service, we're going to count up from zero, sending the numbers to the
	# service and logging the responses.
	vel_new =  float(input("Enter new velocity"))
		# Once we have the service proxy set up, we can use it just like a function.  The arguments to this
		# function are copied into the fields of a DoublerRequest instance, and then sent over.  The functor
		# returns an instance of DoublerResponse, from which you can access as usual.  It's good practice to
		# wrap this in a try-except block in case something goes wrong.
	try:
	    answer = velocity(vel_new)
	    rospy.logwarn('Service call succeeded as {0}'.format(answer.vel_set))

	except rospy.ServiceException as e:
	    rospy.logwarn('Service call failed as {0}'.format(e))

