#!/usr/bin/python

import sys
import time
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
	
	rospy.init_node('amcl_nomotion_updater')

	update_frequency = rospy.get_param('~update_frequency', 2.0)
	rospy.loginfo("amcl not motion updater started with update frequency: %f" % update_frequency)

	# wait for amcl to work with two messages
	rospy.loginfo("wait for amcl")
	rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
	rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
	rospy.loginfo("amcl is started!")

	rate = rospy.Rate(update_frequency)
		
	rospy.wait_for_service('request_nomotion_update')
	
	try:
		nomotion_update = rospy.ServiceProxy('request_nomotion_update', Empty, persistent=True)
	except rospy.ServiceException, e:
		pass

	rospy.loginfo("starting loop for nomotion updates")
	while not rospy.is_shutdown():
		try:
			nomotion_update()
		except rospy.ServiceException:
			pass
		rate.sleep()

	
