#!/usr/bin/env python

import rospy
from msgs.msg import BoolStamped, IntStamped

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines

		# static parameters
		self.update_rate = 50 # set update frequency [Hz]

		# get parameters
		self.deadman_en = rospy.get_param("~deadman_enable", True)
		self.critfault_en = rospy.get_param("~critical_fault_enable", True) 
		self.obstacle_en = rospy.get_param("~obstacle_enable", True) 
		self.deadman_timeout = rospy.get_param("~deadman_timeout", 0.050) # [s]
		self.critfault_timeout = rospy.get_param("~critical_fault_timeout", 0.050) # [s]
		self.obstacle_timeout = rospy.get_param("~critical_fault_timeout", 0.050) # [s]

		# get topic names
		self.topic_deadman = rospy.get_param("~deadman_sub",'/fmSafe/deadman')
		self.topic_critical_fault = rospy.get_param("~critical_fault_sub",'/fmSafe/critical_fault')
		self.topic_obstacle = rospy.get_param("~obstacle_sub",'/fmSafe/obstacle')
		self.topic_act_en = rospy.get_param("~actuation_enable_pub",'/fmSafe/actuation_enable')

		# setup topic publishers
		self.act_en_pub = rospy.Publisher(self.topic_act_en, BoolStamped, queue_size=0)
		self.act_en_msg = BoolStamped()

		# setup topic subscribers
		self.deadman_state = False
		self.deadman_next_tout = 0.0
		rospy.Subscriber(self.topic_deadman, BoolStamped, self.on_deadman_msg)
		self.critfault_state = False
		self.critfault_next_tout = 0.0
		rospy.Subscriber(self.topic_critical_fault, IntStamped, self.on_critfault_msg)
		self.obstacle_state = False
		self.obstacle_next_tout = 0.0
		rospy.Subscriber(self.topic_obstacle, IntStamped, self.on_obstacle_msg)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_deadman_msg(self, msg):
		# save current state and determine next timeout
		self.deadman_state = msg.data
		self.deadman_next_tout = rospy.get_time() + self.deadman_timeout

	def on_critfault_msg(self, msg):
		# save current state and determine next timeout
		self.critfault_state = msg.data
		self.critfault_next_tout = rospy.get_time()  + self.critfault_timeout

	def on_obstacle_msg(self, msg):
		# save current state and determine next timeout
		self.obstacle_state = msg.data
		self.obstacle_next_tout = rospy.get_time()  + self.obstacle_timeout

	def updater(self):
		while not rospy.is_shutdown():
			# default is True
			prev_act_en = self.act_en_msg.data
			self.act_en_msg.data = True

			# obstacle if true or too old
			if self.obstacle_en == True:
				if self.obstacle_state != 0 or self.obstacle_next_tout < rospy.get_time():
					self.act_en_msg.data = False

			# cricital fault if true or too old
			if self.critfault_en == True:
				if self.critfault_state != 0 or self.critfault_next_tout < rospy.get_time():
					self.act_en_msg.data = False

			# deadman fault if false or too old
			if self.deadman_en == True:
				if self.deadman_state == False or self.deadman_next_tout < rospy.get_time():
					self.act_en_msg.data = False
					#print self.deadman_next_tout, rospy.get_time(), self.deadman_state

			# publish actuation_enable message
			self.act_en_msg.header.stamp = rospy.get_rostime()
			self.act_en_pub.publish (self.act_en_msg)

			if prev_act_en != self.act_en_msg.data:
				if self.act_en_msg.data == True:
					rospy.logwarn(rospy.get_name() + ": Enabling actuation")
				else:
					rospy.logwarn(rospy.get_name() + ": Disabling actuation")

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('basic_incident_handler')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

