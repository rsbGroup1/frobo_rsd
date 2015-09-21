#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class obs_detector():
	def __init__(self):
		# initialize stuff
		rospy.init_node('obstacle_detector')
		self.pub_slow = rospy.Publisher('slow', Bool, queue_size=10)
		self.pub_stop = rospy.Publisher('stop', Bool, queue_size=10)
		self.slow = False
		self.stop = False
		self.update_rate = 20 # [Hz]
		self.r = rospy.Rate(self.update_rate)

		# get parameters
		self.threshold_slow = rospy.get_param("~threshold_slow", 1.2)
		self.threshold_stop = rospy.get_param("~threshold_stop", 0.5)		
		self.threshold_ignore = rospy.get_param("~threshold_ignore", 0.2)				
		laser_scan_topic = rospy.get_param("~laser_scan", "/scan")

		#subscribe to laser scan topic
		rospy.Subscriber(laser_scan_topic, LaserScan, self.on_lidar_data)

		#continously run the updater to publish
		self.updater()

	def on_lidar_data(self,msg):
		# our laserscanner has 90 measurements - 270 degree total
		# going right to left
		self.slow = False
		self.stop = False
		for x in range(len(msg.ranges)):
			if msg.ranges[x] > self.threshold_ignore:
				if msg.ranges[x] < self.threshold_slow:
					self.slow = True
				if msg.ranges[x] < self.threshold_stop:
					self.stop = True



	def publish(self):
		#publish three topics		
		self.pub_slow.publish(self.slow)
		self.pub_stop.publish(self.stop)

	def updater(self):
		while not rospy.is_shutdown():
			self.publish()
			self.r.sleep()


if __name__ == '__main__':
    try:
        node_class = obs_detector()
    except rospy.ROSInterruptException: pass

