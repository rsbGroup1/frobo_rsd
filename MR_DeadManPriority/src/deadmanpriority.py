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
from msgs.msg import BoolStamped

def createBoolStampedMessage( data ):
    msg = BoolStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.data = data

    return msg

class deadman():
	def __init__(self):
		# initialize stuff
		rospy.init_node('deadmanpriority')

		# parameters
		self.hmi = False
		self.obstacleDetector = False	
		self.deadman = False
		self.update_rate = 20 # [Hz]
		self.r = rospy.Rate(self.update_rate)
		global pubActuationEna

		# get parameters
		hmiTopic = rospy.get_param( "~hmiTopic", "/fmSafe/hmi_deadman" )
		obstacleDetectorTopic = rospy.get_param( "~obstacleDetectorTopic", "/fmSafe/obstacle_detector_deadman" )
		publishTopic = rospy.get_param("~publishTopic", "/fmSafe/deadman")
		pubActuationEna = rospy.Publisher( publishTopic, BoolStamped, queue_size = 1 )

		#subscribe to laser scan topic
		rospy.Subscriber(hmiTopic, BoolStamped, self.on_hmi_data)
		rospy.Subscriber(obstacleDetectorTopic, BoolStamped, self.on_obs_data)

		#continously run the updater to publish
		self.updater()

	def on_hmi_data(self,msg):
		self.hmi = msg.data

	def on_obs_data(self,msg):
		self.obstacleDetector = msg.data			

	def updater(self):
		while not rospy.is_shutdown():
			self.deadman = self.hmi and self.obstacleDetector
			msg = createBoolStampedMessage( self.deadman )
        		pubActuationEna.publish ( msg )
			self.r.sleep()


if __name__ == '__main__':
    try:
        node_class = deadman()
    except rospy.ROSInterruptException: pass

