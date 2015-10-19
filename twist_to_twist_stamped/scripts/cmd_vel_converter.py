#!/usr/bin/env python
#/****************************************************************************
# FroboMind cmd_vel_converter.py
# Copyright (c) 2011-2015, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
2015-10-15: Adapted from frobomind 
"""

import rospy
from geometry_msgs.msg import TwistStamped,Twist	
from std_msgs.msg import Bool
import std_msgs.msg

class CmdVelConverter():
	"""
		Converter for using FroboMind with stage. 
		Takes TwistStamped message from /fmSignals/cmd_vel and parses as Twist message on /cmd_vel
	"""
	def __init__(self):
		# Init node
		topic_tw_stamped = rospy.get_param("~cmd_vel_pub", "/fmCommand/cmd_vel")
		topic_tw = rospy.get_param("~cmd_vel_sub", "/cmd_vel")
		self.twist_sub = rospy.Subscriber(topic_tw, Twist, self.onTwist )
		self.twist_pub = rospy.Publisher(topic_tw_stamped, TwistStamped, queue_size=1)

		
	def onTwist(self,msg):
		newMsg = TwistStamped()
		head = std_msgs.msg.Header()
		newMsg.twist = msg
		head.stamp = rospy.Time.now()
		newMsg.header = head
		self.twist_pub.publish(newMsg)

if __name__ == '__main__':
	rospy.init_node('cmd_vel_to_stamped_converter')
	node = CmdVelConverter()
	rospy.spin()
	
