#!/usr/bin/env python

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

