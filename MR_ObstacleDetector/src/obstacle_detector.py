#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from msgs.msg import BoolStamped
from mr_obstacle_detector.srv import enabler

safe = 1
proximityAlert = 2
colliding = 3

def createBoolStampedMessage( data ):
    msg = BoolStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.data = data
    return msg

class obs_detector():
	def __init__(self):
		# get parameters
		self.threshold_slow = rospy.get_param("~threshold_proximityAlert", 0.4)
		self.threshold_stop = rospy.get_param("~threshold_colliding", 0.3)		
		self.threshold_ignore = rospy.get_param("~threshold_ignore", 0.05)				
		laserScanSubName = rospy.get_param("~laser_scan", "/scan")
		obstaclePubName = rospy.get_param("~publishTopic", "/mrObstacleDetector/status")
		obstacleEnablerSrvName = rospy.get_param("~enabler", "/mrObstacleDetector/enabler")
		hmiTopicName = rospy.get_param("~hmiTopic", "/mrHMI/status")

		self.obstaclePub = rospy.Publisher(obstaclePubName, String, queue_size=1)
		self.hmiPub = rospy.Publisher(hmiTopicName, String, queue_size=10)
		self.laserScanSub = rospy.Subscriber(laserScanSubName, LaserScan, self.on_lidar_data)
		self.obstacleEnablerSrv = rospy.Service(obstacleEnablerSrvName, enabler, self.enable)
		
		self.slow = False
		self.stop = False
		self.update_rate = 30 # [Hz]
		self.enable = False
		self.oldValue = -1 # 0 = nothing, 1 = slow, 2 = sto

		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def updateHMI(self, state):
	        self.hmiPub.publish("000" + str(state) + ",,")

	def enable(self, req):
		print "Received", req.enable
		if req.enable == True:
			self.enable = True
		else:
			self.enable = False
		return self.enable

	def on_lidar_data(self,msg):
		if (self.enable):
			# our laserscanner has 270 measurements - 270 degree total
			# going right to left
			self.slow = False
			self.stop = False
			for x in range(len(msg.ranges)):
				if msg.ranges[x] > self.threshold_ignore:
					if ((x > len(msg.ranges)/3) and (x < len(msg.ranges)*2/3) and (msg.ranges[x] < self.threshold_stop)):
						self.stop = True
						break
					elif msg.ranges[x] < self.threshold_slow:
						self.slow = True

			self.value = 0
			if self.stop:
				self.value = 2
			elif self.slow:
				self.value = 1
			
			if self.value != self.oldValue:
				if self.value == 0:
					self.updateHMI(safe)
				elif self.value == 1:
					self.updateHMI(proximityAlert)
				elif self.value == 2:
					self.updateHMI(colliding)

			if self.value == 0:
				self.obstaclePub.publish("safe")
			elif self.value == 1:
				self.obstaclePub.publish("proximityAlert")
			elif self.value == 2:
				self.obstaclePub.publish("colliding")

			self.oldValue = self.value

		else:
			self.obstaclePub.publish("safe")

	def updater(self):
		while not rospy.is_shutdown():
			self.r.sleep()


if __name__ == '__main__':
    try:
    	rospy.init_node('mr_obstacle_detector')
        node_class = obs_detector()
    except rospy.ROSInterruptException: pass

