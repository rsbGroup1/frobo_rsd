import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from msgs.msg import BoolStamped

def createBoolStampedMessage( data ):
    msg = BoolStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.data = data
    return msg

class obs_detector():
	def __init__(self):
		# initialize stuff
		rospy.init_node('obstacle_detector')

		publishTopic = rospy.get_param("~publishTopic", "/mrObstacleDetector/status")
		self.obstaclePub = rospy.Publisher(publishTopic, String, queue_size=1)
		self.slow = False
		self.stop = False
		self.update_rate = 20 # [Hz]
		self.r = rospy.Rate(self.update_rate)
		self.oldValue = -1 # 0 = nothing, 1 = slow, 2 = stop

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
		
		#if self.value != self.oldValue:
		if self.value == 0:
			self.obstaclePub.publish("normal")
		elif self.value == 1:
			self.obstaclePub.publish("slow")
		elif self.value == 2:
			self.obstaclePub.publish("stop")
		self.oldValue = self.value

		# publish deadman topic
		#if self.value == 2:
		#        msg = createBoolStampedMessage( False )
		#if (self.stop):
		msg = createBoolStampedMessage( not self.stop )
		pubActuationEna.publish ( msg )
		#else:
		        #msg = createBoolStampedMessage( True )
        		#pubActuationEna.publish ( msg )	
			

	def updater(self):
		while not rospy.is_shutdown():
			self.r.sleep()


if __name__ == '__main__':
    try:
        node_class = obs_detector()
    except rospy.ROSInterruptException: pass

