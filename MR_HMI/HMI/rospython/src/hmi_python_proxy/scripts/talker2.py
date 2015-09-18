#!/usr/bin/env python

import rospy
from std_msgs.msg import String

counter = 0

def talker():
    pub = rospy.Publisher( 'hmi_mobile', String, queue_size = 10 )
    rospy.init_node( 'talker', anonymous = True )
    rate = rospy.Rate( 0.2 ) # 0.2hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        global counter
        if counter < 6:
            counter += 1
        else:
            counter = 0
        location_str = str( counter )
        rospy.loginfo( location_str )
        pub.publish( location_str )
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
