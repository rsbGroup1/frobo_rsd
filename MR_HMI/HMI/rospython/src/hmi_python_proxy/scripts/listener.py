#!/usr/bin/env python

import sys
from twisted.python import log
from twisted.internet import reactor

import rospy
from std_msgs.msg import String

def callback( data ):
    rospy.loginfo( rospy.get_caller_id() + "I heard %s", data.data )

def listen():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node( 'listener', anonymous = True )

    # register Listeners
    rospy.Subscriber( "startStopTopic", String, callback )

    log.startLogging( sys.stdout )

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listen()
