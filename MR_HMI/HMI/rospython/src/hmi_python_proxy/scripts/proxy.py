#!/usr/bin/env python

import json
import jsonlib
import sys
from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String, Bool

LOCATION_REQUEST = "location_request"
REMOTE_UPDATE = "remote_update"

MODE_UPDATE_PUB = "startStopTopic"
CMD_VEL_UPDATE_PUB = "/fmCommand/cmd_vel"
TIPPER_UPDATE_PUB = "mr_tipper_update"

address = ""
direction = 0
button = 0
location = 0
pubModeUpdate = 0
pubTipperUpdate = 0
pubCmdVelUpdate = 0

tipperTilted = False

class MyServerProtocol( WebSocketServerProtocol ):

    def onConnect( self, request ):
        global address
        address = request.peer[5:]
        print("Client connecting: {0}".format(request.peer))

    def onOpen( self ):
        print("WebSocket connection open.")

    def onMessage( self, payload, isBinary ):
        global pubModeUpdate

        if isBinary:
            print( "Binary message received: {0} bytes".format( len( payload ) ) )
        else:
            # print( "Text message received: {0}".format( payload.decode( 'utf8' ) ) )

            messageInRaw = payload.decode('utf8')

            messageIn = jsonlib.read( messageInRaw )

            # print("RETEK: " + messageIn["messageType"] )

            if messageIn["messageType"] == LOCATION_REQUEST:

                massageOutRaw = {
                    "messageType":"location_response",
                    "data":str(location)
                }

                massageOut = json.dumps(massageOutRaw)

                self.sendMessage( massageOut, isBinary )

            elif messageIn["messageType"] == REMOTE_UPDATE:

                leftButton = messageIn["data"]["left"]
                rightButton = messageIn["data"]["right"]

                if leftButton == u"u":
                    drive( 0.2, 0.0 )
                elif leftButton == u"r":
                    drive( 0.0, -0.3 )
                elif leftButton == u"d":
                    drive( -0.2, 0.0 )
                elif leftButton == u"l":
                    drive( 0.0, 0.3 )
                elif rightButton == u"y":
                    tip()
                elif rightButton == u"x":
                    publishCommand( pubModeUpdate, u"manual" )
                elif rightButton == u"a":
                    publishCommand( pubModeUpdate, u"start" )
                elif rightButton == u"b":
                    publishCommand( pubModeUpdate, u"stop" )

    def onClose( self, wasClean, code, reason ):
        print( "WebSocket connection closed: {0}".format( reason ) )

def createdTwistedCommand( linearX, angularZ ):

    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.twist.linear.x = linearX
    msg.twist.angular.z = angularZ

    return msg

def drive( linearX, angularZ ):
    global pubCmdVelUpdate

    msg = createdTwistedCommand( linearX, angularZ )
    publishCommand( pubCmdVelUpdate, msg )

    print "driving..."

def tip():
    global pubTipperUpdate
    global tipperTilted

    tipperTilted = not tipperTilted

    print "tipping"

def callback( data ):
    global location
    location = data.data
    rospy.loginfo( rospy.get_caller_id() + "I heard %s", location )

def publishCommand( rosPublisher, command ):
    rosPublisher.publish( command )

def initProxy():

    global pubModeUpdate
    global pubTipperUpdate
    global pubCmdVelUpdate

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node( 'proxy', anonymous = True )

    # register Publisers
    pubModeUpdate = rospy.Publisher( MODE_UPDATE_PUB, String, queue_size = 10 )
    pubTipperUpdate = rospy.Publisher( TIPPER_UPDATE_PUB, Bool, queue_size = 10 )
    pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 10 )


    # register Listeners
    rospy.Subscriber( "hmi_mobile", String, callback )

    log.startLogging(sys.stdout)

    # Establish WebSocket connection
    factory = WebSocketServerFactory( u"ws://localhost:8888", debug = False )
    factory.protocol = MyServerProtocol
    # factory.setProtocolOptions(maxConnections=2)

    reactor.listenTCP(8888, factory)
    reactor.run()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    initProxy()
