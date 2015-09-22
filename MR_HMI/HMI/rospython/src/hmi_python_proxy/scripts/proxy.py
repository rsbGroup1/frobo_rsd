#!/usr/bin/env python

import json
import jsonlib
import sys
import threading
from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from msgs.msg import BoolStamped
from std_msgs.msg import String, Bool

LOCATION_REQUEST = "location_request"
REMOTE_UPDATE = "remote_update"

MODE_UPDATE_PUB = "startStopTopic"
ACTUATION_ENA_PUB = "/fmSafe/deadman" # a BoolStamped msg. deadman_msg.data = True enables actuation
CMD_VEL_UPDATE_PUB = "/fmCommand/cmd_vel"
TIPPER_UPDATE_PUB = "mr_tipper_update"

address = ""
direction = 0
button = 0
location = 0
pubModeUpdate = 0
pubTipperUpdate = 0
pubCmdVelUpdate = 0
pubActuationEna = 0

tipperTilted = False
actuationEna = False

threadLock = threading.Lock()

class MyServerProtocol( WebSocketServerProtocol ):

    def onConnect( self, request ):
        global address
        address = request.peer[5:]
        print("Client connecting: {0}".format(request.peer))

    def onOpen( self ):
        print("WebSocket connection open.")

    def onMessage( self, payload, isBinary ):
        global pubModeUpdate
        global actuationEna

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
                enaSignal = messageIn["data"]["ena"]

                if enaSignal != actuationEna:
                    actuationEna = enaSignal
                    if actuationEna == True:
                        print "Actuation enabled."
                    else:
                        print "Actuation disabled."
                    
                publishActuationEna()

                if leftButton == u"u":
                    drive( 0.2, 0.0 )
                elif leftButton == u"r":
                    drive( 0.0, -0.8 )
                elif leftButton == u"d":
                    drive( -0.2, 0.0 )
                elif leftButton == u"l":
                    drive( 0.0, 0.8 )
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


class actuationThread( threading.Thread ):
    global threadLock

    def __init__( self, threadID, name ):
        threading.Thread.__init__( self )
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        print "Starting " + self.name
	
	
        # Get lock to synchronize threads
        threadLock.acquire()
        
        

        # Free lock to release next thread
        threadLock.release()

def createdTwistedCommand( linearX, angularZ ):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.twist.linear.x = linearX
    msg.twist.angular.z = angularZ

    return msg

def createBoolStampedMessage( data ):
    msg = BoolStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.data = data

    return msg

def drive( linearX, angularZ ):
    global pubCmdVelUpdate

    msg = createdTwistedCommand( linearX, angularZ )
    publishCommand( pubCmdVelUpdate, msg )

def publishActuationEna():
    global actuationEna

    msg = createBoolStampedMessage( actuationEna )
    pubActuationEna.publish ( msg )

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
    global pubActuationEna

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node( 'proxy', anonymous = True )

    # register Publisers
    pubModeUpdate = rospy.Publisher( MODE_UPDATE_PUB, String, queue_size = 1 )
    pubTipperUpdate = rospy.Publisher( TIPPER_UPDATE_PUB, Bool, queue_size = 1 )
    pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 1 )
    pubActuationEna = rospy.Publisher( ACTUATION_ENA_PUB, BoolStamped, queue_size = 1 )

    # register Listeners
    rospy.Subscriber( "hmi_mobile", String, callback )

    # start publishing activationEna sygnal in a separate thread
    

    log.startLogging(sys.stdout)

    # Establish WebSocket connection
    factory = WebSocketServerFactory( u"ws://localhost:8888", debug = False )
    factory.protocol = MyServerProtocol
    # factory.setProtocolOptions(maxConnections=2)

    reactor.listenTCP(8888, factory)
    reactor.run()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    initProxy()
