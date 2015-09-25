#!/usr/bin/env python

import json
import jsonlib
import sys
import time, threading
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

MODE_UPDATE_PUB = "/mrHMI/start_stop"
ACTUATION_ENA_PUB = "/fmSafe/deadman" # a BoolStamped msg. deadman_msg.data = True enables actuation
CMD_VEL_UPDATE_PUB = "/fmCommand/cmd_vel"
TIPPER_UPDATE_PUB = "/mrMainController/tipper"

WEB_SOCKET_HOSTNAME = "localhost"
#WEB_SOCKET_HOSTNAME = "10.125.11.201"
WEB_SOCKET_PORT = "8888"
address = ""

direction = 0
button = 0
location = 0

pubModeUpdate = 0
pubTipperUpdate = 0
pubCmdVelUpdate = 0
pubActuationEna = 0

#tipperTilted = False
isManual = False
actuationEna = False

class MyServerProtocol( WebSocketServerProtocol ):
    def __init__( self ):
        self.lock = threading.Lock()

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

                self.updateActuation( enaSignal )

                if leftButton == u"u":
                    drive( 0.2, 0.0 )
                elif leftButton == u"r":
                    drive( 0.0, -0.8 )
                elif leftButton == u"d":
                    drive( -0.2, 0.0 )
                elif leftButton == u"l":
                    drive( 0.0, 0.8 )
                elif rightButton == u"x":
                    tip( True )
                elif rightButton == u"y":
                    tip( False )
                elif rightButton == u"a":
                    setManualMode( False )
                    publishCommand( pubModeUpdate, u"start" )
                elif rightButton == u"b":
                    publishCommand( pubModeUpdate, u"stop" )

    def onClose( self, wasClean, code, reason ):
        global actuationEna

        setManualMode( False )
        actuationEna = False

        print( "WebSocket connection closed: {0}".format( reason ) )

    def updateActuation( self, enaSignal ):
        global actuationEna
        posession = False

        posession = self.lock.acquire()
        if actuationEna != enaSignal:
            actuationEna = enaSignal
        self.lock.release()

        if posession == False:
            self.updateActuation( enaSignal )

        if enaSignal == True and posession == True:
            print "Actuation enabled."
        else:
            print "Actuation disabled."

class actuationThread( threading.Thread ):

    def __init__( self, threadID, name ):
        threading.Thread.__init__( self )
        self.lock = threading.Lock()
        self.threadID = threadID
        self.name = name

    def run( self ):
        print "Starting " + self.name
        while True:
            self.publishActuationEna()
            time.sleep(0.05)

    def publishActuationEna( self ):
        global actuationEna

        print "actuationEna: " + str(actuationEna)

        self.lock.acquire()
        msg = createBoolStampedMessage( actuationEna )
        pubActuationEna.publish ( msg )
        self.lock.release()

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

def setManualMode( newState ):
    global isManual

    if( isManual != newState ):
        print( "Manual mode CHANGED" )
        isManual = newState
        if( isManual ):
            publishCommand( pubModeUpdate, u"manual" )
            print( "Manual mode is ON" )
        else:
            print( "Manual mode is OFF" )

def drive( linearX, angularZ ):
    global pubCmdVelUpdate

    msg = createdTwistedCommand( linearX, angularZ )
    setManualMode( True )
    # print( "Msg to be published: " + str(msg.twist.linear.x) )
    publishCommand( pubCmdVelUpdate, msg )
    # print( "Msg published OK")

def tip( direction ):
    global pubTipperUpdate
    global tipperTilted

    publishCommand( pubTipperUpdate, direction )

def callback( data ):
    global location
    location = data.data
    rospy.loginfo( rospy.get_caller_id() + "I heard %s", location )

def publishCommand( rosPublisher, command ):
    rosPublisher.publish( command )

def initProxy():

    global MODE_UPDATE_PUB
    global TIPPER_UPDATE_PUB
    global CMD_VEL_UPDATE_PUB
    global ACTUATION_ENA_PUB

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

    # Read parameters
    MODE_UPDATE_PUB = rospy.get_param( "~missionplanner_pub", MODE_UPDATE_PUB )
    TIPPER_UPDATE_PUB = rospy.get_param( "~deadman_pub", TIPPER_UPDATE_PUB )
    CMD_VEL_UPDATE_PUB = rospy.get_param( "~cmd_pub", CMD_VEL_UPDATE_PUB )
    ACTUATION_ENA_PUB = rospy.get_param( "~tipper_pub", ACTUATION_ENA_PUB )

    # Register Publisers
    pubModeUpdate = rospy.Publisher( MODE_UPDATE_PUB, String, queue_size = 1 )
    pubTipperUpdate = rospy.Publisher( TIPPER_UPDATE_PUB, Bool, queue_size = 1 )
    pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 1 )
    pubActuationEna = rospy.Publisher( ACTUATION_ENA_PUB, BoolStamped, queue_size = 1 )

    # Register Listeners
    rospy.Subscriber( "hmi_mobile", String, callback )

    # Start publishing the activationEna sygnal in a separate thread
    aThread = actuationThread( 1, "actuation_thread" )
    aThread.start()

    log.startLogging(sys.stdout)

    # Establish WebSocket connection
    factory = WebSocketServerFactory( u"ws://" + WEB_SOCKET_HOSTNAME + ":" + WEB_SOCKET_PORT, debug = False )
    factory.protocol = MyServerProtocol
    # factory.setProtocolOptions(maxConnections=2)

    reactor.listenTCP( int(WEB_SOCKET_PORT), factory )
    reactor.run()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    initProxy()
