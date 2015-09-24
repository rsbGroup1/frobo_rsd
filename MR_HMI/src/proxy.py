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

tipperTilted = False
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

                self.updateActuation( enaSignal )

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
        global actuationEna

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
        pubActuationEna.publish( msg )
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

def drive( linearX, angularZ ):
    global pubCmdVelUpdate

    msg = createdTwistedCommand( linearX, angularZ )
    publishCommand( pubCmdVelUpdate, msg )

def tip():
    global pubTipperUpdate
    global tipperTilted

    tipperTilted = not tipperTilted
    publishCommand( pubTipperUpdate, "up" )

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
    missionPlanTopic = rospy.get_param("~missionplanner_pub", "/mrHMI/start_stop")
    deadmanTopic = rospy.get_param("~deadman_pub", "/fmSafe/deadman")
    cmdTopic = rospy.get_param("~cmd_pub", "/fmCommand/cmd_vel")
    tipperTopic = rospy.get_param("~tipper_pub", "/mrMainController/tipper")

    print(missionPlanTopic + " " + deadmanTopic)

    pubModeUpdate = rospy.Publisher( missionPlanTopic, String, queue_size = 1 )
    pubTipperUpdate = rospy.Publisher( tipperTopic, String, queue_size = 1 )
    pubCmdVelUpdate = rospy.Publisher( cmdTopic, TwistStamped, queue_size = 1 )
    pubActuationEna = rospy.Publisher( deadmanTopic, BoolStamped, queue_size = 1 )

    # register Listeners
    rospy.Subscriber( "hmi_mobile", String, callback )

    # start publishing activationEna sygnal in a separate thread
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
