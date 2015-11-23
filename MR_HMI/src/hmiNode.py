#!/usr/bin/env python

import json
import jsonlib
import sys
import time, threading, datetime
from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from msgs.msg import BoolStamped
from std_msgs.msg import String, Bool
from mr_tip_controller.srv import *
from mr_hmi.srv import *

STATUS_REQUEST = "status_request"
REMOTE_UPDATE = "remote_update"

MR_HMI_SUB = "/mrHMI/status" # receiving location and status updates here

MODE_UPDATE_PUB = "/mrHMI/run"
ACTUATION_ENA_PUB = "/fmSafe/deadman" # a BoolStamped msg. deadman_msg.data = True enables actuation
CMD_VEL_UPDATE_PUB = "/fmCommand/cmd_vel"

TIPPER_UPDATE_SRV = "/mrTipController/tip"

WEB_SOCKET_HOSTNAME = "localhost"
#WEB_SOCKET_HOSTNAME = "10.125.11.201"
WEB_SOCKET_PORT = "8888"
address = ""

direction = 0
button = 0
logMessages = ""

subStatus = 0

pubModeUpdate = 0
# pubTipperUpdate = 0
pubCmdVelUpdate = 0
pubActuationEna = 0

srvTipper = 0

linearVelocity = 0.4
angularVelocity = 0.8

#tipperTilted = False
isManual = False
actuationEna = False
er = True

threadCounter = 0
aThreads = []

class MyServerProtocol( WebSocketServerProtocol ):

    def __init__( self ):
        super( MyServerProtocol, self).__init__()
        self.lock = threading.Lock()

    def onConnect( self, request ):
        global address
        address = request.peer[5:]
        print( "Client connecting: {0}".format( request.peer ) )

    def onOpen( self ):
        print("WebSocket connection open.")

    def onMessage( self, payload, isBinary ):
        global pubModeUpdate
        global actuationEna
        global linearVelocity
        global angularVelocity
        global logMessages

        if isBinary:
            print( "Binary message received: {0} bytes".format( len( payload ) ) )
        else:
            # print( "Text message received: {0}".format( payload.decode( 'utf8' ) ) )

            messageInRaw = payload.decode('utf8')

            messageIn = jsonlib.read( messageInRaw )

            if messageIn["messageType"] == STATUS_REQUEST:

                handleEmergencySituation( messageIn["data"]["resume"] ) # Handle signal from the emergency switch

                massageOutRaw = {
                    "messageType":"status_response",
                    "data":{
                        "log":[logMessages[:-1]]
                    }
                }

                massageOut = json.dumps(massageOutRaw)

                self.sendMessage( massageOut, isBinary )

                logMessages = ""

            elif messageIn["messageType"] == REMOTE_UPDATE:

                leftButton = messageIn["data"]["left"]
                rightButton = messageIn["data"]["right"]
                enaSignal = messageIn["data"]["ena"]
                linearVelocity = messageIn["data"]["linV"]
                angularVelocity = messageIn["data"]["angV"]

                self.updateActuation( enaSignal )

                if leftButton == u"u":
                    drive( linearVelocity, 0.0 )
                elif leftButton == u"r":
                    drive( 0.0, -angularVelocity )
                elif leftButton == u"d":
                    drive( -linearVelocity, 0.0 )
                elif leftButton == u"l":
                    drive( 0.0, angularVelocity )
                elif rightButton == u"x":
                    stop_aThreads()
                    setManualMode( False )
                    tipper( True )
                elif rightButton == u"y":
                    stop_aThreads()
                    setManualMode( False )
                    tipper( False )
                elif rightButton == u"a":
                    stop_aThreads()
                    if isManual == True:
                        setManualMode( False )
                        publishCommand( pubModeUpdate, u"start" )
                elif rightButton == u"b":
                    stop_aThreads()
                    if isManual == True:
                        setManualMode( False )
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

        #if enaSignal == True and posession == True:
            #print "Actuation enabled. [NOT PUBLISHED]"
        #else:
            #print "Actuation disabled. [NOT PUBLISHED]"

class actuationThread( threading.Thread ):

    def __init__( self, threadID, name ):
        threading.Thread.__init__( self )
        self.lock = threading.Lock()
        self.threadID = threadID
        self.name = name
        self.stopFlag = threading.Event()

        print "Starting " + self.name

    def stop( self ):
        print "Stoping " + self.name
        self.stopFlag.set()

    def run( self ):
        print "Starting " + self.name
        while not self.stopFlag.is_set():

            self.publishActuationEna()
            time.sleep(0.033)

    def publishActuationEna( self ):
        global actuationEna

        #print "actuationEna: " + str(actuationEna) + " [PUBLISHED]"

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
    global pubCmdVelUpdate
    global pubActuationEna
    global aThread
    global actuationEna

    if( isManual != newState ):
        print( "Manual mode CHANGED" )
        isManual = newState
        if( isManual ):
            # pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 1 )
            # pubActuationEna = rospy.Publisher( ACTUATION_ENA_PUB, BoolStamped, queue_size = 1 )
            start_an_aThread()
            publishCommand( pubModeUpdate, u"manual" )
            print( "Manual mode is ON" )
        else:
            actuationEna = False
            stop_aThreads()
            #pubCmdVelUpdate.unregister()
            #pubActuationEna.unregister()
            print( "Manual mode is OFF" )

def drive( linearX, angularZ ):
    global pubCmdVelUpdate
    global actuationEna

    setManualMode( True )

    msg = createdTwistedCommand( linearX, angularZ )
    # print( "Msg *to be published: " + str(msg.twist.linear.x) )
    publishCommand( pubCmdVelUpdate, msg )
    # print( "Msg published OK")
    
def tipper( direction ):
    """ Method discription
    Calls the tipping service with a direction specified with a boolean ergument

    True = up
    False = down
    """

    global srvTipper

    srvTipper(direction)# req ) # Request tipping (just ignore the response for now)

def logCallback( data ):
    """ Method Description
    Adds a timestamp to all received messages, and concatenates them to the
    logMessages container that is to be sent to the client at a regular bases.

    Input message format: "code,message,"
    Output message format: "code,timestamp,message,"
    """
    #print(data.data)
    global logMessages

    s = "-"
    d = ","

    temp = data.data[:-1].split( d )

    now = datetime.datetime.now()
    logTimestamp = now.strftime("%Y-%m-%d %H:%M:%S")
    # print(logTimestamp)

    newData = ""
    for i in range( 0, len( temp ), 2 ):
        newData += temp[i] + d + logTimestamp + d + temp[i + 1] + d

    logMessages = logMessages + newData
    # print(data.data)
    # print(logMessages)
    # print(newData)

# TODO check if this works, and publish the messages
def handleEmergencySituation( signal ):
    global er

    # print "resume: " + str(signal)

    if signal != er:
        er = signal

    # TODO publish it here

def publishCommand( rosPublisher, command ):
    rosPublisher.publish( command )

def stopServer():
    reactor.stop()

def stop_aThreads():
    global aThreads

    for temp in aThreads:
        temp.stop()

    del aThreads[:]

def start_an_aThread():
    global aThreads
    global threadCounter

    stop_aThreads()

    t = actuationThread( threadCounter, "actuation_thread_" + str(threadCounter) )
    t.start()

    aThreads.append( t )
    threadCounter += 1

def initHMI():

    global MR_HMI_SUB

    global MODE_UPDATE_PUB
    global CMD_VEL_UPDATE_PUB
    global ACTUATION_ENA_PUB

    global TIPPER_UPDATE_SRV

    global subStatus

    global pubModeUpdate
    # global pubTipperUpdate
    global pubCmdVelUpdate
    global pubActuationEna

    global srvTipper

    global aThread

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node( 'mr_hmi', anonymous = True )

    # Read parameters
    MR_HMI_SUB = rospy.get_param( "~mr_hmi_status_sub", MR_HMI_SUB )

    MODE_UPDATE_PUB = rospy.get_param( "~mr_hmi_run_pub", MODE_UPDATE_PUB )
    CMD_VEL_UPDATE_PUB = rospy.get_param( "~cmd_pub", CMD_VEL_UPDATE_PUB )
    ACTUATION_ENA_PUB = rospy.get_param( "~deadman_pub", ACTUATION_ENA_PUB )

    TIPPER_UPDATE_SRV = rospy.get_param( "~tipper_srv", TIPPER_UPDATE_SRV )

    # Register Subscribers
    subStatus = rospy.Subscriber( MR_HMI_SUB, String, logCallback )

    # Register Publisers
    pubModeUpdate = rospy.Publisher( MODE_UPDATE_PUB, String, queue_size = 10 )
    pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 10 )
    pubActuationEna = rospy.Publisher( ACTUATION_ENA_PUB, BoolStamped, queue_size = 10 )

    # Service Definitions
    rospy.wait_for_service(TIPPER_UPDATE_SRV)
    srvTipper = rospy.ServiceProxy( TIPPER_UPDATE_SRV, tip )

    log.startLogging(sys.stdout)

    # Establish WebSocket connection
    factory = WebSocketServerFactory( u"ws://" + WEB_SOCKET_HOSTNAME + ":" + WEB_SOCKET_PORT, debug = False )
    factory.protocol = MyServerProtocol
    # factory.setProtocolOptions(maxConnections=2)

    reactor.listenTCP( int(WEB_SOCKET_PORT), factory )
    reactor.run()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    initHMI()
