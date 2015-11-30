#!/usr/bin/env python

# Imports
import json
import jsonlib
import sys
import time, threading, datetime
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from msgs.msg import BoolStamped
from std_msgs.msg import String, Bool
from mr_tip_controller.srv import *
from mr_navigation_controller.srv import *
from mr_hmi.srv import *
from mr_main.srv import *

# Global variables
STATUS_REQUEST  	= "status_request"
REMOTE_UPDATE   	= "remote_update"
TEXT_FIELD_MSG 		= "text_msg"

MR_MAIN_SUB 		= "/mrMain/mode" # receiving mode from mr main
MR_HMI_SUB 		= "/mrHMI/status" # receiving location and status updates here

ACTUATION_ENA_PUB   	= "/fmSafe/deadman" # a BoolStamped msg. deadman_msg.data = True enables actuation
CMD_VEL_UPDATE_PUB  	= "/fmCommand/cmd_vel"

TIPPER_UPDATE_SRV   	= "/mrTipController/tip"
SET_CURRENT_NODE_SRV 	= "/mrNavigationController/setCurrentNode"
PERFORM_ACTION_SRV  	= "/mrNavigationController/performAction"
MR_MAIN_RUN_SRV     	= "/mrMain/run"

WEB_SOCKET_HOSTNAME 	= "localhost" #"10.125.11.201"
WEB_SOCKET_PORT     	= "8888"
address             	= ""

CURRENT_NODE_MESSAGE    = 0
PERFORM_ACTION_MESSAGE  = 1

direction   		= 0
button      		= 0
logMessages 		= ""

subStatus 		= 0
subRunMode 		= 0

pubModeUpdate  		= 0
#pubTipperUpdate 	= 0
pubCmdVelUpdate 	= 0
pubActuationEna 	= 0

srvTipper           	= 0
srvCurrentMode      	= 0
srvPerformAction    	= 0
srvMainRun 		= 0

linearVelocity  	= 0.4
angularVelocity 	= 0.8

#tipperTilted 		= False
isManual 		= False
actuationEna 		= False
er 			= True

threadCounter 		= 0
aThreads 		= []

IDLE			= 0
AUTO			= 1
MANUAL 			= 2
robotState 		= 0

logOnceBool		= False

# Class
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
        global linearVelocity, angularVelocity
        global logMessages
        global STATUS_REQUEST, REMOTE_UPDATE, TEXT_FIELD_MSG
        global CURRENT_NODE_MESSAGE, PERFORM_ACTION_MESSAGE
        global IDLE, AUTO, MANUAL, robotState 

        #if isBinary:
            #print( "Binary message received: {0} bytes".format( len( payload ) ) )
        #else:
            # print( "Text message received: {0}".format( payload.decode( 'utf8' ) ) )

        messageInRaw = payload.decode('utf8')

        messageIn = jsonlib.read( messageInRaw )

        if messageIn["messageType"] == STATUS_REQUEST:
		# TODO Handle signal from the emergency switch
                handleEmergencySituation( messageIn["data"]["resume"] ) 

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
                    if robotState != AUTO:
	    	        sendMode( u"auto" )
		        robotState = AUTO
                elif rightButton == u"b":
                    stop_aThreads()
                    if isManual == True:
                        setManualMode( False )
                    if robotState != IDLE:
		        sendMode( u"idle" )
		        robotState = IDLE
	# TODO Test it!
        elif messageIn["messageType"] == TEXT_FIELD_MSG: 

            target  = messageIn["data"]["target"]
            text_msg = messageIn["data"]["msg"]

            #print text_msg
            #print target

            if text_msg != "" and target == CURRENT_NODE_MESSAGE:
                sendCurrentNodeMessage( text_msg )
            elif text_msg != "" and target == PERFORM_ACTION_MESSAGE:
                sendPerformActionMessage( text_msg )

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

def setManualMode( newState ):
    global isManual
    global pubCmdVelUpdate
    global pubActuationEna
    global aThread
    global actuationEna
    global IDLE, AUTO, MANUAL, robotState 

    if( isManual != newState ):
        print( "Manual mode CHANGED" )
        isManual = newState
        if( isManual ):
            # pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 1 )
            # pubActuationEna = rospy.Publisher( ACTUATION_ENA_PUB, BoolStamped, queue_size = 1 )
            start_an_aThread() # starts publishing safety signals when manual mode is activated
	    sendMode( u"manual" )
            print( "Manual mode is ON" )
	    robotState = MANUAL
        else:
            actuationEna = False
            stop_aThreads()
            #pubCmdVelUpdate.unregister()
            #pubActuationEna.unregister()
	    sendMode( u"idle" )
            print( "Manual mode is OFF" )
	    robotState = IDLE

def drive( linearX, angularZ ):
    global pubCmdVelUpdate
    global actuationEna

    setManualMode( True )

    msg = createdTwistedCommand( linearX, angularZ )
    # print( "Msg *to be published: " + str(msg.twist.linear.x) )
    publishCommand( pubCmdVelUpdate, msg )
    # print( "Msg published OK")

def tipper( direction ):
    """ Method description
    Calls the tipping service with a direction specified with a boolean ergument

    True = up
    False = down
    """

    global srvTipper

    try:
    	resp = srvTipper( direction ) # Requests tipping (just ignore the response for now)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendMode( mode ):
    """ Method description
    Calls the run service with a state
    "idle", "run", "manual"
    """
    global srvMainRun

    #print mode
    try:
        resp = srvMainRun( mode ) # Sends run message (just ignore the response for now)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendCurrentNodeMessage( msg ):
    """ Method description
    Sends the message to the currentNode service
    """

    global srvCurrentMode

    print msg
    try:
    	resp = srvCurrentMode( msg ) # Sends message (just ignore the response for now)    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def sendPerformActionMessage( msg ):
    """ Method description
    Sends the message to the currentNode service
    """

    global srvPerformAction

    print msg
    try:
    	resp = srvPerformAction( msg ) # Sends message (just ignore the response for now)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def logCallback( data ):
    """ Method Description
    Adds a timestamp to all received messages, and concatenates them to the
    logMessages container that is to be sent to the client at a regular bases.

    Input message format: "code,message,"
    Output message format: "code,timestamp,message,"
    """
    global logMessages, logOnceBool

    #if logOnceBool:

    s = "-"
    d = ","

    temp = data.data[:-1].split( d )

    now = datetime.datetime.now()
    logTimestamp = now.strftime("%Y-%m-%d %H:%M:%S")

    newData = ""
    for i in range( 0, len( temp ), 2 ):
	newData += temp[i] + d + logTimestamp + d + temp[i + 1] + d

    logMessages = logMessages + newData
    #print data.data
    #print newData

    #logOnceBool =  not logOnceBool

def runModeCallback( data ):
    #print "runModeCallback: " + data.data
    if data.data == "auto":
	logCallback( String("1000,Auto mode enabled!,") )
	robotState = AUTO
	#sendMode("auto")
    elif data.data == "idle":
	logCallback( String("2000,Emergency stop!,") )
	robotState = IDLE
	#sendMode("idle")
    elif data.data == "manual":
	logCallback( String("1000,Manual mode!,") )
	robotState = MANUAL
	#sendMode("manual")

# TODO check if this works, and publishes the messages
def handleEmergencySituation( signal ):
    global er

    # print "resume: " + str(signal)

    if signal != er:
        er = signal

    # Publish
    #logCallback( String("3000,Emergency stop!,") )
    #robotState = IDLE
    #sendMode( u"idle" )

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

    global MR_HMI_SUB, MR_MAIN_SUB

    global CMD_VEL_UPDATE_PUB, ACTUATION_ENA_PUB

    global TIPPER_UPDATE_SRV, SET_CURRENT_NODE_SRV, PERFORM_ACTION_SRV, MR_MAIN_RUN_SRV

    global subStatus, subRunMode

    global pubModeUpdate, pubCmdVelUpdate, pubActuationEna

    global srvTipper, srvCurrentMode, srvPerformAction, srvMainRun

    global aThread

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node( 'MR_HMI', anonymous = True )

    # Read parameters
    MR_HMI_SUB = rospy.get_param( "~mr_hmi_status_sub", MR_HMI_SUB )
    MR_MAIN_SUB = rospy.get_param( "~mr_main_mode_sub", MR_MAIN_SUB )

    CMD_VEL_UPDATE_PUB = rospy.get_param( "~cmd_pub", CMD_VEL_UPDATE_PUB )
    ACTUATION_ENA_PUB = rospy.get_param( "~deadman_pub", ACTUATION_ENA_PUB )

    TIPPER_UPDATE_SRV = rospy.get_param( "~tipper_srv", TIPPER_UPDATE_SRV )
    SET_CURRENT_NODE_SRV = rospy.get_param( "~currentNode_srv", SET_CURRENT_NODE_SRV )
    PERFORM_ACTION_SRV = rospy.get_param( "~performAction_srv", PERFORM_ACTION_SRV )
    MR_MAIN_RUN_SRV = rospy.get_param( "~mr_main_run_srv", MR_MAIN_RUN_SRV )

    # Register Subscribers
    subStatus = rospy.Subscriber( MR_HMI_SUB, String, logCallback )
    subRunMode = rospy.Subscriber( MR_MAIN_SUB, String, runModeCallback )

    # Register Publisers
    pubCmdVelUpdate = rospy.Publisher( CMD_VEL_UPDATE_PUB, TwistStamped, queue_size = 10 )
    pubActuationEna = rospy.Publisher( ACTUATION_ENA_PUB, BoolStamped, queue_size = 10 )

    
    # Service Definitions
    #rospy.wait_for_service(TIPPER_UPDATE_SRV)
    srvTipper = rospy.ServiceProxy( TIPPER_UPDATE_SRV, tip )
    #rospy.wait_for_service(SET_CURRENT_NODE_SRV)
    srvCurrentMode = rospy.ServiceProxy( SET_CURRENT_NODE_SRV, setCurrentNode )
    #rospy.wait_for_service(PERFORM_ACTION_SRV)
    srvPerformAction = rospy.ServiceProxy( PERFORM_ACTION_SRV, performAction )
    #rospy.wait_for_service(MR_MAIN_RUN_SRV )
    srvMainRun = rospy.ServiceProxy( MR_MAIN_RUN_SRV , run )

    print "Services ready!"

    log.startLogging(sys.stdout)

    # Establish WebSocket connection
    factory = WebSocketServerFactory( u"ws://" + WEB_SOCKET_HOSTNAME + ":" + WEB_SOCKET_PORT, debug = False )
    factory.protocol = MyServerProtocol
    # factory.setProtocolOptions(maxConnections=2)

    reactor.listenTCP( int(WEB_SOCKET_PORT), factory )
    reactor.run()

    print "We re good!"

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    initHMI()
