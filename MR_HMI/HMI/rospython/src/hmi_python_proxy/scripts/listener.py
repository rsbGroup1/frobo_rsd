#!/usr/bin/env python

import json
import jsonlib
import sys
from autobahn.twisted.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor

import rospy
from std_msgs.msg import String

LOCATION_REQUEST = "location_request"

address = ""
direction = 0
button = 0
location = 0

class MyServerProtocol(WebSocketServerProtocol):

    def onConnect(self, request):
        global address
        address = request.peer[5:]
        print("Client connecting: {0}".format(request.peer))

    def onOpen(self):
        print("WebSocket connection open.")

    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))

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


    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))

    # def send( self, payload ):
    #     # self.sendMessage(payload, False)
    #
    #     data = format( payload.encode( 'utf8' ) )
    #     protocol = MyServerProtocol()
    #     reactor.callFromThread( protocol.sendMessage, protocol, payload )


# msp = MyServerProtocol()

def callback( data ):
    global location
    location = data.data
    rospy.loginfo( rospy.get_caller_id() + "I heard %s", location )
    # print address
    # MyServerProtocol.send(msp, direction)
    # temp = data.data

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node( 'listener', anonymous = True )

    rospy.Subscriber( "hmi_mobile", String, callback )



    log.startLogging(sys.stdout)

    factory = WebSocketServerFactory(u"ws://localhost:8888", debug=False)
    factory.protocol = MyServerProtocol
    # factory.setProtocolOptions(maxConnections=2)

    reactor.listenTCP(8888, factory)
    reactor.run()



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # log.startLogging(sys.stdout)
    #
    # factory = WebSocketServerFactory(u"ws://localhost:8888", debug=False)
    # factory.protocol = MyServerProtocol
    # # factory.setProtocolOptions(maxConnections=2)
    #
    # reactor.listenTCP(8888, factory)
    # reactor.run()

    listener()
