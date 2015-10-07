#!/usr/bin/python

import socket
import random
import time


#TCP/IP client
def client(string):
    HOST, PORT = '10.115.253.233', 21212
    # SOCK_STREAM == a TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #sock.setblocking(0)  # optional non-blocking
    sock.connect((HOST, PORT))

    print "sending data => %s" % (string)
    sock.send(string)
    reply = sock.recv(16384)  # limit reply to 16K
    print "reply => \n [%s]" % (reply)
    sock.close()
    return reply

def main():
    
    str = "Get position 1"
    client(str)

if __name__ == "__main__":
    main()
