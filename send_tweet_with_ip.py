#!/usr/bin/env python 
import tweepy, time, sys
from netifaces import AF_INET, AF_INET6, AF_LINK, AF_PACKET, AF_BRIDGE
import netifaces as ni
 
def send_tweet(ip):
	#enter the corresponding information from your Twitter application:
	CONSUMER_KEY = '1234abcd...'#keep the quotes, replace this with your consumer key
	CONSUMER_SECRET = '1234abcd...'#keep the quotes, replace this with your consumer secret key
	ACCESS_KEY = '1234abcd...'#keep the quotes, replace this with your access token
	ACCESS_SECRET = '1234abcd...'#keep the quotes, replace this with your access token secret
	auth = tweepy.OAuthHandler(CONSUMER_KEY, CONSUMER_SECRET)
	auth.set_access_token(ACCESS_KEY, ACCESS_SECRET)
	api = tweepy.API(auth)
	 
	filename=open(argfile,'r')
	f=filename.readlines()
	filename.close()
	 
	try:
		api.update_status(ip)
		print "Tweet sent"
	except:
		print "Problem sending tweet"
		
if __name__ == "__main__":
	#eth0 or wlan0?
	ip = ni.ifaddresses('wlan1')[AF_INET][0]['addr']
	print ip
	send_tweet(ip)
