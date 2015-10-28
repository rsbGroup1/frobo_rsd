#!/bin/bash
iwconfig wlan1 essid SDU_GUEST
dhclient wlan1
wait
exec send_email_with_ip.py
