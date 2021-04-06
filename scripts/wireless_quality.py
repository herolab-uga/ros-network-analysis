#!/usr/bin/env python
# Description: ROS Node to measure and monitor quality of Wi-Fi link such as the RSSI (received signal strength), LQI (Link Quality Indicator), and the connection status.
# Author: Ramviyas Parasuraman ramviyas@uga.edu
# License: MIT

import os
import rospy
from network_analysis.msg import WirelessLink
import std_msgs.msg

msg = WirelessLink()

def get_rssi_from_os(interface_name):
	msg.rssi = -256 # default rssi value when there is no link
	msg.lqi = 0 # default lqi value when there is no link
	msg.noise = -256 # default noise value when there is no link
	msg.iface = interface_name
	msg.status = False

	try:
		cmd ="cat /proc/net/wireless | grep " + interface_name
		f = os.popen(cmd)
		cmd_output = f.read()
		ans = cmd_output.split()

		msg.rssi = float(ans[3])
		msg.lqi = float(ans[2])
		msg.noise = float(ans[4])
		msg.iface = interface_name
		msg.status = True

	except:
    		rospy.loginfo("The specified interface %s does not exist or is disconnected. Please check",interfacename)
		pass

if __name__ == '__main__':
    try:
	rospy.init_node('rssi_publisher', anonymous=True)
	interfacename = rospy.get_param('~INTERFACE_NAME', 'wlan0')
	update_rate = rospy.get_param('~update_rate_wireless_quality', 10)	
	pub_rssi = rospy.Publisher('network_analysis/wireless_quality', WirelessLink, queue_size=10)
	rate = rospy.Rate(update_rate)
	rospy.loginfo("Initialized measurement of wireless quality of %s interface",interfacename)


	h = std_msgs.msg.Header()
	while not rospy.is_shutdown():

		h.stamp = rospy.Time.now()
		msg = WirelessLink()
 		msg.header = h
		get_rssi_from_os(interfacename)		
	  	pub_rssi.publish(msg)
    		rate.sleep()


    except rospy.ROSInterruptException:
        pass
