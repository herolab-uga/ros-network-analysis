#!/usr/bin/env python
#Deascription: ROS Node for network bandwidth monitoring/link utilization metrics in a given network: reports transmit and receive bytes, packets and rates
#Author: Ramviyas Parasuraman ramviyas@purdue.edu
#License: MIT

import os
import rospy
from std_msgs.msg import String
from network_analysis.msg import LinkUtilization
from time import sleep
import std_msgs.msg


def getparameters():
	f = os.popen(cmd)
	cmd_output = f.read()
	ans = cmd_output.split()

	msg.total_tx_packets = int(ans[10])
	msg.total_tx_bytes = int(ans[9])
	msg.total_rx_packets = int(ans[2])
	msg.total_rx_bytes = int(ans[1])
	f = os.popen(cmd_netstat_tcp)
	cmd_output = f.read()
	ans = cmd_output.split()
	insegs = len(ans) - 6
	outsegs = len(ans) - 5
	msg.tcp_rx_segments = int(ans[insegs])
	msg.tcp_tx_segments = int(ans[outsegs])
	f = os.popen(cmd_netstat_udp)
	cmd_output = f.read()
	ans = cmd_output.split()
	rxdatagrams = len(ans)/2 + 1
	txdatagrams = len(ans)/2 + 4
	msg.udp_rx_datagrams = int(ans[rxdatagrams])
	msg.udp_tx_datagrams = int(ans[txdatagrams])

def linkutilization_publisher():
	interfacename = rospy.get_param('INTERFACE_NAME', 'wlan0')
	global cmd,cmd_netstat_tcp,cmd_netstat_udp,msg
	cmd ="cat /proc/net/dev | grep " + interfacename
	cmd_netstat_tcp = "cat /proc/net/snmp | grep Tcp:"
	cmd_netstat_udp = "cat /proc/net/snmp | grep Udp:"
	pub = rospy.Publisher('/network_analysis/link_utilization', LinkUtilization, queue_size=10)
	rospy.init_node('link_utilization_publisher', anonymous=True)

	msg = LinkUtilization();
	msg.iface = interfacename
	rate = rospy.Rate(1) # 10hz
	h = std_msgs.msg.Header()
	getparameters()

	while not rospy.is_shutdown():
		h.stamp = rospy.Time.now()
		msg.header = h
		previous_total_tx_packets = msg.total_tx_packets
		previous_total_tx_bytes = msg.total_tx_bytes
		previous_total_rx_packets = msg.total_rx_packets
		previous_total_rx_bytes = msg.total_rx_bytes
		previous_tcp_rx_segments = msg.tcp_rx_segments
		previous_tcp_tx_segments = msg.tcp_tx_segments
		previous_udp_rx_datagrams = msg.udp_rx_datagrams
		previous_udp_tx_datagrams  = msg.udp_tx_datagrams
		sleep(1)
		getparameters()	
		msg.total_tx_mbps = (8 * (msg.total_tx_bytes - previous_total_tx_bytes) / float(1000*1000))
		msg.total_rx_mbps = (8 * (msg.total_rx_bytes - previous_total_rx_bytes) / float(1000*1000))
		msg.tcp_tx_segmentrate = msg.tcp_tx_segments - previous_tcp_tx_segments
		msg.tcp_rx_segmentrate = msg.tcp_rx_segments - previous_tcp_rx_segments
		msg.udp_tx_datagramrate = msg.udp_tx_datagrams - previous_udp_tx_datagrams
		msg.udp_rx_datagramrate = msg.udp_rx_datagrams - previous_udp_rx_datagrams
		print msg.total_rx_mbps
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
       	
if __name__ == '__main__':
    try:
		linkutilization_publisher()
		rospy.spin()
    except rospy.ROSInterruptException:
        pass
