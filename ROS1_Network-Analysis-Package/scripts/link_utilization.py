#!/usr/bin/env python
# Description: ROS Node to measure and monitor network traffic (detailed link utilization metrics) from a given interface connected to a wireless network. It reports total transmit and receive bytes, packets and data transfer rates (overall throughput), in addition to tcp/udp segments/datagrams.
# Author: Ramviyas Parasuraman ramviyas@uga.edu
# License: MIT

import rospy
from subprocess import Popen, PIPE
from network_analysis.msg import LinkUtilization
import std_msgs.msg


def getparameters():
	f = Popen(cmd,shell=True,stdout=PIPE,stderr=PIPE)
	cmd_output = f.stdout.read()
	ans = cmd_output.split()
	if (len(ans) < 1): return 0
	msg.total_tx_packets = int(ans[10])
	msg.total_tx_bytes = int(ans[9])
	msg.total_rx_packets = int(ans[2])
	msg.total_rx_bytes = int(ans[1])
	f = Popen(cmd_netstat_tcp,shell=True,stdout=PIPE,stderr=PIPE)
	cmd_output = f.stdout.read()
	ans = cmd_output.split()
	insegs = len(ans) - 6
	outsegs = len(ans) - 5
	msg.tcp_rx_segments = int(ans[insegs])
	msg.tcp_tx_segments = int(ans[outsegs])
	f = Popen(cmd_netstat_udp,shell=True,stdout=PIPE,stderr=PIPE)
	cmd_output = f.stdout.read()
	ans = cmd_output.split()
	rxdatagrams = len(ans)/2 + 1
	txdatagrams = len(ans)/2 + 4
	msg.udp_rx_datagrams = int(ans[rxdatagrams])
	msg.udp_tx_datagrams = int(ans[txdatagrams])
	return 1

def linkutilization_publisher():
	rospy.init_node('link_utilization_publisher', anonymous=True)
	interfacename = rospy.get_param('~INTERFACE_NAME', 'wlan0') # The Wi-Fi interface id
	updaterate = rospy.get_param('~update_rate_link_utilization', 1) # Update frequency in Hz. Note: more than 1 Hz will not be effective in throughput calculation.
	global cmd,cmd_netstat_tcp,cmd_netstat_udp,msg
	cmd ="cat /proc/net/dev | grep " + interfacename
	cmd_netstat_tcp = "cat /proc/net/snmp | grep Tcp:"
	cmd_netstat_udp = "cat /proc/net/snmp | grep Udp:"
	pub = rospy.Publisher('network_analysis/link_utilization', LinkUtilization, queue_size=10)

	msg = LinkUtilization();
	msg.iface = interfacename
	rate = rospy.Rate(updaterate)
	h = std_msgs.msg.Header()
	fout = getparameters()

	while not rospy.is_shutdown():
		h.stamp = rospy.Time.now()
		msg.header = h
		previous_total_tx_packets = msg.total_tx_packets
		previous_total_tx_bytes = msg.total_tx_bytes
		previous_total_rx_bytes = msg.total_rx_bytes
		previous_tcp_rx_segments = msg.tcp_rx_segments
		previous_tcp_tx_segments = msg.tcp_tx_segments
		previous_udp_rx_datagrams = msg.udp_rx_datagrams
		previous_udp_tx_datagrams  = msg.udp_tx_datagrams
		rospy.sleep(1/updaterate)
		fout = getparameters()	
		if (fout == 0):
			print "The interface %s does not exist or is disconnected",interfacename
			continue
		msg.total_tx_mbps = (8 * (msg.total_tx_bytes - previous_total_tx_bytes) / float(1000*1000))
		msg.total_rx_mbps = (8 * (msg.total_rx_bytes - previous_total_rx_bytes) / float(1000*1000))
		msg.tcp_tx_segmentrate = msg.tcp_tx_segments - previous_tcp_tx_segments
		msg.tcp_rx_segmentrate = msg.tcp_rx_segments - previous_tcp_rx_segments
		msg.udp_tx_datagramrate = msg.udp_tx_datagrams - previous_udp_tx_datagrams
		msg.udp_rx_datagramrate = msg.udp_rx_datagrams - previous_udp_rx_datagrams
		rospy.loginfo("Total throughput on interface %s is Transmit %f Mbps and Receive %f Mbps", interfacename, msg.total_tx_mbps,msg.total_rx_mbps)
		pub.publish(msg)
		rate.sleep()
       	
if __name__ == '__main__':
    try:
		linkutilization_publisher()
		rospy.spin()
    except rospy.ROSInterruptException:
        pass
