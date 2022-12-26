#!/usr/bin/env python3
# Description: ROS Node to measure and monitor network traffic (detailed link utilization metrics) from a given interface connected to a wireless network. It reports total transmit and receive bytes, packets and data transfer rates (overall throughput), in addition to tcp/udp segments/datagrams.
# Author: Ramviyas Parasuraman ramviyas@uga.edu
# License: MIT

import rclpy
from subprocess import Popen, PIPE
from ros2_network_analysis.msg import LinkUtilization
import std_msgs.msg
import threading


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
	msg.udp_rx_datagrams = int(ans[int(rxdatagrams)])
	msg.udp_tx_datagrams = int(ans[int(txdatagrams)])
	return 1

def linkutilization_publisher():
	node = rclpy.create_node('link_utilization')
	spin_thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	spin_thread.start()
	interfacename = node.get_parameter_or('~INTERFACE_NAME', 'wlan0') # The Wi-Fi interface id wlan0
	if type(interfacename) == "Parameter":
		interfacename = interfacename.value
	updaterate = node.get_parameter_or('~update_rate_link_utilization', 1) # Update frequency in Hz. Note: more than 1 Hz will not be effective in throughput calculation.
	if type(updaterate) == "Parameter":
		updaterate = updaterate.value
		if updaterate <= 0:
			node.get_logger().error("Update rate should be greater than 0. Setting to 10 hz")
			updaterate = 10
	global cmd,cmd_netstat_tcp,cmd_netstat_udp,msg
	cmd ="cat /proc/net/dev | grep " + interfacename
	cmd_netstat_tcp = "cat /proc/net/snmp | grep Tcp:"
	cmd_netstat_udp = "cat /proc/net/snmp | grep Udp:"
	pub = node.create_publisher(LinkUtilization,'network_analysis/link_utilization',10)

	msg = LinkUtilization()
	msg.iface = interfacename
	rate = node.create_rate(updaterate)
	h = std_msgs.msg.Header()
	fout = getparameters()

	while rclpy.ok():
		h.stamp = node.get_clock().now().to_msg()
		msg.header = h
		previous_total_tx_packets = msg.total_tx_packets
		previous_total_tx_bytes = msg.total_tx_bytes
		previous_total_rx_bytes = msg.total_rx_bytes
		previous_tcp_rx_segments = msg.tcp_rx_segments
		previous_tcp_tx_segments = msg.tcp_tx_segments
		previous_udp_rx_datagrams = msg.udp_rx_datagrams
		previous_udp_tx_datagrams  = msg.udp_tx_datagrams
		rate.sleep()
		fout = getparameters()	
		if (fout == 0):
			print("The interface %s does not exist or is disconnected",interfacename)
			continue
		msg.total_tx_mbps = (8 * (msg.total_tx_bytes - previous_total_tx_bytes) / float(1000*1000))
		msg.total_rx_mbps = (8 * (msg.total_rx_bytes - previous_total_rx_bytes) / float(1000*1000))
		msg.tcp_tx_segmentrate = float(msg.tcp_tx_segments - previous_tcp_tx_segments)
		msg.tcp_rx_segmentrate = float(msg.tcp_rx_segments - previous_tcp_rx_segments)
		msg.udp_tx_datagramrate = float(msg.udp_tx_datagrams - previous_udp_tx_datagrams)
		msg.udp_rx_datagramrate = float(msg.udp_rx_datagrams - previous_udp_rx_datagrams)
		node.get_logger().info(
			"Total throughput on interface {interfacename} is Transmit {tx} Mbps and Receive {rx} Mbps".format(
				interfacename=interfacename, 
				tx=msg.total_tx_mbps,
				rx=msg.total_rx_mbps
			)
		)
		
		pub.publish(msg)
		rate.sleep()
       	
def main():
	rclpy.init()
	linkutilization_publisher()	

if __name__ == '__main__':
	main()