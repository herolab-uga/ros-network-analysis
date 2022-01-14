#!/usr/bin/env python
# Description: ROS Node to report the (cumulative) error metrics of a given network. It monitors and reports three type of errors: ip - tcp/udp related (all interfaces); connection - transmit and receive errors (specific interface); interface (nic) - tx and rx errors (specific interface)
# Dependencies: netstat, ethtool (sudo apt-get install net-tools ethtool)
# Author: Ramviyas Parasuraman ramviyas@uga.edu

from subprocess import Popen, PIPE
import rospy
from network_analysis.msg import NetworkErrors
import std_msgs.msg

def network_errors_publisher():
	rospy.init_node('NetworkErrors_publisher', anonymous=True)
	interfacename = rospy.get_param('~INTERFACE_NAME', 'wlan0')
	updaterate = rospy.get_param('~update_rate_network_errors', 1) # Update frequency in Hz.
	pub = rospy.Publisher('network_analysis/network_errors', NetworkErrors, queue_size=10)
	msg = NetworkErrors();
	rate = rospy.Rate(updaterate) # 10hz
	h = std_msgs.msg.Header()
	msg.iface = interfacename
	rospy.loginfo("Launched the network_errors node to monitor the retries, drops, errors in the network link")
	
	cmd1 ="netstat -s | grep retransmitted | awk '{print $1}'"
	cmd2 ="netstat -s | grep 'bad segments' | awk '{print $1}'"
	cmd3 ="netstat -s | grep 'receive errors' | awk '{print $1}'"
	cmd4 ="ethtool -S " + interfacename +" | grep rx_dropped | awk '{print $2}'"
	cmd5 ="ethtool -S " + interfacename +" | grep tx_retries | awk '{print $2}'"
	cmd6 ="cat /sys/class/net/" + interfacename +"/statistics/tx_errors"
	cmd7 ="cat /sys/class/net/" + interfacename +"/statistics/tx_dropped"
	cmd8 ="cat /sys/class/net/" + interfacename +"/statistics/rx_errors"
	cmd9 ="cat /sys/class/net/" + interfacename +"/statistics/rx_dropped"

	while not rospy.is_shutdown():
		#rospy.sleep(1/updaterate)

		h.stamp = rospy.Time.now()
		msg.header = h
		
		#segment errors at (tcp) protocol level
		f = Popen(cmd1,shell=True,stdout=PIPE,stderr=PIPE)
		msg.retransmits = int(f.stdout.read()) # TCP/IP segments retransmited
		#rospy.loginfo("Total TCP Segments retransmited %d", msg.retransmits)
		
		f = Popen(cmd2,shell=True,stdout=PIPE,stderr=PIPE)
		msg.badsegments = int(f.stdout.read()) # TCP/IP bad segments
		#rospy.loginfo("Total TCP Bad Segments %d", msg.badsegments)

		#errors in udp transmission
		f = Popen(cmd3,shell=True,stdout=PIPE,stderr=PIPE)
		msg.udperrors = int(f.stdout.read()) # UDP packet errors
		#rospy.loginfo("Total UDP packet errors %d", msg.udperrors)

		#interface level (NIC statistics) errors
		f = Popen(cmd6,shell=True,stdout=PIPE,stderr=PIPE)
		msg.nic_tx_errors = int(f.stdout.read())
		#rospy.loginfo("%s NIC TX ERRORS %d", interfacename, msg.nic_tx_errors)

		f = Popen(cmd7,shell=True,stdout=PIPE,stderr=PIPE)
		msg.nic_tx_dropped = int(f.stdout.read())
		#rospy.loginfo("%s NIC TX ERRORS %d", interfacename, msg.nic_tx_dropped)

		f = Popen(cmd8,shell=True,stdout=PIPE,stderr=PIPE)
		msg.nic_rx_errors = int(f.stdout.read())
		#rospy.loginfo("%s NIC TX ERRORS %d", interfacename, msg.nic_rx_errors)

		f = Popen(cmd9,shell=True,stdout=PIPE,stderr=PIPE)
		msg.nic_rx_dropped = int(f.stdout.read())
		#rospy.loginfo("%s NIC TX ERRORS %d", interfacename, msg.nic_rx_dropped)

		#system level (MAC layer) errors
		f = Popen(cmd4,shell=True,stdout=PIPE,stderr=PIPE)
		try: 
			msg.rx_dropped = int(f.stdout.read())
			#rospy.loginfo("%s IP RX_DROPPED %d", interfacename, msg.rx_dropped )
		except:
			rospy.loginfo("For ethtool, the specified interface %s does not exist or is disconnected. Reporting only global network errors (not interface specific).",interfacename)
			pub.publish(msg)
			rate.sleep()
			continue
			
		f = Popen(cmd5,shell=True,stdout=PIPE,stderr=PIPE)
		msg.tx_retires = int(f.stdout.read())
		#rospy.loginfo("%s IP TX_RETRIES %d", interfacename, msg.tx_retires)
		
		#rospy.loginfo(msg)
		pub.publish(msg)
		
		rate.sleep()
        	
if __name__ == '__main__':
    try:
		network_errors_publisher()
		rospy.spin()
    except rospy.ROSInterruptException:
        pass
