#!/usr/bin/env python
import os, sys, time, datetime
import rospy 
from network_analysis.msg import *
import std_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class recorder_rssi_odom:

	msg_vec = WirelessLinkVector()
	odom_val = Odometry()
	vel = Twist()

	time_step = 0
	time_stamp = 0
	robot_pos_x = -2
	robot_pos_y = -2
	robot_orien_x = -2
	robot_orien_y = -2
	robot_orien_z = -2
	robot_orien_w = -2
	robot_twist_x = -2
	robot_twist_y = -2
	robot_twist_z = -2
	vel_x = -2
	vel_y = -2
	vel_w = -2
	network_delay = -2
	c_total_tx_mbps = -2
	c_total_rx_mbps = -2
	c_total_mbps = -2
	c_tx_retires = -2
	c_retransmits = -2
	c_rssi = -2
	c_lqi = -2
	c_rssi_status = -2
	c_total_tx_mbps2 = -2
	c_total_rx_mbps2 = -2
	c_total_mbps2 = -2
	c_rssi2 = -2
	c_lqi2 = -2
	c_rssi_status2 = -2
	s_total_tx_mbps = -2
	s_total_rx_mbps = -2
	s_total_mbps = -2
	s_tx_retires = -2
	s_retransmits = -2
	s_rssi = -2
	s_lqi = -2
	s_rssi_status = -2
	s_total_tx_mbps2 = -2
	s_total_rx_mbps2 = -2
	s_total_mbps2 = -2
	s_tx_retires2 = -2
	s_retransmits2 = -2
	s_rssi2 = -2
	s_lqi2 = -2
	s_rssi_status2 = -2
	
	

	log_on_file = True
    	now = datetime.datetime.now()
	filename = "combined_sync_data.csv"
    	#filename = "/home/kilop/Desktop/OdometryWifi_datalog_" + "{:d}".format(now.month) + '-' + "{:d}".format(now.day) + '_' + "{:d}".format(now.hour) + '-' + "{:d}".format(now.minute)
 
    	file_log = open(filename, 'w')
    

	def __init__(self):
		self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size =1)
		self.sub_cmd_vel = rospy.Subscriber('/teleop_velocity_smoother/raw_cmd_vel', Twist, self.cmd_vel_callback, queue_size =1)
		self.sub_network_delay = rospy.Subscriber('/network_analysis/network_delay', NetworkDelay, self.delay_callback, queue_size =1)
		self.sub_client_link_utilization1 = rospy.Subscriber('/robot/iface1/network_analysis/link_utilization', LinkUtilization, self.c_link1_callback, queue_size =1)
		self.sub_client_network_errors1 = rospy.Subscriber('/robot/iface1/network_analysis/network_errors', NetworkErrors, self.c_error1_callback, queue_size =1)
		self.sub_client_wireless_quality1 = rospy.Subscriber('/robot/iface1/network_analysis/wireless_quality', WirelessLink, self.c_quality1_callback, queue_size =1)
		self.sub_client_link_utilization2 = rospy.Subscriber('/robot/iface2/network_analysis/link_utilization', LinkUtilization, self.c_link2_callback, queue_size =1)
		self.sub_client_wireless_quality2 = rospy.Subscriber('/robot/iface2/network_analysis/wireless_quality', WirelessLink, self.c_quality2_callback, queue_size =1)
		self.sub_server_link_utilization1 = rospy.Subscriber('/server/iface1/network_analysis/link_utilization', LinkUtilization, self.s_link1_callback, queue_size =1)
		self.sub_server_network_errors1 = rospy.Subscriber('/server/iface1/network_analysis/network_errors', NetworkErrors, self.s_error1_callback, queue_size =1)
		self.sub_server_wireless_quality1 = rospy.Subscriber('/server/iface1/network_analysis/wireless_quality', WirelessLink, self.s_quality1_callback, queue_size =1)
		self.sub_server_link_utilization2 = rospy.Subscriber('/server/iface2/network_analysis/link_utilization', LinkUtilization, self.s_link2_callback, queue_size =1)
		self.sub_server_network_errors2 = rospy.Subscriber('/server/iface2/network_analysis/network_errors', NetworkErrors, self.s_error2_callback, queue_size =1)
		self.sub_server_wireless_quality2 = rospy.Subscriber('/server/iface2/network_analysis/wireless_quality', WirelessLink, self.s_quality2_callback, queue_size =1)
 
	def odom_callback(self,data):
		#self.odom_val = data
		self.time_step = data.header.seq
		self.time_stamp = data.header.stamp
		self.robot_pos_x = data.pose.pose.position.x
		self.robot_pos_y = data.pose.pose.position.y
		self.robot_orien_x = data.pose.pose.orientation.x
		self.robot_orien_y = data.pose.pose.orientation.y
		self.robot_orien_z = data.pose.pose.orientation.z
		self.robot_orien_w = data.pose.pose.orientation.w
		self.robot_twist_x = data.twist.twist.linear.x 
		self.robot_iwist_y = data.twist.twist.linear.y
		self.robot_twist_z = data.twist.twist.angular.z

		if (self.log_on_file == True):
			string = "{:11d},".format(self.time_step) + "{:11s},".format(self.time_stamp) + "{:10f},".format(self.robot_pos_x) + "{:10f},".format(self.robot_pos_y) + "{:10f},".format(self.robot_orien_x) + "{:10f},".format(self.robot_orien_y) + "{:10f},".format(self.robot_orien_z) + "{:10f},".format(self.robot_orien_w) + "{:10f},".format(self.robot_twist_x) + "{:10f},".format(self.robot_twist_y) + "{:10f},".format(self.robot_twist_z) + "{:10f},".format(self.vel_x) + "{:10f},".format(self.vel_y) + "{:10f},".format(self.vel_w) + "{:10f},".format(self.network_delay) + "{:10f},".format(self.c_total_tx_mbps) + "{:10f},".format(self.c_total_rx_mbps) + "{:10f},".format(self.c_total_mbps) + "{:10f},".format(self.c_tx_retires) + "{:10f},".format(self.c_retransmits) + "{:10f},".format(self.c_rssi) + "{:10f},".format(self.c_lqi) + "{:2d},".format(self.c_rssi_status) + "{:10f},".format(self.c_total_tx_mbps2) + "{:10f},".format(self.c_total_rx_mbps2) + "{:10f},".format(self.c_total_mbps2) + "{:10f},".format(self.c_rssi2) + "{:10f},".format(self.c_lqi2) + "{:2d},".format(self.c_rssi_status2) + "{:10f},".format(self.s_total_tx_mbps) + "{:10f},".format(self.s_total_rx_mbps) + "{:10f},".format(self.s_total_mbps) + "{:10f},".format(self.s_tx_retires) + "{:10f},".format(self.s_retransmits) + "{:10f},".format(self.s_rssi) + "{:10f},".format(self.s_lqi) + "{:2d},".format(self.s_rssi_status) + "{:10f},".format(self.s_total_tx_mbps2) + "{:10f},".format(self.s_total_rx_mbps2) + "{:10f},".format(self.s_total_mbps2) + "{:10f},".format(self.s_tx_retires2) + "{:10f},".format(self.s_retransmits2) + "{:10f},".format(self.s_rssi2) + "{:10f},".format(self.s_lqi2) + "{:2d},".format(self.s_rssi_status2) + "\n"
		self.file_log.write (string)



	def cmd_vel_callback(self,data):
		self.vel_x = data.linear.x
		self.vel_y = data.linear.y
		self.vel_w = data.angular.z	
	
	def delay_callback(self, data):
		#self.msg_vec = data
		self.network_delay = data.delay
        
	def c_link1_callback(self, data):
		#self.msg_vec = data
        	self.c_total_tx_mbps = data.total_tx_mbps
		self.c_total_rx_mbps = data.total_rx_mbps
		self.c_total_mbps = data.total_tx_mbps + data.total_rx_mbps

	def c_error1_callback(self, data):
		#self.msg_vec = data
        	self.c_tx_retires = data.tx_retires
		self.c_retransmits = data.retransmits

	def c_quality1_callback (self, data):
		#self.msg_vec = data
        	self.c_rssi = data.rssi
		self.c_lqi = data.lqi
		self.c_rssi_status = data.status

	def c_link2_callback(self, data):
		#self.msg_vec = data
        	self.c_total_tx_mbps2 = data.total_tx_mbps
		self.c_total_rx_mbps2 = data.total_rx_mbps
		self.c_total_mbps2 = data.total_tx_mbps + data.total_rx_mbps

	def c_quality2_callback (self, data):
		#self.msg_vec = data
        	self.c_rssi2 = data.rssi
		self.c_lqi2 = data.lqi
		self.c_rssi_status2 = data.status

	def s_link1_callback(self, data):
		#self.msg_vec = data
        	self.s_total_tx_mbps = data.total_tx_mbps
		self.s_total_rx_mbps = data.total_rx_mbps
		self.s_total_mbps = data.total_tx_mbps + data.total_rx_mbps

	def s_error1_callback(self, data):
		#self.msg_vec = data
        	self.s_tx_retires = data.tx_retires
		self.s_retransmits = data.retransmits

	def s_quality1_callback (self, data):
		#self.msg_vec = data
        	self.s_rssi = data.rssi
		self.s_lqi = data.lqi
		self.s_rssi_status = data.status

	def s_link2_callback(self, data):
		#self.msg_vec = data
        	self.s_total_tx_mbps2 = data.total_tx_mbps
		self.s_total_rx_mbps2 = data.total_rx_mbps
		self.s_total_mbps2 = data.total_tx_mbps + data.total_rx_mbps

	def s_error2_callback(self, data):
		#self.msg_vec = data
        	self.s_tx_retires2 = data.tx_retires
		self.s_retransmits2 = data.retransmits

	def s_quality2_callback (self, data):
		#self.msg_vec = data
        	self.s_rssi2 = data.rssi
		self.s_lqi2 = data.lqi
		self.s_rssi_status2 = data.status


	def function_init(self):
		if (self.log_on_file == True):
			string ="time_step,time_stamp,robot_pos_x,robot_pos_y,robot_orien_x,robot_orien_y,robot_orien_z,robot_orien_w,robot_twist_x,robot_twist_y,robot_twist_z,vel_x,vel_y,vel_w,network_delay,c_total_tx_mbps,c_total_rx_mbps,c_total_mbps,c_tx_retires,c_retransmits,c_rssi,c_lqi,c_rssi_status,c_total_tx_mbps2,c_total_rx_mbps2,c_total_mbps2,c_rssi2,c_lqi2,c_rssi_status2,s_total_tx_mbps,s_total_rx_mbps,s_total_mbps,s_tx_retires,s_retransmits,s_rssi,s_lqi,s_rssi_status,s_total_tx_mbps2,s_total_rx_mbps2,s_total_mbps2,s_tx_retires2,s_retransmits2,s_rssi2,s_lqi2,s_rssi_status2\n"
		self.file_log.write (string)



def main(args):
    '''Initializes and cleanup ros node'''
    rec = recorder_rssi_odom()
    rospy.init_node('record_rssi_odom_node', anonymous=True)
    rec.function_init()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
