#!/usr/bin/env python3
# Description: ROS Node to measure and monitor quality of Wi-Fi link such as the RSSI (received signal strength), LQI (Link Quality Indicator), and the connection status.
# Author: Ramviyas Parasuraman ramviyas@uga.edu
# License: MIT

import os
import rclpy
from ros2_network_analysis_interface.msg import WirelessLink
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import std_msgs.msg
import threading

msg = WirelessLink()


def get_rssi_from_os(interface_name, node):
    msg.rssi = -256  # default rssi value when there is no link
    msg.lqi = 0.0  # default lqi value when there is no link
    msg.noise = -256  # default noise value when there is no link
    msg.iface = interface_name
    msg.status = False

    try:
        cmd = "cat /proc/net/wireless | grep " + interface_name
        f = os.popen(cmd)
        cmd_output = f.read()
        ans = cmd_output.split()

        msg.rssi = int(float(ans[3])) # there is a problem here when i use my interface (wlp0s20f3). Not sure what it is though
        msg.lqi = float(ans[2])
        msg.noise = int(ans[4])
        msg.iface = interface_name
        msg.status = True

    except:
        node.get_logger().info(
            "The specified interface {interface} does not exist or is disconnected. Please check".format(
                interface=interfacename)
        )
        pass


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('wireless_quality')
    node.declare_parameter('interface_name',
		value="wlan0",
		descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, 
            description="The Wi-Fi interface id (e.g. wlan0)",
        ),
	)
    node.declare_parameter('update_rate_wireless_quality',
		value=1.0,
		descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, 
            description="Update frequency in Hz. Note: more than 1 Hz will not be effective in throughput calculation",
        ),
	)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()
    interfacename = node.get_parameter("interface_name").get_parameter_value().string_value # The Wi-Fi interface id wlan0
    node.get_logger().info(f"Wi-Fi Interface Name: {interfacename}")
    update_rate = node.get_parameter("update_rate_wireless_quality").get_parameter_value().double_value # Update frequency in Hz. Note: more than 1 Hz will not be effective in throughput calculation.
    node.get_logger().info(f"Update Frequency: {update_rate} Hz")

    if update_rate <= 0:
        node.get_logger().error(
            "Update rate must be greater than 0. Setting to 10 Hz")
        update_rate = 10

    pub_rssi = node.create_publisher(
        WirelessLink, 'network_analysis/wireless_quality', 10)
    rate = node.create_rate(update_rate)
    node.get_logger().info(
        "Initialized measurement of wireless quality of {interface} interface".format(interface=interfacename))

    h = std_msgs.msg.Header()
    while rclpy.ok():

        h.stamp = node.get_clock().now().to_msg()
        msg = WirelessLink()
        msg.header = h
        get_rssi_from_os(interfacename, node)
        pub_rssi.publish(msg)
        rate.sleep()
