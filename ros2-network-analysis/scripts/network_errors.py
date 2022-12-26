#!/usr/bin/env python3
# Description: ROS Node to report the (cumulative) error metrics of a given network. It monitors and reports three type of errors: ip - tcp/udp related (all interfaces); connection - transmit and receive errors (specific interface); interface (nic) - tx and rx errors (specific interface)
# Dependencies: netstat, ethtool (sudo apt-get install net-tools ethtool)
# Author: Ramviyas Parasuraman ramviyas@uga.edu

from subprocess import Popen, PIPE
import rclpy
from ros2_network_analysis.msg import NetworkErrors
import std_msgs.msg
import threading


def network_errors_publisher():
    node = rclpy.create_node('NetworkErrors_publisher')
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()
    interfacename = node.get_parameter_or(
        '~INTERFACE_NAME', 'wlan0')
    if type(interfacename) == "Parameter":
        interfacename = interfacename.value
    # Update frequency in Hz.
    updaterate = node.get_parameter_or('~update_rate_network_errors', 1)
    if type(updaterate) == "Parameter":
        updaterate = updaterate.value
        if updaterate <= 0:
            node.get_logger().error("Update rate should be greater than 0. Setting to 10 hz")
            updaterate = 10
    pub = node.create_publisher(
        NetworkErrors, 'network_analysis/network_errors', 10)
    msg = NetworkErrors()
    rate = node.create_rate(updaterate)  # 10hz
    h = std_msgs.msg.Header()
    msg.iface = interfacename
    node.get_logger().info(
        "Launched the network_errors node to monitor the retries, drops, errors in the network link")

    cmd1 = "netstat -s | grep retransmitted | awk '{print $1}'"
    cmd2 = "netstat -s | grep 'bad segments' | awk '{print $1}'"
    cmd3 = "netstat -s | grep 'receive errors' | awk '{print $1}'"
    cmd4 = "ethtool -S " + interfacename + \
        " | grep rx_dropped | awk '{print $2}'"
    cmd5 = "ethtool -S " + interfacename + \
        " | grep tx_retries | awk '{print $2}'"
    cmd6 = "cat /sys/class/net/" + interfacename + "/statistics/tx_errors"
    cmd7 = "cat /sys/class/net/" + interfacename + "/statistics/tx_dropped"
    cmd8 = "cat /sys/class/net/" + interfacename + "/statistics/rx_errors"
    cmd9 = "cat /sys/class/net/" + interfacename + "/statistics/rx_dropped"

    while rclpy.ok():
        # rospy.sleep(1/updaterate)

        h.stamp = node.get_clock().now().to_msg()
        msg.header = h

        # segment errors at (tcp) protocol level
        f = Popen(cmd1, shell=True, stdout=PIPE, stderr=PIPE)
        msg.retransmits = int(f.stdout.read())  # TCP/IP segments retransmited
        #node.get_logger.info("Total TCP Segments retransmited %d", msg.retransmits)

        f = Popen(cmd2, shell=True, stdout=PIPE, stderr=PIPE)
        msg.badsegments = int(f.stdout.read())  # TCP/IP bad segments
        #node.get_logger.info("Total TCP Bad Segments %d", msg.badsegments)

        # errors in udp transmission
        f = Popen(cmd3, shell=True, stdout=PIPE, stderr=PIPE)
        msg.udperrors = int(f.stdout.read())  # UDP packet errors
        #node.get_logger.info("Total UDP packet errors %d", msg.udperrors)

        # interface level (NIC statistics) errors
        f = Popen(cmd6, shell=True, stdout=PIPE, stderr=PIPE)
        msg.nic_tx_errors = int(f.stdout.read())
        #node.get_logger.info("%s NIC TX ERRORS %d", interfacename, msg.nic_tx_errors)

        f = Popen(cmd7, shell=True, stdout=PIPE, stderr=PIPE)
        msg.nic_tx_dropped = int(f.stdout.read())
        #node.get_logger.info("%s NIC TX ERRORS %d", interfacename, msg.nic_tx_dropped)

        f = Popen(cmd8, shell=True, stdout=PIPE, stderr=PIPE)
        msg.nic_rx_errors = int(f.stdout.read())
        #node.get_logger.info("%s NIC TX ERRORS %d", interfacename, msg.nic_rx_errors)

        f = Popen(cmd9, shell=True, stdout=PIPE, stderr=PIPE)
        msg.nic_rx_dropped = int(f.stdout.read())
        #node.get_logger.info("%s NIC TX ERRORS %d", interfacename, msg.nic_rx_dropped)

        # system level (MAC layer) errors
        f = Popen(cmd4, shell=True, stdout=PIPE, stderr=PIPE)
        try:
            msg.rx_dropped = int(f.stdout.read())
            #node.get_logger.info("%s IP RX_DROPPED %d", interfacename, msg.rx_dropped )
        except:
            node.get_logger().info(
                "For ethtool, the specified interface {interface} does not exist or is disconnected. Reporting only global network errors (not interface specific).".format(interface=interfacename))
            pub.publish(msg)
            rate.sleep()
            continue

        f = Popen(cmd5, shell=True, stdout=PIPE, stderr=PIPE)
        msg.tx_retires = int(f.stdout.read())
        #node.get_logger.info("%s IP TX_RETRIES %d", interfacename, msg.tx_retires)

        # node.get_logger.info(msg)
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':

    rclpy.init()
    network_errors_publisher()
