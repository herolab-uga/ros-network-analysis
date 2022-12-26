// Description: This ROS node calculates delay in the network without using Ping/ICMP echo requests. It uses ROS action calls instead, thus measuring the round trip time (2xlatency) at the applicaiton level layer (with TCP connection).
// Author: Ramviyas Parasuraman ramviyas@uga.edu
// License: MIT

#include <rclcpp/rclcpp.hpp>
#include <ros2_network_analysis/msg/network_delay.hpp>
#include <ros2_network_analysis/action/ping.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <sys/time.h>
#include <iostream>

struct timeval start, end;
ros2_network_analysis::msg::NetworkDelay msg;

void reply_callback(const rclcpp_action::ClientGoalHandle<ros2_network_analysis::action::Ping>::WrappedResult &result)
{
	gettimeofday(&end, NULL);
	


	msg.header.stamp = rclcpp::Time::now();
	msg.iface = "default";
	msg.delay = -1; // default values
	msg.alive = 0;	// default values
	double mtime, seconds, useconds;

	if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
	{
		// *actionlib::SimpleClientGoalState state = ac.getState(); <- this does not exist anymore
		RCLCPP_INFO(n->get_logger(), "ROS Ping Action finished: %s", result.result->out.c_str());
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds)*1000 + useconds / 1000.0) + 0.5;
		msg.delay = (float)mtime;
		msg.alive = 1;
	}
	else
	{
		// *ROS_ERROR("Action did not finish before the timeout. May be a network problem or may be the ping action server node stopped. See network_delay.alive history for more diagnosis");

		msg.delay = -1; // error values
		msg.alive = 0;	// error values
	}
}

int main(int argc, char *argv[])
{
	try
	{
		rclcpp::init(argc, argv);

		auto n = rclcpp::Node::make_shared("network_delay");
		double timeout = 0;
		n->declare_parameter("timeout_network_delay", 2.0); // timeout to wait for action (in seconds);
		int updaterate = 0;
		n->declare_parameter("update_rate_network_delay", 1); // update freqency to publish in the topic;

		// Publish the delay values in the /network_analysis/network_delay topic
		auto pub = n->create_publisher<ros2_network_analysis::msg::NetworkDelay>("network_analysis/network_delay", 10);
		

		// delay calculation using ROS action calls
		auto ac = rclcpp_action::create_client<ros2_network_analysis::action::Ping>(n, "Ping");
		ac->wait_for_action_server(); // will wait for infinite time until the pingactionserver is started
		ros2_network_analysis::action::Ping goal;
		goal.inp = true;

		rclcpp::Rate loop_rate(updaterate);
		// ros::Time last_time = ros::Time::now();

		while (rclcpp::ok())
		{
			auto send_goal_options = rclcpp_action::Client<ros2_network_analysis::action::Ping>::SendGoalOptions();
			// send_goal_options.goal_response_callback = ;
			send_goal_options.result_callback = std::bind(&reply_callback, n ,std::placeholders::_1);
			gettimeofday(&start, NULL);
			ac->async_send_goal(goal,send_goal_options);
			pub->publish(msg);
			// wait for the action to return
			// bool finished_before_timeout = ac->wait_for_result(rclcpp::Duration(timeout));
			loop_rate.sleep();
		}
	}
	catch (std::exception &e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
	}

	return 0;
}
