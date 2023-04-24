#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>
#include <ros2_network_analysis/srv/ping.hpp>
#include <ros2_network_analysis/msg/network_delay.hpp>
#include <iostream>
#include <chrono>
#include <cstdlib>

struct timeval start, end;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ping_client");
    double timeout = 0;
    node->get_parameter_or("timeout_network_delay", timeout, 10.0);
    
    if (timeout <= 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timeout value must be greater than 0");
        return 0;
    }

    int updateRate = 0;
    node->get_parameter_or("update_rate_network_delay", updateRate, 10);
    
    if (updateRate <= 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Update rate value must be greater than 0");
        return 0;
    }

    auto client = node->create_client<ros2_network_analysis::srv::Ping>("network_analysis/ping");
    auto request = std::make_shared<ros2_network_analysis::srv::Ping::Request>();
    ros2_network_analysis::msg::NetworkDelay msg;
    auto delayPub = node->create_publisher<ros2_network_analysis::msg::NetworkDelay>("network_analysis/network_delay", 10);
    request->inp = true;
    rclcpp::Rate rate(updateRate);
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (rclcpp::ok())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Running");
        msg.header.stamp = node->now();
        msg.iface = "default";
        msg.delay = -1; // default values
        msg.alive = 0;  // default values
        double mtime, seconds, useconds;

        gettimeofday(&start, NULL);
        auto response = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, response,std::chrono::seconds((int64_t) timeout*1000)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            gettimeofday(&end, NULL);
            seconds = end.tv_sec - start.tv_sec;
            useconds = end.tv_usec - start.tv_usec;
            mtime = ((seconds)*1000 + useconds / 1000.0) + 0.5;
            msg.delay = (float) mtime;
            msg.alive = 1;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ping response received");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ping response: %d", response.get()->outp);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ping response: %f", mtime);
        }
        else
        {
            msg.delay = -1;
            msg.alive = 0;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ping");
        }
        delayPub->publish(msg);
        rclcpp::spin_some(node);
        rate.sleep();
    }
}