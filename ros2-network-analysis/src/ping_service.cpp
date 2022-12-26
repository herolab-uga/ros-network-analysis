#include <rclcpp/rclcpp.hpp>
#include <ros2_network_analysis/srv/ping.hpp>
#include <ros2_network_analysis/msg/network_delay.hpp>




void respond(std::shared_ptr<ros2_network_analysis::srv::Ping::Request> request, 
            std::shared_ptr<ros2_network_analysis::srv::Ping::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ping service received request");
    if(request->inp)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending Ping response");
        response->outp = true;
    }
    else
    {   
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ping request cancelled");
        response->outp = false;
    }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ping_service");
  auto service = node->create_service<ros2_network_analysis::srv::Ping>("network_analysis/ping", &respond);
  while(rclcpp::ok()){
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}