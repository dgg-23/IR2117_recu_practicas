#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/string.hpp"
#include <cmath>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
    geometry_msgs::msg::String message;
    rclcpp::WallRate loop_rate(10ms);

    int i=0, n=1000; 
    while(rclcpp::ok() && i<n)
    {
        i++;
        message.linear.x = 0.1;
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    int j = 0, n_turn = 1011;
    while(rclcpp::ok() && j < n_turn)
    {
       j++;
       message.linear.x = 0.0;
       message.angular.z = -(9*M_PI/180);
       publisher->publish(message);
       rclcpp::spin_some(node);
       loop_rate.sleep();
    }
    
    //send zero velocity to topic
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
    rclcpp::shutdown();
    return 0;
}
