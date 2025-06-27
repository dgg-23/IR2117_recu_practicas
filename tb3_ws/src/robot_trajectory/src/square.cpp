#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("square");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI / 20);
    
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    
    geometry_msgs::msg::String message;
    rclcpp::WallRate loop_rate(10ms);

    for (int j = 0; j < 4; j++)
    {
        int i = 0, n = 1000;
        while (rclcpp::ok() && i < n) 
        {
            message.linear.x = linear_speed;
            message.angular.z = 0.0;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        
        int t = 0, n_turn = 1010;
        while (rclcpp::ok() && t < n_turn)
        {
            t++;
            message.linear.x = 0.0;
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
    }
    publisher->publish(message);
    rclcpp::shutdown();
    return 0;
}
