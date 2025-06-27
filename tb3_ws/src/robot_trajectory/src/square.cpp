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
    
    node->declare_parameter("square_length", 1.0);
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI / 20);
    node->declare_parameter("turn_angle", M_PI_2);
    
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double turn_angle = node->get_parameter("turn_angle").get_parameter_value().get<double>();
    
    int linear_iterations = static_cast<int>((square_length / linear_speed) * 100);
    int angular_iterations = static_cast<int>((turn_angle / (0.01 * angular_speed)));
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);

    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < linear_iterations && rclcpp::ok(); i++) 
        {
            geometry_msgs::msg::Twist message;
            message.linear.x = linear_speed;
            message.angular.z = 0.0;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        rclcpp::sleep_for(500ms);
        
        for (int i = 0; i < angular_iterations && rclcpp::ok(); i++)
        {
            geometry_msgs::msg::Twist message;
            message.linear.x = 0.0;
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        rclcpp::sleep_for(500ms);
    }
    publisher->publish(message);
    rclcpp::shutdown();
    return 0;
}
