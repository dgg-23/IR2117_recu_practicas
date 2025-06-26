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
    geometry_msgs::msg::String message;
    rclcpp::WallRate loop_rate(10ms);

    for (int j = 0; j < 4; j++)
    {
        int i = 0, n = 1000;
        while (rclcpp::ok() && i < n) 
        {
            message.linear.x = 0.1;
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
            message.angular.z = -(9 * M_PI / 180);
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
