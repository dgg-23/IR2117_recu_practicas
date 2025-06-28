#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

float min_range_l = std::numeric_limits<float>::max();
float min_range_d = std::numeric_limits<float>::max();

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) //min ranges
{
    min_range_l = std::numeric_limits<float>::max(); //min ranges[0..9]
    for (int i = 0; i < 10; ++i) {
        if (msg->ranges[i] < min_range_l)
        {
            min_range_l = msg->ranges[i];
        }
    }

    min_range_r = std::numeric_limits<float>::max(); //min ranges[350..359]

    for (int i = 350; i < 360; ++i)
    {
        if (msg->ranges[i] < min_range_r)
        {
            min_range_r = msg->ranges[i];
        }
    }

    std::cout << "Mínimo [0..9]: " << min_range_l << std::endl;
    std::cout << "Mínimo [350..359]: " << min_range_r << std::endl;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher_cmd_vel = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    geometry_msgs::msg::Twist message_cmd_vel;

    message_cmd_vel.linear.x = 0.5;
    message_cmd_vel.angular.z = 0.0;

    bool obs_detectado = false;

    MinRanges(msg);

    auto subscription_scan = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, [&](const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        for (int i = 0; i < 18; i++)
        {
            if (msg->ranges[i] < 1.0)
            {
                obs_detectado = true;
                break;
            }
        }
    });

    rclcpp::WallRate loop_rate(10ms);

    while(rclcpp::ok() && not obs_detectado)
    {
        publisher_cmd_vel->publish(message_cmd_vel);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    message_cmd_vel.linear.x = 0.0;
    message_cmd_vel.angular.z = 0.0;
    publisher_cmd_vel->publish(message_cmd_vel);

    while (rclcpp::ok())
    {
        if (obs_detectado) 
        {
            if (min_range_l > min_range_d) //gira izq
            {
                message_cmd_vel.linear.x = 0.0;
                message_cmd_vel.angular.z = 0.5;
            }
        }
        else //gira derecha
        {
            message_cmd_vel.linear.x = 0.5;
            message_cmd_vel.angular.z = 0.0;
        }
        publisher_cmd_vel->publish(message_cmd_vel);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}