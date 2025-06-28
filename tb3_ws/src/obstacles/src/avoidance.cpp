#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

using namespace std::chrono_literals;

bool front_obstacle = false;
bool left_obstacle = false;
bool right_obstacle = false;

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg)
{
    front_obstacle = msg->data;
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg)
{
    left_obstacle = msg->data;
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg)
{
    right_obstacle = msg->data;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("avoidance");
	auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subs_front = node->create_subscription<example_interfaces::msg::Bool>("/front/obstacle", 10, callback_front);
    auto subs_left = node->create_subscription<example_interfaces::msg::Bool>("/left/obstacle", 10, callback_left);
    auto subs_right = node->create_subscription<example_interfaces::msg::Bool>("/right/obstacle", 10, callback_right);
	geometry_msgs::msg::Twist message;
	rclcpp::WallRate loop_rate(50ms);
	while (rclcpp::ok()) {
		publisher->publish(message);
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}
