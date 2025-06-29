#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using turtlesim::srv::SetPen;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings");

  //parametro radius
  node->declare_parameter("radius", 1.0);
  double radius = node->get_parameter("radius").get_parameter_value().get<double>();

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  geometry_msgs::msg::Twist message;
  rclcpp::WallRate loop_rate(500ms);

  double linear_speed = 1.0;
  double angular_speed = 1.0 / radius;
  int size = static_cast<int>(radius * 20.0); //para modificar el tiempo que tiene que estar activo en funcion del radio

  auto client_pen = node->create_client<SetPen>("/turtle1/set_pen");
  client_pen->wait_for_service();

  //dibuja el circulo
  for (int i = 0; i < size; i++)
  {
    message.linear.x = linear_speed;
    message.angular.z = angular_speed;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  //stop
  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);

  rclcpp::shutdown();
  return 0;
}

