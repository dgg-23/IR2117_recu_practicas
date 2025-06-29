#include <chrono>
#include <cmath>
#include <array>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using turtlesim::srv::SetPen;
using turtlesim::srv::TeleportAbsolute;

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

   //cambiar color
  auto client_pen = node->create_client<SetPen>("/turtle1/set_pen");
  client_pen->wait_for_service();
  //teletransportar
  auto client_teleport = node->create_client<TeleportAbsolute>("/turtle1/teleport_absolute");
  client_teleport->wait_for_service();

  std::array<std::array<float, 2>, 3> positions = {{
      {3.3f, 5.5f},  // Azul
      {5.5f, 5.5f},  // Negro
      {7.7f, 5.5f},  // Rojo
      {4.4f, 4.0f}   // amarillo
  }};

  std::array<std::array<int, 3>, 3> colors = {{
      {0, 0, 255},   // Azul
      {0, 0, 0},     // Negro
      {255, 0, 0}    // Rojo
      {255, 255, 0}  // Amarillo
  }};

  auto setpen = std::make_shared<SetPen::Request>();
  setpen->width = 3;

  auto teleport = std::make_shared<TeleportAbsolute::Request>();
  teleport->theta = 0.0;

  for (int i = 0; i < 4; i++)
  {
    //apagar lapiz
    request_setpen->off = 1;
    setpen->r = colors[i][0];
    setpen->g = colors[i][1];
    setpen->b = colors[i][2];
    client_pen->async_send_request(request_setpen);
    std::this_thread::sleep_for(200ms);

    //tp al segundo circulo + encender lapiz
    teleport->x = positions[i][0];
    teleport->y = positions[i][1];;
    client_teleport->async_send_request(request_teleport);
    std::this_thread::sleep_for(1s);

    //encender lapiz
    request_setpen->off = 0;
    client_pen->async_send_request(request_setpen);
    std::this_thread::sleep_for(100ms);

    //dibujar circulo
    for (int i = 0; i < size; i++)
    {
      message.linear.x = linear_speed;
      message.angular.z = angular_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
  }

  //stop
  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);

  rclcpp::shutdown();
  return 0;
}

