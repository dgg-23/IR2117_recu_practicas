#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>

using namespace std::chrono_literals;

double ini_x = 0.0;
double ini_y = 0.0;
double ini_a = 0.0; //angulo inicial

double global_x = 0.0;
double global_y = 0.0;
double global_a = 0.0;

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    //obtenemos las coordenadas x e y del mensaje de odom
    global_x = msg->pose.pose.position.x;
    global_y = msg->pose.pose.position.y;
    
    //obtenemos el angulo de theta del mensaje odom
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    global_a = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); //conversión de cuaternión a ángulo de orientación
}

double Distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("square_odom");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odomCallback);
    
    node->declare_parameter("square_length", 1.0);
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI / 20);
    node->declare_parameter("turn_angle", M_PI_2);
    
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double turn_angle = node->get_parameter("turn_angle").get_parameter_value().get<double>();
    
    double distance_travelled = 0.0;
    double target_distance = square_length;
    double angle_turned = 0.0;
    double target_angle = M_PI_2;

    int linear_iterations = static_cast<int>((square_length / linear_speed) * 100);
    int angular_iterations = static_cast<int>((turn_angle / (0.01 * angular_speed)));
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);
    
    double calculate_distance = 0.0;
    double calculate_angle = 0.0;

    for (int j = 0; j < 4; j++)
    {
        distance_travelled = 0.0;
        angle_turned = 0.0;
        ini_x = global_x;
        ini_y = global_y;
        ini_a = global_a;
        std::cout << "Initial position: (" << ini_x << ", " << ini_y << ")" << " Initial θ: " << ini_a << std::endl;
        //para avanzar
        while ((rclcpp::ok()) && (distance_travelled < target_distance)) 
        {
            message.linear.x = linear_speed;
            message.angular.z = 0.0;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
            distance_travelled = Distance(ini_x, ini_y, global_x, global_y);
            calculate_distance = Distance(ini_x, ini_y, global_x, global_y);
            calculate_angle = global_a - ini_a;
        }

        //para girar
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher->publish(message);
        ini_a = global_a;
        
        while ((rclcpp::ok()) && (angle_turned < target_angle))
        {
            message.linear.x = 0.0;
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();

            angle_turned = std::fmod(std::abs(global_a - ini_a), 2 * M_PI);
            if (angle_turned > M_PI)
            {
                angle_turned = 2 * M_PI - angle_turned;
            }
            std::cout << "Angle turned: " << angle_turned << std::endl;
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
    }

    rclcpp::shutdown();
    return 0;
}
