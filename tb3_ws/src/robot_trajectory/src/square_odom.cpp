#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

double ini_x = 0.0;
double ini_y = 0.0;
double ini_angle = 0.0;

double global_x = 0.0;
double global_y = 0.0;
double global_angle = 0.0;

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
    global_theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); //conversión de cuaternión a ángulo de orientación
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
    
    int linear_iterations = static_cast<int>((square_length / linear_speed) * 100);
    int angular_iterations = static_cast<int>((turn_angle / (0.01 * angular_speed)));
    
    rclcpp::WallRate loop_rate(10ms);

    for (int j = 0; j < 4; j++)
    {
        ini_x = global_x;
        ini_y = global_y;
        ini_angle = global_angle;
        std::cout << "Initial position: (" << ini_x << ", " << ini_y << ")" << " Initial θ: " << ini_angle << std::endl;
        
        for (int i = 0; i < linear_iterations && rclcpp::ok(); i++) 
        {
            geometry_msgs::msg::Twist message;
            message.linear.x = linear_speed;
            message.angular.z = 0.0;
            publisher->publish(message);
            rclcpp::spin_some(node);
            loop_rate.sleep();
            
            calculate_distance = Distance(initial_x, initial_y, global_x, global_y);
            calculate_angle = global_angle - ini_angle
            std::cout << "distance between the initial and current positions: " << calculate_distance << std::endl;
            std::cout << "distance between the initial and current angle: " << calculate_angle << std::endl;
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
