#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

void callback(rclcpp::Node::SharedPtr node,
              rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub)
{
    float x, y, z;

    // 사용자로부터 실수값 3개를 입력받음
    std::cout << "x값 입력: ";
    std::cin >> x;
    std::cout << "y값 입력: ";
    std::cin >> y;
    std::cout << "z값 입력: ";
    std::cin >> z;

    // 메시지 생성 및 데이터 설정
    auto message = geometry_msgs::msg::Vector3();
    message.x = x;
    message.y = y;
    message.z = z;

    // 퍼블리시
    RCLCPP_INFO(node->get_logger(), "Publishing: x=%f, y=%f, z=%f", message.x, message.y, message.z);
    pub->publish(message);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("vector3_publisher_node");

    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 퍼블리셔 생성
    auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("vector3_topic", qos_profile);

    // 콜백 함수 설정
    std::function<void()> fn = std::bind(callback, node, pub);

    // 타이머를 사용하여 주기적으로 콜백 실행
    auto timer = node->create_wall_timer(100ms, fn);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
