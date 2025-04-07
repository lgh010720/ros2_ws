#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"  // Vector3 메시지 인터페이스 헤더
#include <memory>
#include <chrono>
#include <iostream>  // cin, cout을 사용하기 위한 헤더

int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv); // ROS 2 노드 초기화

    auto node = std::make_shared<rclcpp::Node>("node_pub1_2"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // QoS 프로필 설정
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile); // Vector3 타입의 퍼블리셔 생성
    
    geometry_msgs::msg::Vector3 message; // 메시지 초기화

    rclcpp::WallRate loop_rate(1.0); // 반복 주기 설정 (1Hz)

    while (rclcpp::ok())
    {
        std::cout << "실수1 입력: "; 
        std::cin >> message.x ;
        std::cout << "실수2 입력: "; 
        std::cin >> message.y ;
        std::cout << "실수3 입력: "; 
        std::cin >> message.z ;
        std::cout << "Publishing: 1 = " << message.x << ", 2 = " << message.y << ", 3 = " << message.z << std::endl; // 입력받은 값 출력
        mypub->publish(message); // 메시지 퍼블리시
        loop_rate.sleep(); // 주기 맞추기 위해 sleep
    }

    
    rclcpp::shutdown(); // ROS 2 노드 종료
    return 0;
}
