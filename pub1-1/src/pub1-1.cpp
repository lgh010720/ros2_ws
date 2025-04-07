#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"  // 정수 메시지를 위한 헤더 파일
#include <memory>
#include <chrono>

int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv); // ROS 2 노드 초기화

    auto node = std::make_shared<rclcpp::Node>("node_pub1_1"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // QoS 프로필 설정
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1_1", qos_profile); // Int32 타입의 퍼블리셔 생성

    std_msgs::msg::Int32 message; // 메시지 초기화 (정수 0으로 시작)
    message.data = 0;

    rclcpp::WallRate loop_rate(1.0); // 주기 설정 (1Hz)

    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data); // 퍼블리시할 메시지 출력
        mypub->publish(message); // 메시지 퍼블리시
        message.data++; // 메시지 값 증가
        loop_rate.sleep(); // 주기 맞추기 위해 sleep
    }

    rclcpp::shutdown(); // ROS 2 노드 종료
    return 0;
}
