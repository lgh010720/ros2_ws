
#ifndef _PUB_HPP_
#define _PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"  // 정수 메시지를 위한 헤더 파일
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class Pub : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    size_t count_;
    void publish_msg();
    
public:
    Pub();
};

#endif //_PUB_HPP_