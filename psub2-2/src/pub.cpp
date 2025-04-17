#include "psub2-2/pub.hpp"

Pub::Pub() : Node("mypub"), count_(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile);
    timer_ = this->create_wall_timer(1s, std::bind(&Pub::publish_msg, this));
}

void Pub::publish_msg()
{
    static int count = 0;
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = count;
    msg.y = count;
    msg.z = count++;
    RCLCPP_INFO(this->get_logger(), "Published message: x:'%lf', y:'%lf', z:'%lf'", msg.x, msg.y, msg.z);
    pub_->publish(msg);
}