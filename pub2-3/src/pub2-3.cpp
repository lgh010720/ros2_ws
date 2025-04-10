#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class TurtleRotationPublisher : public rclcpp::Node
{
public:
    TurtleRotationPublisher() : Node("turtle_rotation_publisher")
    {
        // 퍼블리셔 생성
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // 주기적인 타이머 설정 (0.5초마다 발행)
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TurtleRotationPublisher::publish_velocity, this));
    }

private:
    void publish_velocity()
    {
        // 거북이가 회전하도록 하는 선속도와 각속도 설정
        geometry_msgs::msg::Twist msg;

        // 선속도 설정 (0.0: 정지, 0.5: 회전 속도)
        msg.linear.x = 0.5;   // 직선으로 나아가는 속도
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        // 각속도 설정 (양수: 시계방향, 음수: 반시계방향 회전)
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 1.0;  // 각속도를 설정하여 회전

        // 메시지 발행
        cmd_vel_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleRotationPublisher>());
    rclcpp::shutdown();
    return 0;
}
