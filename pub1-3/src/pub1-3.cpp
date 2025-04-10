#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

using std::placeholders::_1;

// 노드 클래스 정의
class TurtleKeyboardPublisher : public rclcpp::Node
{
public:
    TurtleKeyboardPublisher() : Node("turtle_keyboard_publisher") // 노드 이름 설정
    {
        // 퍼블리셔 생성: /turtle1/cmd_vel 토픽에 Twist 메시지를 퍼블리시함
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Press [f] forward, [b] backward, [l] left, [r] right, [q] quit");
    }

    // 키보드 입력을 받아서 메시지를 퍼블리시하는 루프
    void run()
    {
        char c;
        while (rclcpp::ok()) {
            c = getch();  // 키보드에서 문자 입력 받기
            geometry_msgs::msg::Twist msg;  // 빈 Twist 메시지 생성

            // 키에 따라 메시지 값 설정
            switch (c) {
                case 'f':  // 앞으로
                    msg.linear.x = 2.0;
                    break;
                case 'b':  // 뒤로
                    msg.linear.x = -2.0;
                    break;
                case 'l':  // 좌회전
                    msg.angular.z = 2.0;
                    break;
                case 'r':  // 우회전
                    msg.angular.z = -2.0;
                    break;
                case 'q':  // 종료
                    return;
                default:
                    continue;  // 다른 키는 무시
            }

            publisher_->publish(msg);  // 메시지 퍼블리시
            RCLCPP_INFO(this->get_logger(), "Published command for key: %c", c); // 퍼블리시된 키 출력
        }
    }

private:
    // 퍼블리셔 핸들
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // 키보드에서 한 글자 입력받기 (즉시 입력)
    char getch()
    {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);  // 현재 터미널 설정 저장
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);  // 입력을 즉시 받고, 화면에 출력하지 않음
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 새 설정 적용
        ch = getchar();  // 문자 입력 받기
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 원래 설정 복구
        return ch;
    }
};

// 메인 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // ROS2 초기화
    auto node = std::make_shared<TurtleKeyboardPublisher>();  // 노드 생성
    node->run();  // 노드 실행 (키보드 입력 루프)
    rclcpp::shutdown();  // ROS2 종료
    return 0;
}
