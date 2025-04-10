#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

using std::placeholders::_1;

class TurtleKeyboardPublisher : public rclcpp::Node
{
public:
    TurtleKeyboardPublisher() : Node("turtle_keyboard_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Press [f] forward, [b] backward, [l] left, [r] right, [q] quit");
    }

    void run()
    {
        char c;
        while (rclcpp::ok()) {
            c = getch();
            geometry_msgs::msg::Twist msg;

            switch (c) {
                case 'f':
                    msg.linear.x = 2.0;
                    break;
                case 'b':
                    msg.linear.x = -2.0;
                    break;
                case 'l':
                    msg.angular.z = 2.0;
                    break;
                case 'r':
                    msg.angular.z = -2.0;
                    break;
                case 'q':
                    return;
                default:
                    continue;
            }

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published command for key: %c", c);
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    char getch()
    {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleKeyboardPublisher>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
