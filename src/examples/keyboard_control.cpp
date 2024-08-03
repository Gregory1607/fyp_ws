#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_base_controller/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Use WASD keys to move the robot. Press 'q' to quit.");
    }

    char getch()
    {
        char buf = 0;
        struct termios old = {0};
        fflush(stdout);
        if(tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if(tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if(read(0, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if(tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return buf;
    }

    void run()
    {
        char c;
        while (rclcpp::ok())
        {
            c = getch();
            geometry_msgs::msg::TwistStamped twist_stamped;
            twist_stamped.header.stamp = this->now();
            twist_stamped.header.frame_id = "base_link";

            switch (c)
            {
                case 'w':
                    twist_stamped.twist.linear.x = 0.5;
                    break;
                case 's':
                    twist_stamped.twist.linear.x = -0.5;
                    break;
                case 'a':
                    twist_stamped.twist.angular.z = 0.5;
                    break;
                case 'd':
                    twist_stamped.twist.angular.z = -0.5;
                    break;
                case 'q':
                    RCLCPP_INFO(this->get_logger(), "Quitting");
                    return;
                default:
                    continue;
            }
            publisher_->publish(twist_stamped);
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    node->run();
    rclcpp::shutdown();
    return 0;
}

