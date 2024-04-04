#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MultiRobotCommandPublisher : public rclcpp::Node
{
public:
    MultiRobotCommandPublisher()
        : Node("drive_omo")
    {
        // Publisher 선언
        omo1_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omo1/cmd_vel", 10);
        omo2_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omo2/cmd_vel", 10);
        omo3_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omo3/cmd_vel", 10);
        omo4_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/omo4/cmd_vel", 10);

        // 일정 주기로 메시지 발행
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MultiRobotCommandPublisher::publish_commands, this));
    }

private:
    void publish_commands()
    {
        // Twist 메시지 생성
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();

        // 여기서 원하는 움직임 명령 설정
        msg->linear.x = 1.0;  // 전진 속도
        msg->angular.z = 0.5; // 회전 속도

        // 메시지 발행
        omo1_publisher_->publish(*msg);
        omo2_publisher_->publish(*msg);
        omo3_publisher_->publish(*msg);
        omo4_publisher_->publish(*msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr omo1_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr omo2_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr omo3_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr omo4_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiRobotCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
