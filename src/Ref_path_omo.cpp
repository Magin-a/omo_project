#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Ref_path : public rclcpp::Node
{
public:
    Ref_path()
        : Node("Ref_path")
    {
        // "path" 토픽을 구독하는 subscription 생성
        global_path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "path", 10,
            std::bind(&Ref_path::global_path_callback, this, std::placeholders::_1)
        );

        // "ref_path" 토픽을 발행하는 publisher 생성
        ref_path_pub = this->create_publisher<nav_msgs::msg::Path>("ref_path", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub;
    
    std::mutex buf;

    std::vector<nav_msgs::msg::Path> global_q;
    std::vector<nav_msgs::msg::Path> ref_q;
    
    // "path" 토픽에서 메시지를 수신할 때 호출되는 콜백 함수
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        buf.lock();
        global_q.emplace_back(*msg);
        buf.unlock();
    }

    void Ref_path_update()
    {
    if (global_q.empty())
        return;

    std::vector<nav_msgs::msg::Path> paths;
    buf.lock();
    std::swap(paths, global_q);
    buf.unlock();

    size_t index = 0;
    while (index < paths.size())
    {
        // 다음 5개의 인덱스 묶음을 추출하여 ref_q에 추가
        ref_q.emplace_back();
        for (size_t i = 0; i < 5 && index < paths.size(); ++i)
        {
            ref_q.back().header = paths[index].header;
            ref_q.back().poses.insert(ref_q.back().poses.end(), paths[index].poses.begin(), paths[index].poses.end());
            ++index;
        }
    }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ref_path>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
