#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "auto_aim_interfaces/msg/send_data.hpp"
#include <rclcpp/logging.hpp>


class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        // 创建发布者
        send_data_publisher_ = this->create_publisher<auto_aim_interfaces::msg::SendData>("send_data", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        auto_aim_interfaces::msg::SendData message;
        message.header.stamp = this->now();
        message.pitch = 3.0;
        message.yaw = 2.0;
        message.position.x = 1.0;
        message.position.y = 2.0;
        message.position.z = 3.0;
        // 发布消息
        send_data_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<auto_aim_interfaces::msg::SendData> ::SharedPtr send_data_publisher_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}