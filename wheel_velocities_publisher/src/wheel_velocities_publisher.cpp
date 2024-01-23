#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class HolonomicController : public rclcpp::Node
{
public:
    HolonomicController(int delay = 5)
        : Node("holonomic_controller"), delay_(delay)
    {
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        RCLCPP_INFO(this->get_logger(), "Initializing basic Holonomic Controller node");
        RCLCPP_INFO(this->get_logger(), "Delay time: %d", delay_);
        
    }

public:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    int delay_;
    std_msgs::msg::Float32MultiArray msg;

    void move_forward()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {1.0, 1.0, 1.0, 1.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        rclcpp::sleep_for(std::chrono::seconds(delay_));
    }

    void move_backward()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {-1.0, -1.0, -1.0, -1.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving backward");
        rclcpp::sleep_for(std::chrono::seconds(delay_));
    }

    void move_left()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {-1.0, 1.0, -1.0, 1.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving left");
        rclcpp::sleep_for(std::chrono::seconds(delay_));
    }

    void move_right()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {1.0, -1.0, 1.0, -1.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving right");
        rclcpp::sleep_for(std::chrono::seconds(delay_));
    }

    void move_clockwise()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {1.0, -1.0, -1.0, 1.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving clockwise");
        rclcpp::sleep_for(std::chrono::seconds(delay_));
    }

    void move_counterclockwise()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {-1.0, 1.0, 1.0, -1.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving counterclockwise");
        rclcpp::sleep_for(std::chrono::seconds(delay_));
    }

    void stop()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {0.0, 0.0, 0.0, 0.0};
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Stop");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HolonomicController>(3);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    node->move_forward();
    node->move_backward();
    node->move_left();
    node->move_right();
    node->move_clockwise();
    node->move_counterclockwise();
    node->stop();
    rclcpp::shutdown();
    return 0;
}

