#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>

class KinematicModel : public rclcpp::Node
{
public:
    KinematicModel(float half_length=85.0, float half_wheel_separation=134.845, float radius=50.0)
        : Node("kinematic_model")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "wheel_speed", 10, std::bind(&KinematicModel::callback, this, std::placeholders::_1));
        l_=half_length;
        w_=half_wheel_separation;
        r_=radius;
        // Initialize the transformation matrix M
        M_ << 1,1,1,1,
              1,-1,-1,1,
              -1/(l_+w_),1/(l_+w_),-1/(l_+w_),1/(l_+w_);
        M_ = r_/4*M_;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    // 4 wheel omnidirectional robot dimensions
    float l_;
    float w_;
    float r_; // wheel radius
    // Transformation matrix from wheels velocities to robot velocities
    Eigen::Matrix<float, 3, 4> M_;

    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        Eigen::Matrix<float, 3, 1> robot_velocities;
        // Transform the four wheel speeds into a twist message
        // [robot_velocities]=M*[wheel_velocities] 
        // [3x1]=[3x4]*[4x1]
        Eigen::Matrix<float, 4, 1> input_raw;
        Eigen::Matrix<float, 4, 1> wheel_velocities;
        input_raw = Eigen::Map<const Eigen::Matrix<float, 4, 1>>(msg->data.data());
        wheel_velocities << input_raw[0], 
                            input_raw[1], 
                            input_raw[3],
                            input_raw[2];
        robot_velocities = M_ * wheel_velocities;
        RCLCPP_INFO(this->get_logger(), "Linear: %f, %f, %f", robot_velocities[0], robot_velocities[1], robot_velocities[2]);

        twist.linear.x = robot_velocities[0];
        twist.linear.y = -robot_velocities[1];
        twist.angular.z = robot_velocities[2];

        publisher_->publish(twist);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicModel>());
    rclcpp::shutdown();
    return 0;
}