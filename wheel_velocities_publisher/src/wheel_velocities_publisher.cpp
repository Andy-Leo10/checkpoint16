// humble

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

class EigenTestNode : public rclcpp::Node
{
public:
    EigenTestNode()
        : Node("eigen_test_node")
    {
        // Create a 3x3 matrix
        Eigen::Matrix3f m = Eigen::Matrix3f::Random();

        // Create a 3x1 vector
        Eigen::Vector3f v = Eigen::Vector3f::Random();

        // Perform a matrix-vector multiplication
        Eigen::Vector3f result = m * v;

        // Print the result
        RCLCPP_INFO(this->get_logger(), "The result of the multiplication is:\n\n %f, %f, %f", result[0], result[1], result[2]);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EigenTestNode>());
    rclcpp::shutdown();
    return 0;
}
