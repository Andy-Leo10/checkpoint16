#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "eigen3/Eigen/Dense"

class AbsoluteMotion : public rclcpp::Node
{
public:
    AbsoluteMotion(float half_length=85.0, float half_wheel_separation=134.845, float radius=50.0)
        : Node("absolute_motion")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&AbsoluteMotion::odom_callback, this, std::placeholders::_1));
        l=half_length;
        w=half_wheel_separation;
        r=radius;
        // Initialize the transformation matrix H
        H << -l-w, 1, -1,
            l+w, 1,  1,
            l+w, 1, -1,
            -l-w, 1,  1;
        H = H / r;
    }

public:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    double phi_ = 0.0, x_ = 0.0, y_ = 0.0;
    double l;  // half of the wheel base distance
    double r;  // the radius of the wheels
    double w;  // half of track width
    Eigen::Matrix<float, 4, 3> H;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, phi_);
        //RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, phi: %.2f", x_, y_, phi_);
    }

    std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy)
    {
        Eigen::Matrix3d R;
        R << 1, 0, 0,
             0, std::cos(phi_), std::sin(phi_),
             0, -std::sin(phi_), std::cos(phi_);
        Eigen::Vector3d v(dphi, dx, dy);
        Eigen::Vector3d twist = R * v;

        RCLCPP_INFO(this->get_logger(), "wz: %.2f, vx: %.2f, vy: %.2f", twist(0), twist(1), twist(2));
        return std::make_tuple(twist(0), twist(1), twist(2));
    }

    std_msgs::msg::Float32MultiArray twist2wheels(double wz, double vx, double vy)
    {
        Eigen::Matrix<float, 3, 1> robot_velocities;
        Eigen::Matrix<float, 4, 1> wheel_velocities;
        
        // let's multiply to get the wheel velocities
        robot_velocities << wz, vx, vy;
        wheel_velocities = H * robot_velocities;

        // transform the wheel velocities into a message
        std_msgs::msg::Float32MultiArray msg;
        //msg.data = {wheel_velocities[2], wheel_velocities[3], wheel_velocities[0], wheel_velocities[1]};
        //msg.data = {wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3]};
        msg.data = {wheel_velocities[3], wheel_velocities[2], wheel_velocities[1], wheel_velocities[0]};
        return msg;
    }

    void stop()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {0.0, 0.0, 0.0, 0.0};
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Stopping");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto rosbotXL = std::make_shared<AbsoluteMotion>();
    rclcpp::WallRate loop_rate(10); // Hz

    std::vector<double> w1, w2, w3, w4, w5, w6, w7, w8;
    w1 = {0.0, 1, -1};
    w2 = {0.0, 1, 1};
    w3 = {0.0, 1, 1};
    w4 = {1.5708, 1, -1};
    w5 = {-3.1415, -1, -1};
    w6 = {0.0, -1, 1};
    w7 = {0.0, -1, 1};
    w8 = {0.0, -1, -1};

    while (rclcpp::ok())
    {
        // get the velocities from method velocity2twist
        // given the desired position and orientation
        std::tuple<double, double, double> twist = rosbotXL->velocity2twist(w1[0], w1[1], w1[2]);
        // transform the velocities into wheel velocities
        std_msgs::msg::Float32MultiArray wheel_velocities = rosbotXL->twist2wheels(std::get<0>(twist), std::get<1>(twist), std::get<2>(twist));
        // publish the wheel velocities
        rosbotXL->publisher_->publish(wheel_velocities);

        rclcpp::spin_some(rosbotXL);
        RCLCPP_INFO(rosbotXL->get_logger(), "Publishing: '%.2f, %.2f, %.2f, %.2f'", wheel_velocities.data[0], wheel_velocities.data[1], wheel_velocities.data[2], wheel_velocities.data[3]);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}