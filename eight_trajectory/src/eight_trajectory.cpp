#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "eigen3/Eigen/Dense"

class AbsoluteMotion : public rclcpp::Node
{
public:
    AbsoluteMotion(float half_length=85.0, float half_wheel_separation=134.845, float radius=50.0, double tolerance=0.04)
        : Node("absolute_motion")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&AbsoluteMotion::odom_callback, this, std::placeholders::_1));
        l=half_length;
        w=half_wheel_separation;
        r=radius;
        threshold = tolerance;
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
    double threshold; // allowed error in position goal

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
        // RCLCPP_INFO(this->get_logger(), "ODOM_READ ~~~~~~ x: %.2f, y: %.2f, phi: %.2f", x_, y_, phi_);
    }

    std::tuple<double, double, double> velocity2twist(double dphi, double dx, double dy)
    {
        Eigen::Matrix3d R;
        R << 1, 0, 0,
             0, std::cos(phi_), std::sin(phi_),
             0, -std::sin(phi_), std::cos(phi_);
        Eigen::Vector3d v(dphi, dx, dy);
        Eigen::Vector3d twist = R * v;

        // RCLCPP_INFO(this->get_logger(), "wz: %.2f, vx: %.2f, vy: %.2f", twist(0), twist(1), twist(2));
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

        // there might be a method that invert the order of the wheels
        // for that reason I am inverting the order of the wheels
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

    void move_to_target(double desired_phi, double desired_x, double desired_y)
    {
        // Calculate the difference in odometry
        double dphi = M_PI * desired_phi / 180.0 - phi_;
        double dx = desired_x - x_;
        double dy = desired_y - y_;

        while (std::abs(dx) > threshold || std::abs(dy) > threshold || std::abs(dphi) > threshold)
        {
            std::tuple<double, double, double> twist = velocity2twist(dphi, dx, dy);
            std_msgs::msg::Float32MultiArray u = twist2wheels(std::get<0>(twist), std::get<1>(twist), std::get<2>(twist));

            std_msgs::msg::Float32MultiArray msg;
            msg.data = u.data;
            publisher_->publish(msg);

            rclcpp::spin_some(shared_from_this()); // Process callbacks
            rclcpp::sleep_for(std::chrono::milliseconds(25)); // odometry is published at 10-12Hz

            // Update the difference in odometry
            dphi = M_PI * desired_phi / 180.0 - phi_;
            dx = desired_x - x_;
            dy = desired_y - y_;
            // RCLCPP_INFO(this->get_logger(), "DIFFERENCE ---> x: %.2f, y: %.2f, phi: %.2f", x_, y_, phi_);
        }
        this->stop();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto rosbotXL = std::make_shared<AbsoluteMotion>();
    rclcpp::WallRate loop_rate(10); // Hz

    std::vector<double> w1, w2, w3, w4, w5, w6, w7, w8;
    //   angle, x, y
    w1 = {0.0, 1.0, -1.0};
    w2 = {0.0, 2.0, 0.0};
    w3 = {0.0, 3.0, 1.0};
    w4 = {90.0, 4.0, 0.0};
    w5 = {-90.0, 3.0, -1.0};
    w6 = {-90.0, 2.0, 0.0};
    w7 = {-90.0, 1.0, 1.0};
    w8 = {0.0, 0.0, 0.0};
    std::vector<std::vector<double>> w = {w1, w2, w3, w4, w5, w6, w7, w8};

    // // Spin the node in a separate thread
    // std::thread spin_thread([&]() {
    //     while (rclcpp::ok()) {
    //         rclcpp::spin_some(rosbotXL);
    //         loop_rate.sleep();
    //     }
    // });

    for (auto i : w)
    {
        RCLCPP_INFO(rosbotXL->get_logger(), "Moving to ---> phi: %.2f, x: %.2f, y: %.2f", i[0], i[1], i[2]);
        rosbotXL->move_to_target(i[0], i[1], i[2]);
    }

    // // Wait for the spin thread to finish
    // spin_thread.join();

    rclcpp::shutdown();
    return 0;
}