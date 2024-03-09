#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
    : Node("joint_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this)); 
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        // Add names of the joints you want to control
        message.joint_names.push_back("arm_upper_to_lower_right");
        message.joint_names.push_back("arm_upper_to_lower_left");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        double position1 = 1.5*(1-cos(count_*0.1));
        double position2 = -position1; 
        point.positions.push_back(position1);
        point.positions.push_back(position2);
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", position1, position2);
        count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}