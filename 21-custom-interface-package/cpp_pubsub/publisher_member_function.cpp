#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp" //NEW

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10); //NEW
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
    void timer_callback()
    {
        auto message = tutorial_interfaces::msg::Num(); //NEW
        message.num = this->count_++; //NEW
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'"); //NEW
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_; //NEW
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}