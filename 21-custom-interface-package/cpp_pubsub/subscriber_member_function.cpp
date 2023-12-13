#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp" //NEW
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>( //NEW
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
//NEW
    void topic_callback(const tutorial_interfaces::msg::Num & msg) const 
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'"); 
    }
    rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_; 
//END NEW 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}