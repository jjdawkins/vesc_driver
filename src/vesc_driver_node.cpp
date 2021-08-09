#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "vesc_driver/vesc_driver.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char ** argv)
{
    
  rclcpp::init(argc, argv);
 // auto vesc_node = std::make_shared<vesc_driver::VescDriver>();  
 // rclcpp::spin(vesc_node);
   std::cout<<"Start Spin"<<std::endl;
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  //rclcpp::spin(std::make_shared<vesc_driver::VescDriver>());
  rclcpp::shutdown();

  return 0;
}
