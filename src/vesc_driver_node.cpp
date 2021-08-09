#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "vesc_driver/vesc_driver.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


int main(int argc, char ** argv)
{
    
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vesc_driver::VescDriver>());
  rclcpp::shutdown();

  return 0;
}
