#include "rclcpp/utilities.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("teste"), "ROS 2 respondendo");
  rclcpp::shutdown();
  return 0;
}
