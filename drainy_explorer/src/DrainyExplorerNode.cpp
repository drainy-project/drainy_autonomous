#include "drainy_explorer/DrainyExplorer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<easynav::DrainyExplorer>());
  rclcpp::shutdown();
  return 0;
}