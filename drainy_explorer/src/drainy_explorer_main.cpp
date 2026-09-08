#include "drainy_explorer/DrainyExplorer.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exploring_node = easynav::DrainyExplorer::make_shared();
  rclcpp::spin(exploring_node);

  rclcpp::shutdown();
  return 0;
}