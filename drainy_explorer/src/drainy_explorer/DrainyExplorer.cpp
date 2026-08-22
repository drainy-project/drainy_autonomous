#include "drainy_explorer/DrainyExplorer.hpp"

namespace easynav
{
    DrainyExplorer::DrainyExplorer()
    : Node("drainy_explorer")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("prueba", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&DrainyExplorer::timer_callback, this));
    }

    DrainyExplorer::~DrainyExplorer() {}

    void DrainyExplorer::timer_callback(){
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
} // namespace easynav