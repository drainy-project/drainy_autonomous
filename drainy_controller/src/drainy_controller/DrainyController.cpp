#include <memory>
#include <vector>

#include "drainy_controller/DrainyController.hpp"


namespace easynav
{
    DrainyController::DrainyController(){}

    DrainyController::~DrainyController(){}

    void DrainyController::on_initialize()
    {

        auto node = get_node();
        const std::string & plugin_name = this->get_plugin_name();

        RCLCPP_INFO(node->get_logger(), "%s plugin has been initialized", plugin_name.c_str());


    }

    void DrainyController::update_rt([[maybe_unused]] NavState & nav_state)
    {
        if(!nav_state.has("robot_pose"))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Robot pose has not been defined in this scope.");
            return;
        }

        // const auto & robot_pose_msg = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
        // const auto & robot_p = robot_pose_msg.pose.pose.position;
        
    }

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DrainyController, easynav::ControllerMethodBase)